/* M33'ün TELEM paketlerini ve F746'nın satırlarını /dev/rpmsg0'dan okur, UDP ile PC'ye yollar: TELEM <port>'a,
 * satırlar <port> + 1'e. PC'nin <port> + 1'e yolladığı komut baytlarını M33'e verir.
 * Kullanım: telem_udp <PC IP> <port> [aygıt]
 * Olay bekler, yoklamaz: aygıt, sinyal ve 1 saniyelik zamanlayıcı tek poll() içinde. Çıkış: 0 düzgün, 1 kurulum, 2 aygıt kapandı. */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

#define PKT_SIZE    35                               // TELEM paketi, bayt (M9 sonrası: + M33 süresi)
#define LINE_TYPE   'L'                              // satır mesajının ilk baytı
#define CMD_TYPE    'C'                              // M33'e giden komut mesajının ilk baytı
#define CMD_MAX     480                              // bir datagramdaki komut baytı (RPMsg 496)
#define LINE_HEADER 4                                // tür, bayrak, sıra (2 bayt)
#define BUF_SIZE    512                              // RPMsg tamponu; küçük olsaydı fazlası sessizce kesilirdi
#define RENEW_S     1                                // '1' yenileme aralığı; M33 3 saniye yenilenmezse durur
#define CMD_START   "1"
#define CMD_STOP    "0"

enum { FD_DEV, FD_SIGNAL, FD_TIMER, FD_LINE, FD_COUNT };

typedef struct
{
    unsigned long received;                          // aygıttan okunan paket
    unsigned long sent;                              // UDP'ye verilen paket
    unsigned long stale;                             // açılışta atılan eski paket
    unsigned long lines;                             // UDP'ye verilen satır mesajı
    unsigned long line_errors;                       // UDP'nin reddettiği satır mesajı
    unsigned long commands;                          // M33'e verilen komut mesajı
    unsigned long command_errors;                    // M33'ün almadığı ya da CMD_MAX'tan uzun komut
    unsigned long bad_len;                           // TELEM ya da satır olmayan mesaj
    unsigned long send_errors;                       // UDP'nin reddettiği paket (PC dinlemiyor, hat kapalı)
    unsigned long renew_errors;                      // M33'e verilemeyen '1'
    long long lat_min;                               // Linux süresi, ns: poll uyandı -> send bitti (yollananlar)
    long long lat_max;
    long long lat_sum;
} Counters;

/* Linux'un tekdüze saati, ns. */
static long long now_ns(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);

    return t.tv_sec * 1000000000LL + t.tv_nsec;
}

/* Hata mesajını yazar; kurulum hatası kodunu döndürür. */
static int fail(const char *what)
{
    fprintf(stderr, "telem_udp: %s: %s\n", what, strerror(errno));

    return 1;
}

/* Ondalık portu okur; 1..65535 dışında 0 döndürür. */
static uint16_t parse_port(const char *text)
{
    char *end;

    errno = 0;
    unsigned long value = strtoul(text, &end, 10);

    if (errno != 0 || end == text || *end != '\0' || value > 65535)
    {
        return 0;
    }

    return (uint16_t)value;
}

/* PC'ye bağlı UDP soketi açar, 'local' sıfır değilse o porta bağlar; hata -1. */
static int open_socket(const char *ip, uint16_t port, uint16_t local)
{
    struct sockaddr_in to = { .sin_family = AF_INET, .sin_port = htons(port) };

    if (inet_pton(AF_INET, ip, &to.sin_addr) != 1)
    {
        errno = EINVAL;
        return -1;
    }

    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (fd < 0)
    {
        return -1;
    }

    struct sockaddr_in from = { .sin_family = AF_INET, .sin_port = htons(local) };

    // Bağlı soket yalnız PC'nin bu portundan geleni alır
    if ((local != 0 && bind(fd, (struct sockaddr *)&from, sizeof(from)) != 0)
        || connect(fd, (struct sockaddr *)&to, sizeof(to)) != 0)
    {
        int saved = errno;

        close(fd);
        errno = saved;
        return -1;
    }

    return fd;
}

/* Ctrl+C ve kill'i dosya olayına çevirir; hata -1. */
static int open_signals(void)
{
    sigset_t set;

    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);                         // ssh koparsa da düzgün kapan

    if (sigprocmask(SIG_BLOCK, &set, NULL) != 0)
    {
        return -1;
    }

    return signalfd(-1, &set, SFD_CLOEXEC | SFD_NONBLOCK);
}

/* Saniyede bir dolan zamanlayıcı; hata -1. */
static int open_timer(void)
{
    struct itimerspec every = { .it_interval.tv_sec = RENEW_S, .it_value.tv_sec = RENEW_S };
    int fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK);

    if (fd >= 0 && timerfd_settime(fd, 0, &every, NULL) != 0)
    {
        int saved = errno;

        close(fd);
        errno = saved;
        return -1;
    }

    return fd;
}

/* Açılışta çekirdekte birikmiş eski paketleri atar. */
static void discard_stale(int dev, Counters *n)
{
    uint8_t buf[BUF_SIZE];

    while (read(dev, buf, sizeof(buf)) > 0)
    {
        ++n->stale;
    }
}

/* Bekleyen bütün paketleri UDP'ye verir, Linux süresini yazar; aygıt çalışıyorsa 1, kapandıysa 0. */
static int forward(int dev, int sock, int line_sock, Counters *n, long long woke)
{
    for (;;)
    {
        uint8_t buf[BUF_SIZE];
        ssize_t len = read(dev, buf, sizeof(buf));   // her okuma tam bir RPMsg mesajı

        if (len < 0)
        {
            return errno == EAGAIN || errno == EINTR; // EAGAIN: bekleyen paket kalmadı
        }

        if (len == 0)
        {
            return 0;
        }

        if (len > LINE_HEADER && buf[0] == LINE_TYPE)
        {
            if (send(line_sock, buf, (size_t)len, 0) == len)
            {
                ++n->lines;
            }
            else
            {
                ++n->line_errors;
            }

            continue;
        }

        if (len != PKT_SIZE)
        {
            ++n->bad_len;
            continue;
        }

        ++n->received;

        if (send(sock, buf, PKT_SIZE, 0) == PKT_SIZE)
        {
            long long d = now_ns() - woke;           // aynı uyanışta birden çok paket varsa sonrakiler bekleyişi de taşır

            if (n->sent == 0 || d < n->lat_min)
            {
                n->lat_min = d;
            }

            if (d > n->lat_max)
            {
                n->lat_max = d;
            }

            n->lat_sum += d;
            ++n->sent;
        }
        else
        {
            ++n->send_errors;                        // canlı veri: tekrar denenmez, sayılır
        }
    }
}

/* Bekleyen bütün komut datagramlarını M33'e verir. */
static void take_commands(int dev, int line_sock, Counters *n)
{
    for (;;)
    {
        uint8_t buf[1 + CMD_MAX + 1];
        ssize_t len = recv(line_sock, &buf[1], CMD_MAX + 1, 0);

        if (len < 0)
        {
            if (errno == ECONNREFUSED || errno == EINTR)
            {
                continue;                            // PC dinlemiyordu: bekleyen soket hatası temizlendi
            }

            return;                                  // EAGAIN: kalmadı
        }

        buf[0] = CMD_TYPE;

        if (len >= 1 && len <= CMD_MAX && write(dev, buf, (size_t)len + 1) == len + 1)
        {
            ++n->commands;
        }
        else
        {
            ++n->command_errors;                     // PC cevapsız komutu yeniden dener
        }
    }
}

/* Olay döngüsü; 0 sinyalle düzgün bitiş, 2 aygıt kapandı. */
static int run(struct pollfd *fds, int sock, int line_sock, Counters *n)
{
    for (;;)
    {
        if (poll(fds, FD_COUNT, -1) < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }

            return 2;
        }

        long long woke = now_ns();                    // Linux süresinin başı

        if ((fds[FD_DEV].revents & POLLIN) && !forward(fds[FD_DEV].fd, sock, line_sock, n, woke))
        {
            return 2;
        }

        if (fds[FD_DEV].revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            return 2;                                // M33 durdu ya da yeniden başlıyor
        }

        if (fds[FD_TIMER].revents & POLLIN)
        {
            uint64_t ticks;

            if (read(fds[FD_TIMER].fd, &ticks, sizeof(ticks)) == (ssize_t)sizeof(ticks)
                && write(fds[FD_DEV].fd, CMD_START, 1) != 1)
            {
                ++n->renew_errors;
            }
        }

        if (fds[FD_LINE].revents & (POLLIN | POLLERR))
        {
            take_commands(fds[FD_DEV].fd, line_sock, n);
        }

        if (fds[FD_SIGNAL].revents & POLLIN)
        {
            return 0;
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4)
    {
        fprintf(stderr, "kullanim: telem_udp <PC IP> <port> [aygit]\n");
        return 1;
    }

    const char *path = argc == 4 ? argv[3] : "/dev/rpmsg0";
    uint16_t port = parse_port(argv[2]);

    if (port == 0 || port == UINT16_MAX)             // satırlar port + 1'den gider
    {
        fprintf(stderr, "telem_udp: port 1..65534 olmali: %s\n", argv[2]);
        return 1;
    }

    int sock = open_socket(argv[1], port, 0);
    if (sock < 0)
    {
        return fail(argv[1]);
    }

    int line_sock = open_socket(argv[1], (uint16_t)(port + 1), (uint16_t)(port + 1));
    if (line_sock < 0)
    {
        return fail(argv[1]);
    }

    int signals = open_signals();
    if (signals < 0)
    {
        return fail("sinyal");
    }

    int timer = open_timer();
    if (timer < 0)
    {
        return fail("zamanlayici");
    }

    int dev = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (dev < 0)
    {
        return fail(path);
    }

    Counters n = { 0 };

    discard_stale(dev, &n);

    if (write(dev, CMD_START, 1) != 1)
    {
        return fail(path);
    }

    struct pollfd fds[FD_COUNT] = {
        [FD_DEV] = { .fd = dev, .events = POLLIN },
        [FD_SIGNAL] = { .fd = signals, .events = POLLIN },
        [FD_TIMER] = { .fd = timer, .events = POLLIN },
        [FD_LINE] = { .fd = line_sock, .events = POLLIN },
    };

    int result = run(fds, sock, line_sock, &n);

    // Okuyan kalmıyor: M33 dursun. Aygıt kapandıysa bu yazma zaten başarısız olur
    if (write(dev, CMD_STOP, 1) != 1 && result == 0)
    {
        ++n.renew_errors;
    }

    fprintf(stderr, "telem_udp: alinan %lu yollanan %lu eski %lu uzunluk %lu udp_hatasi %lu yenileme_hatasi %lu cikis %d"
            " linux_us en_az %.1f ort %.1f en_cok %.1f satir %lu satir_hatasi %lu komut %lu komut_hatasi %lu\n",
            n.received, n.sent, n.stale, n.bad_len, n.send_errors, n.renew_errors, result,
            n.lat_min / 1e3, n.sent ? n.lat_sum / 1e3 / n.sent : 0.0, n.lat_max / 1e3,
            n.lines, n.line_errors, n.commands, n.command_errors);

    return result;
}
