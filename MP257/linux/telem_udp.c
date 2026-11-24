/* M33'ün TELEM paketlerini /dev/rpmsg0'dan okur, UDP ile PC'ye yollar. Kullanım: telem_udp <PC IP> <port> [aygıt]
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
#include <unistd.h>

#define PKT_SIZE    33                               // TELEM paketi, bayt
#define BUF_SIZE    512                              // RPMsg tamponu; küçük olsaydı fazlası sessizce kesilirdi
#define RENEW_S     1                                // '1' yenileme aralığı; M33 3 saniye yenilenmezse durur
#define CMD_START   "1"
#define CMD_STOP    "0"

enum { FD_DEV, FD_SIGNAL, FD_TIMER, FD_COUNT };

typedef struct
{
    unsigned long received;                          // aygıttan okunan paket
    unsigned long sent;                              // UDP'ye verilen paket
    unsigned long stale;                             // açılışta atılan eski paket
    unsigned long bad_len;                           // 33 bayt olmayan mesaj
    unsigned long send_errors;                       // UDP'nin reddettiği paket (PC dinlemiyor, hat kapalı)
    unsigned long renew_errors;                      // M33'e verilemeyen '1'
} Counters;

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

/* PC'ye bağlı UDP soketi açar; hata -1. */
static int open_socket(const char *ip, uint16_t port)
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

    if (connect(fd, (struct sockaddr *)&to, sizeof(to)) != 0)
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

/* Bekleyen bütün paketleri UDP'ye verir; aygıt çalışıyorsa 1, kapandıysa 0. */
static int forward(int dev, int sock, Counters *n)
{
    for (;;)
    {
        uint8_t buf[BUF_SIZE];
        ssize_t len = read(dev, buf, sizeof(buf));   // her okuma tam bir RPMsg mesajı

        if (len < 0)
        {
            return errno == EAGAIN || errno == EINTR; // EAGAIN: bekleyen paket kalmadı
        }

        if (len != PKT_SIZE)
        {
            if (len == 0)
            {
                return 0;
            }

            ++n->bad_len;
            continue;
        }

        ++n->received;

        if (send(sock, buf, PKT_SIZE, 0) == PKT_SIZE)
        {
            ++n->sent;
        }
        else
        {
            ++n->send_errors;                        // canlı veri: tekrar denenmez, sayılır
        }
    }
}

/* Olay döngüsü; 0 sinyalle düzgün bitiş, 2 aygıt kapandı. */
static int run(struct pollfd *fds, int sock, Counters *n)
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

        if ((fds[FD_DEV].revents & POLLIN) && !forward(fds[FD_DEV].fd, sock, n))
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

    if (port == 0)
    {
        fprintf(stderr, "telem_udp: port 1..65535 olmali: %s\n", argv[2]);
        return 1;
    }

    int sock = open_socket(argv[1], port);
    if (sock < 0)
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
    };

    int result = run(fds, sock, &n);

    // Okuyan kalmıyor: M33 dursun. Aygıt kapandıysa bu yazma zaten başarısız olur
    if (write(dev, CMD_STOP, 1) != 1 && result == 0)
    {
        ++n.renew_errors;
    }

    fprintf(stderr, "telem_udp: alinan %lu yollanan %lu eski %lu uzunluk %lu udp_hatasi %lu yenileme_hatasi %lu cikis %d\n",
            n.received, n.sent, n.stale, n.bad_len, n.send_errors, n.renew_errors, result);

    return result;
}
