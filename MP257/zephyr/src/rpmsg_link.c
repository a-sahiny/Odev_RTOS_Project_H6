/* RPMsg kanalı: link thread'i Linux hazır olunca rpmsg-raw uç noktasını açar ve komutları alır; main thread'i paketi link_send ile verir. */
#include "rpmsg_link.h"
#include "ipcc.h"
#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <resource_table.h>

#define SHM_ADDR DT_REG_ADDR(DT_CHOSEN(zephyr_ipc_shm))
#define SHM_SIZE DT_REG_SIZE(DT_CHOSEN(zephyr_ipc_shm))

K_SEM_DEFINE(ipcc_sem, 0, 1);                        // 1 yeter: bir uyanışta bütün mesajlar işlenir
K_TIMER_DEFINE(lease_timer, lease_isr, NULL);        // her '1' baştan başlatır

volatile uint32_t link_state = LINK_WAIT;
volatile int32_t link_error;
volatile uint32_t link_rx;
volatile uint32_t link_unknown;
volatile uint32_t link_on;
volatile uint32_t link_expired;
volatile uint32_t link_sent;
volatile uint32_t link_skipped;
volatile uint32_t link_drops;

static metal_phys_addr_t shm_phys = SHM_ADDR;
static metal_phys_addr_t table_phys;
static struct metal_io_region shm_io;                // vring'ler ve tamponlar
static struct metal_io_region table_io;              // resource table
static struct rpmsg_virtio_device rvdev;
static struct rpmsg_endpoint endpoint;

/* Kütüphane Linux'a haber vermek isteyince çağrılır. */
static int notify_linux(void *priv, uint32_t id)
{
    (void)priv;
    ipcc_notify(id);

    return 0;
}

/* Linux'un tek baytlık komutunu uygular: '1' LINK_LEASE_MS boyunca gönder, '0' dur. */
static int on_message(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
    (void)ept;
    (void)src;
    (void)priv;
    ++link_rx;

    uint8_t command = len == 1 ? *(const uint8_t *)data : 0;

    if (command == '1')
    {
        // Okuyan program çökerse '1' kesilir, süre dolunca gönderme durur
        k_timer_start(&lease_timer, K_MSEC(LINK_LEASE_MS), K_NO_WAIT);
        link_on = 1;                                 // Linux'un adresi bu mesajla öğrenildi
    }
    else if (command == '0')
    {
        k_timer_stop(&lease_timer);
        link_on = 0;                                 // okuyan yok: paketler Linux çekirdeğinde birikmesin
    }
    else
    {
        ++link_unknown;
    }

    return RPMSG_SUCCESS;
}

/* Paketi tampona kopyalar ve Linux'a haber verir; başarı 1. */
int link_send(const void *data, size_t size)
{
    if (!link_on)
    {
        ++link_skipped;
        return 0;
    }

    // trysend tampon yoksa beklemeden hata döner
    if (rpmsg_trysend(&endpoint, data, size) < 0)
    {
        ++link_drops;
        return 0;
    }

    ++link_sent;

    return 1;
}

/* Linux'un kanalı açmasını kesmeyle bekler; başarı 0. */
static int wait_linux(struct virtio_device *vdev)
{
    for (;;)
    {
        uint8_t status;
        int ret = virtio_get_status(vdev, &status);

        // Linux'un haberi kesme açılmadan gelmiş olabilir: uyumadan önce durum okunur
        if (ret != 0 || (status & VIRTIO_CONFIG_STATUS_DRIVER_OK))
        {
            return ret;
        }

        k_sem_take(&ipcc_sem, K_FOREVER);
    }
}

/* Vring'leri bağlar ve uç noktayı açar; başarı 0. */
static int open_channel(struct virtio_device *vdev, void *table)
{
    struct fw_rsc_vdev_vring *tx = rsc_table_get_vring0(table); // M33'ten Linux'a
    struct fw_rsc_vdev_vring *rx = rsc_table_get_vring1(table); // Linux'tan M33'e

    int ret = rproc_virtio_init_vring(vdev, 0, tx->notifyid, (void *)tx->da, &table_io, tx->num, tx->align);
    if (ret != 0)
    {
        return ret;
    }

    ret = rproc_virtio_init_vring(vdev, 1, rx->notifyid, (void *)rx->da, &table_io, rx->num, rx->align);
    if (ret != 0)
    {
        return ret;
    }

    ret = rpmsg_init_vdev(&rvdev, vdev, NULL, &shm_io, NULL);
    if (ret != 0)
    {
        return ret;
    }

    // Linux bu adı görünce rpmsg_char sürücüsünü bağlar
    return rpmsg_create_ept(&endpoint, rpmsg_virtio_get_rpmsg_device(&rvdev), "rpmsg-raw",
                            RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, on_message, NULL);
}

/* Kanalı durdurur; telemetri ayrı thread'de çalışmaya devam eder. */
static void link_stop(int error)
{
    link_error = error;
    link_state = LINK_FAILED;
}

/* Link thread'i: kanalı açar, sonra Linux'un her haberinde gelen mesajları on_message'a verir. */
static void link_task(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    struct metal_init_params params = METAL_INIT_DEFAULTS;
    void *table;
    int table_size;

    int ret = metal_init(&params);
    if (ret != 0)
    {
        link_stop(ret);
        return;
    }

    rsc_table_get(&table, &table_size);
    table_phys = (metal_phys_addr_t)table;
    metal_io_init(&shm_io, (void *)SHM_ADDR, &shm_phys, SHM_SIZE, -1, 0, NULL);
    metal_io_init(&table_io, table, &table_phys, table_size, -1, 0, NULL);

    struct virtio_device *vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID, rsc_table_to_vdev(table),
                                                          &table_io, NULL, notify_linux, NULL);
    if (vdev == NULL)
    {
        link_stop(-ENOMEM);                          // CONFIG_HEAP_MEM_POOL_SIZE yetmedi
        return;
    }

    if (!ipcc_open())
    {
        link_stop(-ENODEV);
        return;
    }

    ret = wait_linux(vdev);
    if (ret == 0)
    {
        ret = open_channel(vdev, table);
    }

    if (ret != 0)
    {
        link_stop(ret);
        return;
    }

    link_state = LINK_OPEN;

    for (;;)
    {
        k_sem_take(&ipcc_sem, K_FOREVER);
        rproc_virtio_notified(vdev, VRING1_ID);
    }
}

// Öncelik 5: telemetri thread'i (main, 0) her zaman önce; stack 1536 bayt
K_THREAD_DEFINE(link_id, 1536, link_task, NULL, NULL, NULL, 5, 0, 0);
