/* Zephyr uygulaması ile main_remote.c (ST/Zephyr OpenAMP örneğinden kopya) arasındaki bağlantı. */
#pragma once
#include <stddef.h>

#include <openamp/open_amp.h>

void app_set_tty_endpoint(struct rpmsg_endpoint *ept);  // "rpmsg-tty" uç noktası oluşunca çağrılır
void app_on_rx(const void *data, size_t len);           // TTY thread'i: A35'ten gelen baytlar
