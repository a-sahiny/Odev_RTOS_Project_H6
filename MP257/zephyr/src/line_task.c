/* Line thread'i: USART6 kesmesinin verdiği satır parçalarını birleştirir, tam satırı Linux'a verir. */
#include "app.h"
#include "rpmsg_link.h"

// Parça en az 0,43 ms'de bir gelir: 32 parça 14 ms pay
K_MSGQ_DEFINE(piece_queue, sizeof(Piece), 32, 4);
volatile uint32_t piece_drops;
volatile uint32_t line_count;

static LineRx rx;

/* piece_queue'dan okur, satır bitince link_send_line ile verir; eksik parçalı satır rx.lost'ta sayılır. */
static void line_task(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    for (;;)
    {
        Piece piece;

        k_msgq_get(&piece_queue, &piece, K_FOREVER); // thread parça gelene kadar uyur
        size_t len = line_add(&rx, piece.data, piece.size);

        if (len > 0)
        {
            ++line_count;
            (void)link_send_line(rx.buf, len);       // kayıp link_drops ve link_skipped'te
        }
    }
}

// Öncelik 6: telemetri (main, 0) ve link (5) thread'lerinden sonra; stack 1024 bayt
K_THREAD_DEFINE(line_id, 1024, line_task, NULL, NULL, NULL, 6, 0, 0);
