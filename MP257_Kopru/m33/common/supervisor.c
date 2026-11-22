/* M33 gözetim çekirdeği: mesajlardan ve zamandan modu çıkarır, LED desenini ve SUP satırını üretir.
   Yalnız gözetim task'ı çağırır (FreeRTOS: sup_task, Zephyr: sup_thread); kilit gerekmez. */
#include "supervisor.h"

#include <stdio.h>
#include <string.h>

#define F7_STATE_FAULT 0x04U                // F746 STATE baytı: bit2 FAULT (app_config.h APP_STATE_BIT_FAULT)

typedef struct
{
    uint32_t on_us;                         // periyodun başında yanık kalma süresi
    uint32_t period_us;
} LedPattern;

/* Mod başına LED3 (turuncu) deseni; tek LED olduğu için desenler süreyle ayrılır. */
static const LedPattern led_patterns[] =
{
    [SUP_WAIT_A35] = {  100000U, 1000000U },  // 1 Hz kısa yanıp sönme
    [SUP_NO_DATA]  = {  500000U, 1000000U },  // 1 Hz yarı yarıya
    [SUP_RUN]      = { 1000000U, 1000000U },  // sürekli yanık
    [SUP_ALARM]    = {   50000U,  100000U },  // 10 Hz hızlı
    [SUP_F7_FAULT] = {  200000U,  400000U },  // 2,5 Hz
    [SUP_A35_LOST] = {  100000U, 2000000U },  // 2 s'de bir kısa
};

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

void sup_init(Supervisor *s, uint32_t now_us)
{
    memset(s, 0, sizeof(*s));
    s->mode = SUP_WAIT_A35;
    s->lim_cdeg = SUP_LIM_DEFAULT_CDEG;
    s->report_us = now_us;
    s->dt_min_us = UINT32_MAX;
}

void sup_on_msg(Supervisor *s, const LinkMsg *msg, uint32_t now_us)
{
    switch (msg->type)
    {
        case LINK_MSG_HB:
        {
            s->hb_seen = 1U;
            s->hb_us = now_us;
            break;
        }
        case LINK_MSG_TM:
        {
            uint16_t seq = (uint16_t)msg->v[0];
            if (s->tm_seen != 0U)
            {
                uint32_t dt = now_us - s->tm_us;     // uint32 farkı taşmada da doğru
                if (dt < s->dt_min_us)
                {
                    s->dt_min_us = dt;
                }
                if (dt > s->dt_max_us)
                {
                    s->dt_max_us = dt;
                }

                uint16_t step = (uint16_t)(seq - s->tm_seq);
                if ((step > 1U) && (step < 0x8000U))
                {
                    s->tm_gaps += (uint32_t)(step - 1U);  // ileri sıçrama = kayıp; geri sıçrama F746 yeniden başladı
                }
            }
            s->tm_seen = 1U;
            s->tm_us = now_us;
            s->tm_seq = seq;
            ++s->tm_count;
            break;
        }
        case LINK_MSG_AT:
        {
            s->at_seen = 1U;
            s->at_us = now_us;
            s->roll_cdeg = msg->v[0];
            s->pitch_cdeg = msg->v[1];
            break;
        }
        case LINK_MSG_ST:
        {
            s->f7_state = (uint8_t)msg->v[0];
            s->f7_why = (uint8_t)msg->v[1];
            break;
        }
        case LINK_MSG_LIM:
        {
            s->lim_cdeg = msg->v[0];
            break;
        }
        default:
        {
            ++s->bad_lines;                          // link_parse başka tip üretmez; olursa say
            break;
        }
    }
}

void sup_on_bad_line(Supervisor *s)
{
    ++s->bad_lines;
}

SupMode sup_step(Supervisor *s, uint32_t now_us)
{
    /* Öncelik sırası: A35 yok > F746 FAULT > veri yok > alarm > çalışıyor. */
    if (s->hb_seen == 0U)
    {
        s->mode = SUP_WAIT_A35;
    }
    else if ((now_us - s->hb_us) > SUP_HB_TIMEOUT_US)
    {
        s->mode = SUP_A35_LOST;
    }
    else if ((s->f7_state & F7_STATE_FAULT) != 0U)
    {
        s->mode = SUP_F7_FAULT;
    }
    else if ((s->tm_seen == 0U) || ((now_us - s->tm_us) > SUP_TM_TIMEOUT_US))
    {
        s->mode = SUP_NO_DATA;
    }
    else if ((s->at_seen != 0U) && ((now_us - s->at_us) <= SUP_AT_TIMEOUT_US) &&
             ((abs32(s->roll_cdeg) > s->lim_cdeg) || (abs32(s->pitch_cdeg) > s->lim_cdeg)))
    {
        s->mode = SUP_ALARM;
    }
    else
    {
        s->mode = SUP_RUN;
    }
    return s->mode;
}

int sup_button_sample(Supervisor *s, int pressed)
{
    if (pressed == 0)
    {
        s->btn_count = 0U;
        s->btn_latched = 0U;                         // bırakıldı: sonraki basış yeniden sayılır
        return 0;
    }

    if (s->btn_count < SUP_BTN_STABLE)
    {
        ++s->btn_count;
    }
    if ((s->btn_count >= SUP_BTN_STABLE) && (s->btn_latched == 0U))
    {
        s->btn_latched = 1U;                         // basılı tutmak tek basış sayılır
        return 1;
    }
    return 0;
}

int sup_led_on(SupMode mode, uint32_t now_us)
{
    if ((uint32_t)mode >= (sizeof(led_patterns) / sizeof(led_patterns[0])))
    {
        return 1;                                    // bilinmeyen mod: LED yanık kalsın, göze çarpsın
    }
    const LedPattern *p = &led_patterns[mode];
    return (now_us % p->period_us) < p->on_us;
}

int sup_report_due(Supervisor *s, uint32_t now_us)
{
    return (now_us - s->report_us) >= SUP_REPORT_US;
}

size_t sup_format_report(Supervisor *s, uint32_t now_us, char *out, size_t cap)
{
    uint32_t tm_age_ms = (s->tm_seen != 0U) ? ((now_us - s->tm_us) / 1000U) : UINT32_MAX;
    uint32_t hb_age_ms = (s->hb_seen != 0U) ? ((now_us - s->hb_us) / 1000U) : UINT32_MAX;
    uint32_t dt_min = (s->dt_min_us == UINT32_MAX) ? 0U : s->dt_min_us;

    /* SUP <mod> <tm_age_ms> <tm_count> <tm_gaps> <dt_min_us> <dt_max_us> <hb_age_ms> <bad> */
    int n = snprintf(out, cap, "SUP %u %lu %lu %lu %lu %lu %lu %lu\n",
                     (unsigned)s->mode,
                     (unsigned long)tm_age_ms,
                     (unsigned long)s->tm_count,
                     (unsigned long)s->tm_gaps,
                     (unsigned long)dt_min,
                     (unsigned long)s->dt_max_us,
                     (unsigned long)hb_age_ms,
                     (unsigned long)s->bad_lines);

    /* Pencere değerleri her raporda sıfırlanır; sayaçlar birikir. */
    s->report_us = now_us;
    s->dt_min_us = UINT32_MAX;
    s->dt_max_us = 0U;

    if ((n < 0) || ((size_t)n >= cap))
    {
        return 0U;                                   // kesilmiş satır gönderilmez
    }
    return (size_t)n;
}
