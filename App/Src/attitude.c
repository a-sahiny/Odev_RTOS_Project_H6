/* Yönelim süzgeci: quaternion + 6 durumlu Kalman (3 açı hatası, 3 jiro bias'ı).
   Her nokta jiroyla ilerletir, her çıkışta ivme düzeltir, kart durunca bias öğrenilir. */
#include "attitude.h"

#include <math.h>
#include <string.h>

/* Debug derlemesi -O0: yalnız bu hesap dosyası hızlandırılır. */
#if defined(__arm__)
#pragma GCC optimize ("O2")
#endif

#define DEG                 (3.14159265f / 180.0f)

/* Kalman gürültüleri (PC karşılaştırmasıyla seçildi, IMU/ belgesi) */
#define GYRO_NOISE          (0.05f * DEG)   // rad/s/√Hz: gürültü + ölçek hatası + titreşim payı
#define BIAS_WALK           (0.01f * DEG)   // rad/s/√s: bias'ın motor ısısıyla yürümesi
#define ACC_NOISE_STILL     0.02f           // rad: kart dururken ivmenin gösterdiği açı
#define ACC_NOISE_MOVE      0.50f           // rad: uçuşta; gövde ivmesi açıyı saptırır
#define ACC_DEV_GAIN        10.0f           // |a| 1 g'den saptıkça güven düşer (rad / g)
#define ACC_RATE_GAIN       0.30f           // hızlı dönüşte güven düşer (rad / (rad/s))
#define ACC_DEV_MAX         0.5f            // g: daha çok saparsa ivme hiç kullanılmaz
#define ZARU_NOISE          (0.02f * DEG)   // rad/s: dururken ölçülen ortalama hızın hatası

/* Hareketsizlik ve kalibrasyon */
#define STILL_WINDOW_S      0.5f
#define COLLECT_WINDOW_S    1.0f
#define COLLECT_LIMIT_S     10.0f           // kart durmazsa kalibrasyondan vazgeçilir
#define STILL_GYRO_STD      (0.6f * DEG)    // rad/s; duran kartta ölçülen ~0,15-0,25 °/s
#define STILL_GYRO_MEAN     (0.2f * DEG)    // rad/s: bias belirsizliğinin (3 sigma) üstüne pay
#define STILL_LOCK_ANGLE    (0.1f * DEG)    // rad: kilitliyken biriken dönme bunu aşarsa hareket başladı
#define STILL_ACC_STD       0.020f          // g; duran kartta ölçülen 0,002-0,004 g
#define STILL_ACC_JUMP      0.10f           // g
#define GYRO_ZERO_MAX       (25.0f * DEG)   // rad/s: datasheet ±5 °/s + sıcaklık; üstü bias olamaz
#define ACC_ONE_G_TOL       0.10f           // g: kalibrasyonda |a| 1 g'den en çok bu kadar sapar

/* Çıkış alçak geçirenleri: kesim = çıkış hızının katı (Nyquist'in altında) */
#define GYRO_CUT_RATIO      0.30f           // 200 Hz'te 60 Hz: titreşim artığı / gecikme dengesi
#define ACC_CUT_RATIO       0.15f           // 200 Hz'te 30 Hz

/* Ortalama ve varyans penceresi: 0-2 ivme (g), 3-5 jiro (rad/s). */
typedef struct
{
    uint32_t n;
    float mean[6];
    float m2[6];
} Window;

/* 2. derece Butterworth, 6 kanal */
typedef struct
{
    float b0, a1, a2;
    float z1[3], z2[3];
} Lowpass;

typedef enum
{
    COLLECT_NONE,
    COLLECT_START,                          // START: yalnız jiro bias'ı ve yaw
    COLLECT_CAL                             // CAL: bias + montaj hizalaması + ölçek, bütün açılar 0
} Collect;

static struct
{
    float q[4];                             // gövde -> dünya
    float bias[3];                          // gyro_zero'dan sonra kalan bias, gövde ekseni (rad/s)
    float p[6][6];                          // hata kovaryansı: 0-2 açı, 3-5 bias
    float mount[3][3];                      // sensör -> gövde (CAL hizalaması)
    float gyro_zero[3];                     // ölçülmüş bias, sensör ekseni (rad/s)
    float acc_scale;                        // durağan |a| = 1 g
    Lowpass lp_acc;
    Lowpass lp_gyro;
    Window still_win;
    Window collect_win;
    float still_acc[3];                     // son hareketsiz pencerenin ivme ortalaması (g)
    float lock[3];                          // kilitliyken biriken dönme (rad): hareketse geri verilir
    uint32_t still_target;                  // çıkış örneği
    uint32_t collect_target;
    uint32_t collect_left;                  // vazgeçmeye kalan çıkış örneği
    Collect collect;
    uint8_t still;
    uint8_t aligned;                        // ilk ivmeden açı alındı
    uint8_t filled;                         // alçak geçirenler dolduruldu
    uint8_t bias_ready;                     // bu START / CAL'den sonra ölçüldü
    uint8_t bias_known;                     // açılıştan beri en az bir kez ölçüldü
    uint8_t cal_done;
} s;

/* Vektör çarpımı: sensör -> gövde. */
static void to_body(const float in[3], float out[3])
{
    for (size_t i = 0; i < 3U; ++i)
    {
        out[i] = s.mount[i][0] * in[0] + s.mount[i][1] * in[1] + s.mount[i][2] * in[2];
    }
}

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/* Kovaryansı baştan kurar: açı ve bias belirsizliği (rad, rad/s). */
static void covariance_reset(float angle, float bias)
{
    memset(s.p, 0, sizeof(s.p));
    for (size_t i = 0; i < 3U; ++i)
    {
        s.p[i][i] = angle * angle;
        s.p[3U + i][3U + i] = bias * bias;
    }
}

/* Butterworth katsayıları; durum değerleri korunur. */
static void lowpass_set(Lowpass *f, float cut_hz, float rate_hz)
{
    float k = tanf(3.14159265f * cut_hz / rate_hz);
    float d = 1.0f / (1.0f + 1.41421356f * k + k * k);

    f->b0 = k * k * d;
    f->a1 = 2.0f * (k * k - 1.0f) * d;
    f->a2 = (1.0f - 1.41421356f * k + k * k) * d;
}

/* Süzgeci bir değerde durağan hale getirir: ilk örnekte sıçrama olmaz. */
static void lowpass_fill(Lowpass *f, const float v[3])
{
    for (size_t i = 0; i < 3U; ++i)
    {
        f->z1[i] = v[i] * (1.0f - f->b0);
        f->z2[i] = v[i] * (f->b0 - f->a2);
    }
}

/* Bir adım (transposed direct form II); süzülmüş değer out'a yazılır. */
static void lowpass_step(Lowpass *f, const float v[3], float out[3])
{
    for (size_t i = 0; i < 3U; ++i)
    {
        float x = f->b0 * v[i];
        float y = x + f->z1[i];

        f->z1[i] = 2.0f * x - f->a1 * y + f->z2[i];
        f->z2[i] = x - f->a2 * y;
        out[i] = y;
    }
}

static void window_clear(Window *w)
{
    memset(w, 0, sizeof(*w));
}

/* Welford: ortalama ve kareler toplamı, taşma ve sayısal kayıp olmadan. */
static void window_add(Window *w, const float acc[3], const float gyro[3])
{
    ++w->n;
    for (size_t i = 0; i < 6U; ++i)
    {
        float x = (i < 3U) ? acc[i] : gyro[i - 3U];
        float d = x - w->mean[i];

        w->mean[i] += d / (float)w->n;
        w->m2[i] += d * (x - w->mean[i]);
    }
}

/* Pencere hareketsiz mi? 1: bütün kanalların sapması eşiğin altında ve |a| ~ 1 g. */
static int window_quiet(const Window *w)
{
    float var = 1.0f / (float)w->n;

    for (size_t i = 0; i < 6U; ++i)
    {
        float limit = (i < 3U) ? STILL_ACC_STD : STILL_GYRO_STD;

        if (w->m2[i] * var > limit * limit)
        {
            return 0;
        }
    }

    return fabsf(norm3(w->mean) * s.acc_scale - 1.0f) < ACC_ONE_G_TOL;
}

/* q <- q ⊗ küçük dönme (rad, gövde ekseni). Seri 0,035 rad'da 1e-9 doğru; sin / cos gerekmez. */
static void rotate(const float d[3])
{
    float sq = 0.25f * (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    float c = 1.0f - 0.5f * sq;
    float k = 0.5f * (1.0f - sq * (1.0f / 6.0f));
    float x = k * d[0];
    float y = k * d[1];
    float z = k * d[2];
    float q0 = s.q[0];
    float q1 = s.q[1];
    float q2 = s.q[2];
    float q3 = s.q[3];

    s.q[0] = c * q0 - x * q1 - y * q2 - z * q3;
    s.q[1] = c * q1 + x * q0 + z * q2 - y * q3;
    s.q[2] = c * q2 + y * q0 - z * q1 + x * q3;
    s.q[3] = c * q3 + z * q0 + y * q1 - x * q2;
}

static void normalise(void)
{
    float n = s.q[0] * s.q[0] + s.q[1] * s.q[1] + s.q[2] * s.q[2] + s.q[3] * s.q[3];
    float k = 1.0f / sqrtf(n);

    for (size_t i = 0; i < 4U; ++i)
    {
        s.q[i] *= k;
    }
}

/* Dünya "yukarı" yönü, gövde ekseninde: duran ivmeölçerin göstereceği değer (g). */
static void up_in_body(float up[3])
{
    up[0] = 2.0f * (s.q[1] * s.q[3] - s.q[0] * s.q[2]);
    up[1] = 2.0f * (s.q[2] * s.q[3] + s.q[0] * s.q[1]);
    up[2] = 1.0f - 2.0f * (s.q[1] * s.q[1] + s.q[2] * s.q[2]);
}

/* Açıyı ivmeden alır: roll ve pitch ölçülür, yaw 0. */
static void align_to(const float acc[3])
{
    float roll = 0.5f * atan2f(acc[1], acc[2]);
    float pitch = 0.5f * atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    s.q[0] = cr * cp;
    s.q[1] = sr * cp;
    s.q[2] = cr * sp;
    s.q[3] = -sr * sp;
}

/* Kovaryans ilerletme: F = [I - [w t]x, -t I; 0, I], P = F P F' + Q. */
static void covariance_predict(const float w[3], float t)
{
    float x = w[0] * t;
    float y = w[1] * t;
    float z = w[2] * t;

    for (size_t c = 0; c < 6U; ++c)
    {
        float p0 = s.p[0][c];
        float p1 = s.p[1][c];
        float p2 = s.p[2][c];

        s.p[0][c] = p0 + z * p1 - y * p2 - t * s.p[3][c];
        s.p[1][c] = p1 - z * p0 + x * p2 - t * s.p[4][c];
        s.p[2][c] = p2 + y * p0 - x * p1 - t * s.p[5][c];
    }
    for (size_t r = 0; r < 6U; ++r)
    {
        float p0 = s.p[r][0];
        float p1 = s.p[r][1];
        float p2 = s.p[r][2];

        s.p[r][0] = p0 + z * p1 - y * p2 - t * s.p[r][3];
        s.p[r][1] = p1 - z * p0 + x * p2 - t * s.p[r][4];
        s.p[r][2] = p2 + y * p0 - x * p1 - t * s.p[r][5];
    }
    for (size_t i = 0; i < 3U; ++i)
    {
        s.p[i][i] += GYRO_NOISE * GYRO_NOISE * t;
        s.p[3U + i][3U + i] += BIAS_WALK * BIAS_WALK * t;
    }
}

/* Üç ölçümü tek tek işler (matris tersi gerekmez). h: ölçüm satırlarının sıfır olmayan üç sütunu,
   first'ten başlar (0 açı, 3 bias). r: artık, var: varyans. */
static void correct(const float h[3][3], size_t first, const float r[3], float var)
{
    float dx[6] = { 0 };

    for (size_t m = 0; m < 3U; ++m)
    {
        float ph[6];
        float innov = r[m];
        float sum = var;

        for (size_t i = 0; i < 6U; ++i)
        {
            const float *p = &s.p[i][first];

            ph[i] = p[0] * h[m][0] + p[1] * h[m][1] + p[2] * h[m][2];
        }
        for (size_t i = 0; i < 3U; ++i)
        {
            sum += h[m][i] * ph[first + i];
            innov -= h[m][i] * dx[first + i];
        }

        float k = 1.0f / sum;

        for (size_t i = 0; i < 6U; ++i)
        {
            dx[i] += ph[i] * k * innov;
            for (size_t j = i; j < 6U; ++j)
            {
                s.p[i][j] -= ph[i] * ph[j] * k;
                s.p[j][i] = s.p[i][j];      // simetri korunur
            }
        }
    }

    rotate(dx);
    normalise();
    for (size_t i = 0; i < 3U; ++i)
    {
        s.bias[i] += dx[3U + i];
    }
}

/* İvme düzeltmesi: ölçülen yön ile beklenen "yukarı" yönü arasındaki fark. 1: kullanıldı. */
static int correct_accel(const float acc[3], const float rate[3])
{
    float g = norm3(acc);
    float dev = fabsf(g - 1.0f);

    if (dev > ACC_DEV_MAX)
    {
        return 0;                           // serbest düşme ya da darbe: yön bilgisi yok
    }

    float up[3];
    float r[3];
    float noise = ACC_NOISE_STILL;

    if (s.still == 0U)
    {
        noise = ACC_NOISE_MOVE + ACC_DEV_GAIN * dev + ACC_RATE_GAIN * norm3(rate);
    }

    up_in_body(up);
    for (size_t i = 0; i < 3U; ++i)
    {
        r[i] = acc[i] / g - up[i];
    }

    const float h[3][3] =
    {
        { 0.0f, -up[2], up[1] },            // [up]x
        { up[2], 0.0f, -up[0] },
        { -up[1], up[0], 0.0f },
    };

    correct(h, 0U, r, noise * noise);
    return 1;
}

/* Dururken ortalama hız bias'ın kendisidir (sıfır hız güncellemesi, ZARU). */
static void correct_still_rate(const float rate[3])
{
    static const float unit[3][3] = { { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };
    float r[3];

    for (size_t i = 0; i < 3U; ++i)
    {
        r[i] = rate[i] - s.bias[i];
    }
    correct(unit, 3U, r, ZARU_NOISE * ZARU_NOISE);
}

/* Ölçülen ortalama hız jironun sıfırı olur; Kalman bias'ı onun üstünde baştan başlar. */
static void set_gyro_zero(const float mean[3])
{
    memcpy(s.gyro_zero, mean, sizeof(s.gyro_zero));
    memset(s.bias, 0, sizeof(s.bias));
    s.bias_ready = 1U;
    s.bias_known = 1U;
}

/* Hareketsizliği bırakır, yeni pencere başlatır. */
static void still_clear(void)
{
    s.still = 0U;
    memset(s.lock, 0, sizeof(s.lock));
    window_clear(&s.still_win);
}

/* Hareket başladı: kilitliyken biriken dönme açıya geri verilir, hiçbir hareket kaybolmaz. */
static void still_release(void)
{
    rotate(s.lock);
    still_clear();
}

/* Hareketsizlik: 0,5 s'lik pencere karar verir; biriken dönme ya da ivme sıçraması hemen bitirir. */
static void still_update(const float acc[3], const float gyro[3])
{
    if (s.still != 0U)
    {
        for (size_t i = 0; i < 3U; ++i)
        {
            if ((fabsf(acc[i] - s.still_acc[i]) > STILL_ACC_JUMP) || (fabsf(s.lock[i]) > STILL_LOCK_ANGLE))
            {
                still_release();
                return;
            }
        }
    }

    window_add(&s.still_win, acc, gyro);
    if (s.still_win.n < s.still_target)
    {
        return;
    }

    float raw[3];
    float rate[3];
    int quiet = window_quiet(&s.still_win);

    for (size_t i = 0; i < 3U; ++i)
    {
        raw[i] = s.still_win.mean[3U + i] - s.gyro_zero[i];
    }
    to_body(raw, rate);
    for (size_t i = 0; i < 3U; ++i)
    {
        /* Sabit, çok yavaş bir dönüş bias sanılmasın. */
        float gate = STILL_GYRO_MEAN + 3.0f * sqrtf(s.p[3U + i][3U + i]);

        if ((s.bias_ready != 0U) && (fabsf(rate[i] - s.bias[i]) > gate))
        {
            quiet = 0;
        }
    }

    if (quiet == 0)
    {
        still_release();
        return;
    }

    memcpy(s.still_acc, s.still_win.mean, sizeof(s.still_acc));
    if (s.bias_ready != 0U)
    {
        correct_still_rate(rate);
    }
    else if (s.collect == COLLECT_NONE)
    {
        set_gyro_zero(&s.still_win.mean[3]);    // START'ta kart hareketliydi: ilk duruşta ölç
    }
    else
    {
        /* START / CAL ölçümü sürüyor, bias'ı o verecek. */
    }
    still_clear();
    s.still = 1U;                               // kilitte biriken yalnız gürültüydü
}

/* CAL başarılı: ölçülen yer çekimi +z olur, |a| = 1 g, bütün açılar 0. */
static void apply_calibration(const float acc[3])
{
    float g = norm3(acc);
    float x = acc[0] / g;
    float y = acc[1] / g;
    float z = acc[2] / g;

    s.acc_scale = 1.0f / g;
    if (z < -0.9999f)                       // kart ters: en kısa dönme belirsiz, x çevresinde 180°
    {
        const float flip[3][3] = { { 1.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } };
        memcpy(s.mount, flip, sizeof(s.mount));
    }
    else
    {
        float k = 1.0f / (1.0f + z);
        const float m[3][3] =
        {
            { 1.0f - k * x * x, -k * x * y, -x },
            { -k * x * y, 1.0f - k * y * y, -y },
            { x, y, z },
        };
        memcpy(s.mount, m, sizeof(s.mount));
    }

    s.q[0] = 1.0f;
    s.q[1] = 0.0f;
    s.q[2] = 0.0f;
    s.q[3] = 0.0f;
    s.cal_done = 1U;
}

/* START / CAL ölçümü: 1 s hareketsiz veri; hareket varsa baştan, 10 s sonra vazgeçer. */
static void collect_update(const float acc[3], const float gyro[3])
{
    if (s.collect == COLLECT_NONE)
    {
        return;
    }

    window_add(&s.collect_win, acc, gyro);
    --s.collect_left;

    if (s.collect_win.n >= s.collect_target)
    {
        int ok = window_quiet(&s.collect_win);

        for (size_t i = 0; i < 3U; ++i)
        {
            if (fabsf(s.collect_win.mean[3U + i]) > GYRO_ZERO_MAX)
            {
                ok = 0;
            }
        }

        if (ok != 0)
        {
            set_gyro_zero(&s.collect_win.mean[3]);
            if (s.collect == COLLECT_CAL)
            {
                apply_calibration(s.collect_win.mean);
            }
            else
            {
                float body[3];

                to_body(s.collect_win.mean, body);
                align_to(body);
            }
            covariance_reset(0.3f * DEG, 0.02f * DEG);
            s.filled = 0U;
            s.collect = COLLECT_NONE;
            memcpy(s.still_acc, s.collect_win.mean, sizeof(s.still_acc));
            still_clear();
            s.still = 1U;                   // son 1 s hareketsizdi: açı hemen kilitli
            return;
        }
        window_clear(&s.collect_win);
    }

    if (s.collect_left == 0U)
    {
        s.collect = COLLECT_NONE;           // kart durmadı: eski hizalama ve bias geçerli kalır
    }
}

/* START / CAL ölçümünü başlatır; vazgeçme süresi çıkış örneği olarak sayılır. */
static void collect_begin(Collect what)
{
    s.collect = what;
    s.collect_left = (uint32_t)(COLLECT_LIMIT_S / COLLECT_WINDOW_S) * s.collect_target;
    window_clear(&s.collect_win);
}

/* Açılış: hizalama yok (birim matris), ölçek 1, bias bilinmiyor (geniş belirsizlik). */
void att_init(void)
{
    memset(&s, 0, sizeof(s));
    s.q[0] = 1.0f;
    s.acc_scale = 1.0f;
    for (size_t i = 0; i < 3U; ++i)
    {
        s.mount[i][i] = 1.0f;
    }
    covariance_reset(30.0f * DEG, 3.0f * DEG);
}

/* SET_RATE: süzgeç katsayıları ve pencere boyları yeni hıza göre. */
void att_set_rate(float out_hz, float point_hz)
{
    lowpass_set(&s.lp_gyro, GYRO_CUT_RATIO * out_hz, point_hz);
    lowpass_set(&s.lp_acc, ACC_CUT_RATIO * out_hz, point_hz);
    s.filled = 0U;
    s.still_target = (uint32_t)(STILL_WINDOW_S * out_hz);
    s.collect_target = (uint32_t)(COLLECT_WINDOW_S * out_hz);
    still_clear();
    if (s.collect != COLLECT_NONE)
    {
        collect_begin(s.collect);           // pencere boyu değişti: ölçüm baştan
    }
}

/* START: açı ivmeden yeniden alınır, yaw 0, bias 1 s yeniden ölçülür. */
void att_restart(void)
{
    s.aligned = 0U;
    s.bias_ready = 0U;
    s.filled = 0U;
    still_clear();
    if (s.collect != COLLECT_CAL)           // bekleyen CAL daha kapsamlı: onu bozma
    {
        collect_begin(COLLECT_START);
    }
}

/* CAL: kart durunca sıfırlama; o zamana kadar eski çözüm sürer. */
void att_calibrate(void)
{
    s.cal_done = 0U;
    s.bias_ready = 0U;
    collect_begin(COLLECT_CAL);
}

/* Bir blok -> bir çıkış: ilerlet, süz, düzelt, kalibrasyonu topla, açıları yaz. */
void att_update(const AttPoint *pt, size_t n, float dt, AttOutput *out)
{
    float acc_sum[3] = { 0 };               // sensör ekseninde blok ortalaması
    float gyro_sum[3] = { 0 };
    float body_acc[3] = { 0 };
    float rate[3] = { 0 };
    float k = 1.0f / (float)n;

    for (size_t j = 0; j < n; ++j)
    {
        float a[3];
        float w[3];
        float d[3];

        for (size_t i = 0; i < 3U; ++i)
        {
            acc_sum[i] += pt[j].acc[i];
            gyro_sum[i] += pt[j].gyro[i];
            a[i] = pt[j].acc[i] * s.acc_scale;
            w[i] = pt[j].gyro[i] - s.gyro_zero[i];
        }

        float ab[3];
        float wb[3];

        to_body(a, ab);
        to_body(w, wb);
        for (size_t i = 0; i < 3U; ++i)
        {
            wb[i] -= s.bias[i];
            d[i] = wb[i] * dt;
            body_acc[i] += ab[i] * k;
            rate[i] += wb[i] * k;
        }

        if (s.filled == 0U)
        {
            lowpass_fill(&s.lp_acc, ab);
            lowpass_fill(&s.lp_gyro, wb);
            s.filled = 1U;
        }
        lowpass_step(&s.lp_acc, ab, out->acc);
        lowpass_step(&s.lp_gyro, wb, out->gyro);

        if (s.still == 0U)
        {
            rotate(d);
        }
        else                                // dururken açı kilitli: gürültü integre edilmez
        {
            for (size_t i = 0; i < 3U; ++i)
            {
                s.lock[i] += d[i];
            }
        }
    }
    normalise();

    for (size_t i = 0; i < 3U; ++i)
    {
        acc_sum[i] *= k;
        gyro_sum[i] *= k;
    }

    if (s.aligned == 0U)
    {
        align_to(body_acc);
        covariance_reset(10.0f * DEG, (s.bias_known != 0U) ? 0.1f * DEG : 3.0f * DEG);
        s.aligned = 1U;
    }

    covariance_predict(rate, dt * (float)n);
    still_update(acc_sum, gyro_sum);

    /* Süzülmüş ivme: titreşim, |a| ~ 1 g görünüp yanlış yön vermesin. */
    out->flags = (correct_accel(out->acc, rate) != 0) ? ATT_FLAG_ACC_USED : 0U;
    collect_update(acc_sum, gyro_sum);

    out->angle[0] = atan2f(2.0f * (s.q[0] * s.q[1] + s.q[2] * s.q[3]), 1.0f - 2.0f * (s.q[1] * s.q[1] + s.q[2] * s.q[2]));
    out->angle[1] = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (s.q[0] * s.q[2] - s.q[3] * s.q[1]))));
    out->angle[2] = atan2f(2.0f * (s.q[0] * s.q[3] + s.q[1] * s.q[2]), 1.0f - 2.0f * (s.q[2] * s.q[2] + s.q[3] * s.q[3]));

    out->flags |= (s.bias_ready != 0U) ? ATT_FLAG_BIAS_READY : 0U;
    out->flags |= (s.collect == COLLECT_CAL) ? ATT_FLAG_CAL_BUSY : 0U;
    out->flags |= (s.cal_done != 0U) ? ATT_FLAG_CAL_DONE : 0U;
    out->flags |= (s.still != 0U) ? ATT_FLAG_STILL : 0U;
}
