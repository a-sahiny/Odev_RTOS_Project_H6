/* App/Src/attitude.c'nin PC birim testi (gcc). Kart gerekmez; kos.ps1 derler ve çalıştırır. */
#include "attitude.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG         (3.14159265358979 / 180.0)
#define POINT_HZ    1000
#define OUT_HZ      200
#define N           (POINT_HZ / OUT_HZ)

static int failed;
static AttOutput out;
static double now;                          // s
static double sim_bias[3];                  // °/s, sensörün sahte bias'ı
static double sim_gain = 1.0;               // ivmeölçer ölçeği
static double q[4];                         // gerçek duruş (sensör -> dünya)

/* Bir koşulu yazar; kalan sayılır. */
static void check(int ok, const char *text, double value)
{
    printf("  %s  %-62s (%.4f)\n", ok ? "GECTI" : "KALDI", text, value);
    failed += ok ? 0 : 1;
}

/* Normal dağılımlı gürültü (Box-Muller). */
static double noise(double sigma)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0);
    double v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sigma * sqrt(-2.0 * log(u)) * cos(6.28318530718 * v);
}

/* Gerçek duruşu kurar: roll, pitch (derece). */
static void pose(double roll, double pitch)
{
    double cr = cos(roll * DEG / 2);
    double sr = sin(roll * DEG / 2);
    double cp = cos(pitch * DEG / 2);
    double sp = sin(pitch * DEG / 2);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

/* Bir çıkış örneği üretir: gerçek hız w (°/s), ek özgül kuvvet a (g, sensör ekseni). Gürültü gerçek kartınki. */
static void step(const double w[3], const double a[3], double vib_g, double vib_dps)
{
    AttPoint pt[N];

    for (int j = 0; j < N; ++j)
    {
        double d[3];
        double n2 = 0.0;
        double p[4];

        now += 1.0 / POINT_HZ;
        for (int i = 0; i < 3; ++i)
        {
            d[i] = w[i] * DEG / POINT_HZ / 2;
            n2 += d[i] * d[i];
        }
        memcpy(p, q, sizeof(p));
        q[0] = p[0] * cos(sqrt(n2)) - d[0] * p[1] - d[1] * p[2] - d[2] * p[3];
        q[1] = p[1] + d[0] * p[0] + d[2] * p[2] - d[1] * p[3];
        q[2] = p[2] + d[1] * p[0] - d[2] * p[1] + d[0] * p[3];
        q[3] = p[3] + d[2] * p[0] + d[1] * p[1] - d[0] * p[2];
        n2 = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (int i = 0; i < 4; ++i)
        {
            q[i] /= n2;
        }

        double up[3] = { 2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]) };
        double s = sin(6.28318530718 * 147.0 * now);

        for (int i = 0; i < 3; ++i)
        {
            pt[j].acc[i] = (float)((up[i] + a[i]) * sim_gain + noise(0.004) + vib_g * s);
            pt[j].gyro[i] = (float)((w[i] + sim_bias[i] + noise(0.2) + vib_dps * s) * DEG);
        }
    }
    att_update(pt, N, 1.0f / POINT_HZ, &out);
}

/* sec saniye boyunca sabit hızla yürütür. */
static void run(double sec, double wx, double wy, double wz)
{
    const double w[3] = { wx, wy, wz };
    const double a[3] = { 0, 0, 0 };

    for (int k = 0; k < (int)(sec * OUT_HZ + 0.5); ++k)
    {
        step(w, a, 0, 0);
    }
}

static double angle(int i)
{
    return out.angle[i] / DEG;
}

static void boot(double roll, double pitch)
{
    sim_bias[0] = 1.2;
    sim_bias[1] = 2.7;
    sim_bias[2] = 2.3;
    sim_gain = 0.9905;
    pose(roll, pitch);
    att_init();
    att_set_rate(OUT_HZ, POINT_HZ);
    att_restart();
}

static void test_start(void)
{
    int ready = 0;

    printf("1. START: egim ivmeden, bias 1 s'de\n");
    boot(20, -30);
    for (int k = 1; (k <= 400) && (ready == 0); ++k)
    {
        run(1.0 / OUT_HZ, 0, 0, 0);
        if (out.flags & ATT_FLAG_BIAS_READY)
        {
            ready = k;
        }
    }
    check(ready == OUT_HZ, "BIAS_READY tam 1 s'lik ornekte", ready);
    run(2, 0, 0, 0);
    check(fabs(angle(0) - 20) < 0.2, "roll 20 derece", angle(0));
    check(fabs(angle(1) + 30) < 0.2, "pitch -30 derece", angle(1));
    check(fabs(out.gyro[2] / DEG) < 0.3, "jiro z ~ 0 (bias cikti)", out.gyro[2] / DEG);
    check((out.flags & ATT_FLAG_CAL_DONE) == 0, "CAL verilmedi: CAL_DONE yok", out.flags);
}

static void test_calibrate(void)
{
    printf("2. CAL: her sey sifir\n");
    boot(0.1, 16.3);
    run(2, 0, 0, 0);
    att_calibrate();
    run(1.0 / OUT_HZ, 0, 0, 0);
    check((out.flags & ATT_FLAG_CAL_BUSY) != 0, "CAL_BUSY basladi", out.flags);
    run(1.2, 0, 0, 0);
    check((out.flags & (ATT_FLAG_CAL_BUSY | ATT_FLAG_CAL_DONE)) == ATT_FLAG_CAL_DONE, "1,2 s sonra CAL_DONE", out.flags);
    run(2, 0, 0, 0);
    for (int i = 0; i < 3; ++i)
    {
        check(fabs(angle(i)) < 0.1, "aci 0", angle(i));
        check(fabs(out.acc[i] - (i == 2)) < 0.005, "ivme (0, 0, 1) g", out.acc[i]);
        check(fabs(out.gyro[i] / DEG) < 0.3, "jiro 0", out.gyro[i] / DEG);
    }
    printf("   ters (180 derece) duran kartta CAL\n");
    boot(180, 0);
    att_calibrate();
    run(3, 0, 0, 0);
    check(fabs(angle(0)) + fabs(angle(1)) < 0.2, "ters kartta da 0", fabs(angle(0)) + fabs(angle(1)));
}

static void test_calibrate_moving(void)
{
    printf("3. Hareketliyken CAL: sifirlamaz, 10 s sonra vazgecer\n");
    boot(0, 10);
    att_calibrate();
    run(2, 0, 0, 0);
    att_calibrate();
    for (int k = 0; k < 9 * OUT_HZ; ++k)
    {
        const double w[3] = { 20 * sin(8 * now), 12 * cos(6 * now), 0 };
        const double a[3] = { 0, 0, 0 };
        step(w, a, 0, 0);
    }
    check((out.flags & ATT_FLAG_CAL_BUSY) != 0, "9 s: hala bekliyor", out.flags);
    for (int k = 0; k < 2 * OUT_HZ; ++k)
    {
        const double w[3] = { 20 * sin(8 * now), 12 * cos(6 * now), 0 };
        const double a[3] = { 0, 0, 0 };
        step(w, a, 0, 0);
    }
    check((out.flags & (ATT_FLAG_CAL_BUSY | ATT_FLAG_CAL_DONE)) == 0, "11 s: vazgecti, CAL_DONE yok", out.flags);
    run(3, 0, 0, 0);
    check((out.flags & ATT_FLAG_BIAS_READY) != 0, "durunca bias yeniden hazir", out.flags);
}

static void test_rotation(void)
{
    printf("4. Donme: hicbir hareket kaybolmaz\n");
    boot(0, 0);
    att_calibrate();
    run(3, 0, 0, 0);
    run(1, 0, 0, 90);
    run(2, 0, 0, 0);
    check(fabs(angle(2) - 90) < 0.3, "90 derece/s x 1 s = 90", angle(2));
    run(10, 0, 0, 0.8);
    run(2, 0, 0, 0);
    check(fabs(angle(2) - 98) < 0.4, "0,8 derece/s x 10 s = 8 (yavas donus)", angle(2));
    run(2, 0, 180, 0);
    run(1, 0, 0, 0);
    check(fabs(angle(1)) < 0.5, "pitch 360 derece takla (90'dan gecer) -> 0", angle(1));
    check(isfinite(out.angle[0]) && isfinite(out.angle[2]), "NaN yok", out.angle[2]);
    run(0.5, 180, 0, 0);
    check(fabs(angle(0) - 90) < 0.5, "roll 90", angle(0));
}

static void test_still(void)
{
    double first;

    printf("5. Dururken: kayma yok, bias degisirse ogrenir\n");
    boot(0, 0);
    att_calibrate();
    run(3, 0, 0, 0);
    first = angle(2);
    run(120, 0, 0, 0);
    check(fabs(angle(2) - first) < 0.05, "120 s'de yaw kaymasi < 0,05 derece", angle(2) - first);
    check((out.flags & ATT_FLAG_STILL) != 0, "STILL", out.flags);
    sim_bias[2] += 0.15;
    run(10, 0, 0, 0);
    first = angle(2);
    run(20, 0, 0, 0);
    check(fabs(angle(2) - first) < 0.1, "bias 0,15 derece/s kaydi: 20 s'de yaw < 0,1", angle(2) - first);
}

static void test_vibration(void)
{
    const double w[3] = { 0, 0, 0 };
    const double a[3] = { 0, 0, 0 };
    double sum = 0;
    int still = 0;

    printf("6. Motor titresimi 147 Hz, 3 g, 40 derece/s\n");
    boot(0, 0);
    att_calibrate();
    run(3, 0, 0, 0);
    for (int k = 0; k < 20 * OUT_HZ; ++k)
    {
        step(w, a, 3.0, 40.0);
        sum += pow(out.gyro[0] / DEG, 2);
        still += ((k > 200) && (out.flags & ATT_FLAG_STILL)) ? 1 : 0;
    }
    sum = sqrt(sum / (20 * OUT_HZ));
    check(sum < 40.0 / sqrt(2) / 5, "cikis jirosunda titresim < giristeki / 5 (derece/s)", sum);
    check(still == 0, "titresirken STILL denmez", still);
    check(fabs(angle(0)) < 1.0 && fabs(angle(1)) < 1.0, "20 s sonra roll, pitch < 1 derece", angle(0));
    check(fabs(angle(2)) < 1.0, "20 s sonra yaw < 1 derece", angle(2));
}

static void test_free_fall(void)
{
    const double w[3] = { 30, 0, 0 };

    printf("7. Serbest dusme: ivme kullanilmaz\n");
    boot(0, 0);
    att_calibrate();
    run(3, 0, 0, 0);
    for (int k = 0; k < OUT_HZ; ++k)
    {
        double a[3] = { -2 * (q[1] * q[3] - q[0] * q[2]), -2 * (q[2] * q[3] + q[0] * q[1]), -(1 - 2 * (q[1] * q[1] + q[2] * q[2])) };
        step(w, a, 0, 0);
    }
    check((out.flags & ATT_FLAG_ACC_USED) == 0, "ACC_USED 0", out.flags);
    check(fabs(angle(0) - 30) < 0.5, "roll yalniz jirodan 30", angle(0));
}

static void test_rates(void)
{
    static const int rate[][2] = { { 10, 250 }, { 20, 500 }, { 50, 1000 }, { 100, 1000 }, { 200, 1000 } };

    printf("8. Butun SET_RATE hizlari: BIAS_READY 1 s'de\n");
    for (size_t r = 0; r < 5U; ++r)
    {
        AttPoint pt[25];
        size_t n = (size_t)(rate[r][1] / rate[r][0]);
        int ready = 0;

        att_init();
        att_set_rate((float)rate[r][0], (float)rate[r][1]);
        att_restart();
        for (int k = 1; (k <= 3 * rate[r][0]) && (ready == 0); ++k)
        {
            for (size_t j = 0; j < n; ++j)
            {
                for (int i = 0; i < 3; ++i)
                {
                    pt[j].acc[i] = (float)((i == 2) + noise(0.002));
                    pt[j].gyro[i] = (float)((1.0 + noise(0.1)) * DEG);
                }
            }
            att_update(pt, n, 1.0f / (float)rate[r][1], &out);
            ready = (out.flags & ATT_FLAG_BIAS_READY) ? k : 0;
        }
        check(ready == rate[r][0], "hiz kadar ornek sonra hazir", ready);
    }
}

int main(void)
{
    srand(1);
    test_start();
    test_calibrate();
    test_calibrate_moving();
    test_rotation();
    test_still();
    test_vibration();
    test_free_fall();
    test_rates();
    printf("\n%s: %d kosul kaldi\n", failed ? "KALDI" : "GECTI", failed);
    return failed ? 1 : 0;
}
