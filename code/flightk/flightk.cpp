#include <stdio.h>
#include <math.h>
#include <stdarg.h>

#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/pio.h"

#include "uprs.h"
#include "littlefs.hpp"
#include "imu.h"
#include "rf.h"
#include "umotor.h"
#include "nmeap.h"
#include "../lib/ikafly_pin.h"
//#include "imgcone.h"
#include "ulog.h"
#include "ugps.h"
#include "tjpg.hpp"
#include "cam.h"

#include "config.h"

extern "C" {
#include "util.h"
}
#define CONST_180_DIVIDED_BY_PI 57.2957795130823
#define RAD2DEG 57.2958

repeating_timer_t timer;

#define XCLK 0
#define Y2_PIO_BASE 1
#define PIO pio0
#define PIO_SM 0
#define DMA_CH 0

Cam cam(i2c1, XCLK, Y2_PIO_BASE, PIO, PIO_SM, DMA_CH);
TJPGD tjpg;

Motor motor;
Press prs(i2c1, 0x77);
IMU imu(i2c1);
//IMGCONE imgcone;
Log logging(0);
static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect);
void count_vert(uint8_t vert[5], uint32_t area[15]);
void aupdate();
// NSE19(8/16) lat: 40.142646, long: 139.987610
// NSE19(8/17) lat: 40.142647, long: 139.987595
// 3448.09403,N,13546.25752,
GPS gps(
    34801609, // goal_lat
    135770979, // goal_long
    111, 92, // noshiro
    [](double x) {return (x);}
);

//#ifdef CODE_A
//Log //logging('A', 0);
//#endif //CODE_A
//
//#ifdef CODE_B
//Log //logging('B', 1);
//#endif //CODE_B
//
//#ifdef CODE_C
//Log //logging('C', 2);
//#endif //CODE_C

uint8_t logbuf[17];
bool rt_flag = false;

//bool bnoIsEnoughAccuracy() {
//    double accq = //bno08x.getQuatRadianAccuracy();
//    double accd = accq * CONST_180_DIVIDED_BY_PI;
//    printf("acc: rad: %f, deg: %f\n", accq, accd);
//    if (accd < 15.0f) return true;
//    else return false;
//}
#define H_MIN_1 0 // 固定
#define H_MAX_1 60
#define H_MIN_2 300
#define H_MAX_2 360 // 固定
// Satuation
#define S_MIN 10
#define S_MAX 100
// Value
#define V_MIN 15
#define V_MAX 90

// util funcs
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

// Result: 20x15
uint32_t red_area[15] = {0};
uint8_t vert[5] = {0};

void addLogBuf(const char* fmt, ...) {
    va_list arg;
    va_start(arg, fmt);
    vsnprintf((char *)logbuf, 17, fmt, arg);
    va_end(arg);
}

class Mode {
public:
    float alt_ref;
    Mode() {
        // landing
        for (int i = 0; i < 10; i++) alt_change[i] = 3776.0f;
        isDetectRise = false; isDetectFall = false;
        landingCnt = 0;
        // expansion, forwardLanding
        expansionCnt = 0;
        isOnlyGnss = false;
    }
    int8_t landing() {
        for (int8_t i = 1; i < 10; i++) {
            alt_change[i-1] = alt_change[i];
        }
        alt_change[9] = prs.getAltM();
        printf("alt: %f\n", alt_change[9]);
        printf("rise: %d, fall: %d\n", isDetectRise, isDetectFall);
        for (int i = 0; i < 10; i++) printf("%3.2f ", alt_change[i]);
        printf("\n");

        if (!isDetectRise) {
            if (alt_change[0] >= 3000) {
                addLogBuf("%3.1f altNRdy", alt_change[9]);
                return MODE_LANDING;
            }
            int8_t cnt = 0;
            for (int8_t i = 1; i < 10; i++) {
                // さっきより高度が高い=より上にいる
                if ((alt_change[i] - alt_change[i-1]) > 0.04) cnt++;
                //printf("isDetectRise: %d\n", cnt);
            }
            if (cnt > 4) {
                addLogBuf("%3.1f c%d Rise", alt_change[9], cnt);
                isDetectRise = true;
            }
        }
        if (isDetectRise && !isDetectFall) {
            if (alt_change[0] >= 1000) return MODE_LANDING;
            int8_t cnt = 0;
            for (int8_t i = 1; i < 10; i++) {
                // さっきより高度が低い=より下にいる（地面に近い）
                if ((alt_change[i-1] - alt_change[i]) > 0.05) cnt++;
            }
            if (cnt > 4) {
                addLogBuf("%3.1f c%d Fall", alt_change[9], cnt);
                isDetectFall = true;
            }
            //return MODE_LANDING;
        } 
        if (isDetectRise && isDetectFall) {

            // 分散
            double avg = 0, s = 0;
            for (int8_t i = 0; i < 10; i++) {
                avg += alt_change[i];
            }
            avg /= 10;
            for (int8_t i = 0; i < 10; i++) {
                s += (alt_change[i]-avg)*(alt_change[i]-avg);
            }
            s /= 10;
            printf("alt: %f, s: %f\n", alt_change[9], s);
            if (s < 0.06) {
                addLogBuf("%3.1f s%1.2f a%3.0f Land");
                return MODE_NICHROME;
            }
        } else {
            //landingCnt = 0;
            // 300s
            if (landingCnt > (300*SEC2CNT)) {
                if (abs(alt_change[9]-alt_ref) < 10) {
                    addLogBuf("lcnt%4d Land", landingCnt);
                    return MODE_NICHROME;
                } else if (landingCnt > (500*SEC2CNT)) {
                    addLogBuf("lcnt%4d Land", landingCnt);
                    return MODE_NICHROME;
                }
            } else {
                landingCnt++;
            }
        }
        return MODE_LANDING;
    }

    int8_t nichrome() {
        if (expansionCnt > (3*SEC2CNT)) {
            expansionCnt = 0;
            gpio_put(pin_nichrome_left, 0);
            addLogBuf("NC %d Done", expansionCnt);
            printf("nichrome done\n");
            return MODE_GNSS;
        } else {
            gpio_put(pin_nichrome_left, 1);
            addLogBuf("NC %d Heat", expansionCnt);
            expansionCnt++;
            return MODE_NICHROME;
        }
    }

    int8_t gnss() {
        static float distlog[20] = {0};
        static int8_t stack_cnt = 0;
        static bool stack_left = true;
        float euler[3] = {0};
        if (gps.isReady()) {

            gps.calc();
            //printf("gps ready\n");
            float dist = gps.getDistance();

            for (int8_t i = 0; i < 20-1; i++) distlog[i] = distlog[i+1];
            distlog[19] = dist;
            int8_t dcnt = 0;
            if (abs(distlog[0]-distlog[10]) < 0.15) dcnt++;
            if (abs(distlog[11]-distlog[19]) < 0.15) dcnt++;
            if (dcnt >= 2) {
                is_stack = true;
                if (stack_left) {
                    stack_left = false;
                    motor.forward(1023, 0);
                    addLogBuf("stack left");
                } else {
                    stack_left = true;
                    motor.backward(1023);
                    addLogBuf("stack right");
                }
            }
            if (is_stack) {
                if (stack_cnt > 5*SEC2CNT) {
                    motor.stop();
                    is_stack = false;
                    addLogBuf("release stack cnt");
                } else {
                    stack_cnt++;
                    return MODE_GNSS;
                }
            }

            if (dist < 5.0f) {
                gps.setFx([](double x) {return x;});                    
            }
            if (dist < 3.0f) {
                forwardYaw = gps.getDirection();
            }
            if (dist < 4.0f) {
                return MODE_CAM;
            }
            float dir = gps.getDirection();
            imu.getAttEuler(euler);
            float yaw = -euler[2];
            printf("dist: %f, dir: %f, yaw: %f\n", dist, dir, yaw);
            if ((yaw > dir-angle_th) && (yaw < dir+angle_th)) {
                printf("forward\n");
                addLogBuf("%2.1f %3.0f f", dist, dir);
                motor.forward(1023);
            } 
            else if (yaw < (dir-angle_th)) {
                printf("rightM\n");
                addLogBuf("%2.1f %3.0f r", dist, dir);
                motor.forward(1023,700);
            } else if (yaw > (dir+angle_th)) {
                addLogBuf("%2.1f %3.0f l", dist, dir);
                printf("leftM\n");
                motor.forward(700, 1023);
            } else {
                printf("sikatanaku rightM\n");
                motor.forward(1023,700);
            }
        }
        return MODE_GNSS;
    }

    int8_t camera() {
        static bool iscap = true;
        if (iscap) {
            printf("capture");
            cam.capture();
            iscap = false;
        } else {
            printf("no-capture");
            iscap = true;

            uint32_t size = cam.getJpegSize();
            //        printf("last: ");
            //        for (uint32_t i = size - 10; i < size; i++)
            //            printf("%02x", cam.image_buf[i]);
            //        printf(", size: %d\n", size);

            memset(red_area, 0, sizeof(red_area));
            JRESULT res = tjpg.prepare(cam.image_buf, size);
            if (res == JDR_OK) {
                printf("Image size is %u x %u.\n%u bytes of work area is free.\n", tjpg.jdec.width, tjpg.jdec.height, tjpg.jdec.sz_pool);
                res = tjpg.decomp(my_out_func, 3);
                // 160x120 -> (1/2^3) -> 20x15
                if (res == JDR_OK) {
                    printf("\rDecompression succeeded.\n");
                    count_vert(vert, red_area);
                    printf("count: %d %d %d %d %d\n", vert[4], vert[3], vert[2], vert[1], vert[0]);
                } else {
                    printf("jd_decomp() failed (rc=%d)\n", res);
                }
            } else {
                printf("jd_prepare() failed (rc=%d)\n", res);
            }

            int8_t dire = 0;
            int8_t most = vert[0];

            for (int8_t q = 0; q < 5; q++) {
                if (vert[q] > most) {
                    most=vert[q];
                    dire = q;
                }
            }

            switch (dire) {
                case 0:
                    if (vert[0] == 0) {
                        printf("ゴールは見当たりません");
                        motor.forward(1023, 700);
                    } else {
                        motor.forward(1023, 800);
                        printf("右前方ゴールです\n");
                    }
                    if (most > 20) return MODE_GOAL;
                    break;
                case 1:
                    printf("少し右前方ゴールです\n");
                    motor.forward(1023, 900);
                    break;
                case 2:
                    printf("前方ゴールです\n");
                    motor.forward(1023);
                    break;
                case 3:
                    motor.forward(900, 1023);
                    printf("少し左前方ゴールです\n");
                    break;
                case 4:
                    motor.forward(700, 1023);
                    printf("左前方ゴールです\n");
                    break;
            }
        }

        return MODE_CAM;
    }

    int8_t goal() {
        static bool is_first = true;
        static bool is_goal = false;

        if (is_first) {
            is_first = false;
        }
        if (is_goal) {
            motor.stop();
            return MODE_GOAL;
        }
        if (gps.isReady()) {
            float dist = 0;
            if ((dist = gps.getDistance()) < 5.0f) {
                printf("d:%2.0fm ok", dist);
                printf("goal\n");
                is_goal = true;
                cam.capture();
                logging.storeImg(cam.image_buf, cam.getJpegSize());
            } else {
                printf("onlyGnss\n");
                isOnlyGnss = true;
                return MODE_GNSS;
            }
        }
        return MODE_GOAL;
    }

    int8_t showlog() {
        char c;
        printf("press s to show\n");
        while (1) {
            if ((c = getchar_timeout_us(1000)) == 's') {
                logging.showAll();
            }
            sleep_ms(10);
        }
    }

    bool isStack() {
        return (is_stack);
    }
private:
    // landing
    float alt_change[10];
    bool isDetectRise;
    bool isDetectFall;
    uint16_t landingCnt;
    // expansion, fowardLanding
    uint16_t expansionCnt;
    // tof dame
    bool isOnlyGnss;
    // forwardTof
    float forwardYaw;
    bool is_stack = false;
} mode;

bool rtCallback(repeating_timer_t* rt) {
    static int16_t i = 0;
    //    printf("%d-", time_us_32()/1000);
    imu.update();
    if (i == 25) {
        rt_flag = true;
        i = 0;
        aupdate();
    } else {
        i++;
    }
    //    printf("-%d\n", time_us_32()/1000);
    return true;
}

void aupdate() {
    uint32_t before = time_us_32();
    printf("mode_now: %d\n", mode_now);
    static int8_t cnt = 0;
    static bool islog = false;
    int ret = mode_now;
    switch (mode_now)
    {
        case MODE_WAIT:
            cnt++;
            if (cnt > 5) ret = MODE_LANDING;
            break;
        case MODE_LANDING:
            ret = mode.landing();
            break;
        case MODE_NICHROME:
            gpio_put(pin_led, 1);
            ret = mode.nichrome();
            break;
        case MODE_GNSS:
            gpio_put(pin_led, 0);
            ret = mode.gnss();
            break;
        case MODE_CAM:
            ret = mode.camera();
            break;
        case MODE_GOAL:
            ret = mode.goal();
            break;
        case MODE_SHOWLOG:
            ret = mode.showlog();
            break;
    }
    mode_now = ret;
    if (islog) {
        float euler[3];
        imu.getAttEuler(euler);
        float roll = euler[0]*RAD2DEG;
        float pitch = euler[1]*RAD2DEG;
        float yaw = -euler[2]*RAD2DEG;
        logging.addLog(gps.getLatitude(), gps.getLongitude(), gps.getDistance(),
                       yaw, roll, pitch, gps.getDirection(),
                       motor.getPwmLeft(), motor.getPwmRight(),
                       mode_now, mode.isStack(),
                       logbuf);
    }
    islog = (islog ? (false) : (true));
    uint32_t et = time_us_32() - before;
    printf("elapsed: %d ms\n", et / 1000);
}

// dirty zone

static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect)
{
    // ref:https://qiita.com/yosihisa/items/c326e59ca5d65f35a181
    uint8_t *src;
    uint16_t x, y; //, bws;
    uint8_t r, g, b;
    double h = 0, s, v;
    uint32_t bitshift = 0;


    //    printf("\n");
    src = (uint8_t *)bitmap;
    //    bws = N_BPP * (rect->right - rect->left + 1);
    for (y = 0; y < (rect->bottom - rect->top + 1); y++)
    {
        for (x = 0; x < (rect->right - rect->left + 1); x += 1)
        {
            //            printf("y: %d\n", y);
            r = *(src + 3 * (y * (rect->right - rect->left + 1) + x));
            g = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 1);
            b = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 2);
            //            printf("(%03d,%03d,%03d),", r,g,b);
            double MAX = max((max(r, g)), b);
            double MIN = min((min(r, g)), b);
            v = MAX / 256 * 100;

            if (MAX == MIN)
            {
                h = 0;
                s = 0;
            }
            else
        {
                if (MAX == r)
                    h = 60.0 * (g - b) / (MAX - MIN) + 0;
                else if (MAX == g)
                    h = 60.0 * (b - r) / (MAX - MIN) + 120.0;
                else if (MAX == b)
                    h = 60.0 * (r - g) / (MAX - MIN) + 240.0;

                if (h > 360.0)
                    h = h - 360.0;
                else if (h < 0)
                    h = h + 360.0;
                s = (MAX - MIN) / MAX * 100.0;
            }

            if (h > 360.0)
                h -= 360;

            // 赤色の判定
            if ((h >= H_MIN_1 && h <= H_MAX_1) || (h >= H_MIN_2 && h <= H_MAX_2))
            {
                if ((s >= S_MIN && s <= S_MAX) && (v >= V_MIN && v <= V_MAX))
                {
                    //                    printf("red!: (rx,ry)=(%d,%d)\n", rect->left+x, rect->top+y);
                    bitshift = (rect->left + x);
                    red_area[rect->top + y] |= (1 << bitshift);
                }
            }
        }
    }

    return 1; // Continue to decompress
}

void count_vert(uint8_t vert[5], uint32_t area[15])
{
    // count ones in vertically devided area
    // [4] [3] [2] [1] [0]
    // max count in each area: 4*15=60
    // 左右・上下反転に注意
    memset(vert, 0, 5);
    uint8_t bit;
    for (uint8_t y = 0; y < 15; y++)
    {
        for (int8_t x = 20 - 1; x >= 0; x--)
        {
            bit = (area[y] >> x) & 1;
            if (bit == 1)
            {
                if ((x >= 0) && (x < 4))
                    vert[0]++;
                else if (x < 8)
                    vert[1]++;
                else if (x < 12)
                    vert[2]++;
                else if (x < 16)
                    vert[3]++;
                else
                    vert[4]++;
            }
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    // gnss 1on
    gpio_init(pin_gnss_vcc);
    gpio_set_dir(pin_gnss_vcc, GPIO_OUT);
    gpio_put(pin_gnss_vcc, 0);

    // motor
    motor.init(pin_motor1_a, pin_motor2_a);
    //
    //
    motor.setDirForward(-1, 1);

    // i2c1: prs, imu
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(pin_i2c1_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_i2c1_scl, GPIO_FUNC_I2C);

    // imu
    bool st = imu.init();
    printf("IMU init %s\n", st?"ok":"fail");

    // prs
    st = prs.init();
    printf("Press init %s\n", st?"ok":"fail");
    for (int8_t i = 0; i < 10; i++) {
        prs.getAltM();
        sleep_ms(50);
    }
    mode.alt_ref = prs.getAltM();

    // nichrome
    gpio_init(pin_nichrome_left);
    gpio_set_dir(pin_nichrome_left, GPIO_OUT);
    gpio_put(pin_nichrome_left, 0);

    // led
    gpio_init(pin_led);
    gpio_put(pin_led, 0);

    // gnss
    gps.init(pin_uart0_mosi, pin_uart0_miso);

    // camera
    cam.init();
    cam.enableJpeg();

    // logging
    bool ret = logging.init();
    printf("logging: %s\n", ret?"ok":"fail");

    // imu calibration
    #ifdef ALWAYS_CALIB
    printf("always calibrate mode. calibrating...\n");
    imu.calibration();
    logging.storeCalibData(imu.co);
    #else
    ret = logging.readCalibData(imu.co);
    if (!ret) {
        printf("no calibration data found. calibrating...\n");
        imu.calibration();
        logging.storeCalibData(imu.co);
    } else {
        printf("found calibration data\n");
    }
    #endif

    add_repeating_timer_ms(-20, &rtCallback, NULL, &timer);

    while (1) {
        if (rt_flag) {
            printf("r %d ", rt_flag);
            //            aupdate();
            rt_flag = false;
        }   
    }
}
