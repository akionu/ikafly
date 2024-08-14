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
#include "tjpg.hpp"
#include "cam.h"

#include "ulog.h"
#include "ugps.h"
#include "angle.h"

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
Log logging(CODE);
Radio rf(pin_rf_mosi, pin_rf_miso);

static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect);
void count_vert(uint8_t vert[5], uint32_t area[15]);
void aupdate();
// NSE19(8/16) lat: 40.142646, long: 139.987610
// NSE19(8/17) lat: 40.142647, long: 139.987595
// 3448.09403,N,13546.25752,
// 34801609, 135770979
GPS gps(
    GOAL_LAT, // goal_lat
    GOAL_LONG, // goal_long
    111, 92, // noshiro
#if CODE==1
    [](double x) {return (x);}
    #elif CODE==2
       [](double x) {return (x*x);}
#elif CODE==3
[](double x) {return (x*(2-x));}
#elif CODE==4
[](double x) {return ((x*x)*(3-2*x));}
#elif CODE==5
[](double x) {return (x);}
#elif CODE==6
[](double x) {return (x*x);}
#elif CODE==7
[](double x) {return (x*(2-x));}
#elif CODE==8
[](double x) {return ((x*x)*(3-2*x));}
#elif CODE==9
[](double x) {return (x);}
#elif CODE==10
[](double x) {return (x*x);}
#endif
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
uint8_t rsvdata[32];
bool rt_flag = false;

//bool bnoIsEnoughAccuracy() {
//    double accq = //bno08x.getQuatRadianAccuracy();
//    double accd = accq * CONST_180_DIVIDED_BY_PI;
//    printf("acc: rad: %f, deg: %f\n", accq, accd);
//    if (accd < 15.0f) return true;
//    else return false;
//}
#define H_MIN_1 0 // 固定
#define H_MAX_1 30
#define H_MIN_2 300
#define H_MAX_2 360 // 固定
// Satuation
#define S_MIN 10
#define S_MAX 100
// Value
#define V_MIN 20
#define V_MAX 100

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
                if ((alt_change[i] - alt_change[i-1]) > RISE_TH) cnt++;
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
                if ((alt_change[i-1] - alt_change[i]) > FALL_TH) cnt++;
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
            if (landingCnt > (CNT_LAND_ALTREF*SEC2CNT)) {
                if (abs(alt_change[9]-alt_ref) < 10) {
                    addLogBuf("lcnt%4d Land", landingCnt);
                    return MODE_NICHROME;
                } else if (landingCnt > (CNT_LAND_NOALT*SEC2CNT)) {
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
        static float dist = 0, dir = 0;
        if (gps.isReady()) {

            gps.calc();
            //printf("gps ready\n");
            dist = gps.getDistance();

            for (int8_t i = 0; i < 20-1; i++) distlog[i] = distlog[i+1];
            distlog[19] = dist;
            int8_t dcnt = 0;
            if (abs(distlog[0]-distlog[10]) < 0.15) dcnt++;
            if (abs(distlog[11]-distlog[19]) < 0.15) dcnt++;
            if (dcnt >= 2) {
                // 左右・前後を切り替え
                is_stack = true;
                if (stack_left) {
                    stack_left = false;
                    motor.forward(1023, 0);
                    addLogBuf("stack left");
                } else {
                    stack_left = true;
                    motor.backward(1023);
                    addLogBuf("stack back");
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

            if (dist < 10.0f) {
                gps.setFx([](double x) {return x;});
            }
            if (dist < 4.0f) {
                return MODE_CAM;
            }

            dir = gps.getDirection();
        }
        imu.getAttEuler(euler); 
        //            angle_t vpi_d, vyaw, vgps, vcur;
        //            Angle::deg2vec(M_PI_2, &vpi_d);
        //            Angle::deg2vec(-euler[2], &vyaw);
        //            Angle::deg2vec(dir, &vgps);
        //            Angle::minus(vyaw, vpi_d, &vcur);
        //            float yaw = acos(vcur.ct);
        //            float theta = Angle::theta(vgps, vcur);

        float yaw = euler[2];
        //            float yaw = gps.getCource_rad();
        //            float speed = gps.getSpeed_mps();
        float theta = Angle::theta(dir, yaw);

        printf("dist: %f, dir: %3.2f, yaw: %3.2f, theta: %3.2f\n", dist, dir*RAD2DEG, yaw*RAD2DEG, theta*RAD2DEG);
        //            printf("dist: %f, dir: %3.2f, e2: %3.2f, yaw: %3.2f, theta: %3.2f\n", dist, dir*RAD2DEG, -euler[2]*RAD2DEG,yaw*RAD2DEG, theta*RAD2DEG);
        //            if (speed < 0.05) {
        //                printf("forward because too slow\n");
        //                addLogBuf("s%3.2f Forward", theta);
        //            } else 
        if (abs(theta) < angle_th) {
            printf("forward\n");
            addLogBuf("t%3.2f Forward", theta);
            motor.forward(1023);
        } else if (theta > 0) {
            printf("rightM\n");
            addLogBuf("t%3.2f Right", theta);
            motor.forward(1023, 550);
        } else if (theta < 0) {
            addLogBuf("t%3.2f Left", theta);
            printf("leftM\n");
            motor.forward(550, 1023);
        } else {
            printf("sikatanaku rightM\n");
            motor.forward(1023,700);
        }
        return MODE_GNSS;
    }

    int8_t camera() {
        static bool iscap = true;
        const uint8_t goal_th = 30;
        static uint16_t found_cnt = 0;

        if (iscap) {
            printf("capture\n");
            cam.capture();
            iscap = false;
        } else {
            printf("no-capture\n");
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

            if (most < 10) {
                printf("Cam: No Goal Found\n");
                addLogBuf("%02d %02d %02d %02d %02dNO GOAL", vert[4], vert[3], vert[2], vert[1], vert[1], vert[0]);
                found_cnt = 0;
                motor.forward(1023, 400);
                return MODE_CAM;
            } else {
                found_cnt++;
            }

            switch (dire) {
                case 0:
                    printf("Cam: Right\n");
                    addLogBuf("%02d %02d %02d %02d %02dRight", vert[4], vert[3], vert[2], vert[1], vert[1], vert[0]);
                case 1:
                    printf("Cam: a bit Right\n");
                    addLogBuf("%02d %02d %02d %02d %02dsRight", vert[4], vert[3], vert[2], vert[1], vert[1], vert[0]);
                    motor.forward(1023, 800);
                    break;
                case 2:
                    printf("Cam: Forward\n");
                    addLogBuf("%02d %02d %02d %02d %02dForward", vert[4], vert[3], vert[2], vert[1], vert[1], vert[0]);
                    motor.forward(1023);
                    break;
                case 3:
                    printf("Cam: a bit Left\n");
                    addLogBuf("%02d %02d %02d %02d %02daLeft", vert[4], vert[3], vert[2], vert[1], vert[1], vert[0]);
                    motor.forward(880, 1023);
                    break;
                case 4:
                    printf("Cam: Left\n");
                    addLogBuf("%02d %02d %02d %02d %02daLeft", vert[4], vert[3], vert[2], vert[1], vert[1], vert[0]);
                    motor.forward(650, 1023);
            }

            if ((found_cnt >= 5) && (most > goal_th)) {
                return MODE_FORWARD_CAM;
            } else {
                return MODE_CAM;
            }
        }
        return MODE_CAM;
    }

    int8_t forwardCam() {
        static bool is_first = true;
        static int8_t cnt = 0;

        addLogBuf("forwardCam %d", cnt);
        if (is_first) {
            is_first = false;
            cnt = 0;
            return MODE_FORWARD_CAM;
        } else if (cnt < 4*SEC2CNT){
            cnt++;
            motor.forward(1023);
            return MODE_FORWARD_CAM;
        } else {
            is_first = true;
            return MODE_GOAL;
        }
    }

    int8_t goal() {
        static bool is_first = true;
        static bool is_goal = false;
        float dist = 0.0f;

        if (is_first) {
            is_first = false;
        }
        if (is_goal) {
            motor.stop();
            addLogBuf("%2.2f GOAL!", dist);
            return MODE_GOAL;
        }
        if (gps.isReady()) {
            //            if (true) { // 忘れずに書き換え！！！
            if ((dist = gps.getDistance()) < 5.0f) {
                printf("%2.1f GOAL!", dist);
                addLogBuf("%2.2f GOAL!", dist);
                printf("goal\n");
                is_goal = true;
                cam.capture();
                logging.storeImg(cam.image_buf, cam.getJpegSize());
            } else {
                printf("onlyGnss\n");
                addLogBuf("Goal: OnlyGNSS");
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
    float alt_change[10];
    bool isDetectRise;
    bool isDetectFall;
    uint16_t landingCnt;
    uint16_t expansionCnt;
    bool isOnlyGnss;
    bool is_stack = false;
} mode;

bool rtCallback(repeating_timer_t* rt) {
    static int16_t i = 0;
    static float euler[3];
    //    printf("%d-", time_us_32()/1000);
    imu.update();
    if (i == 25) {
        motor.stop();
        rt_flag = true;
//        imu.getAttEuler(euler);
        //		printf("%+03.2f %+03.2f %+03.2f\n", euler[0]*RAD2DEG, euler[1]*RAD2DEG, euler[2]*RAD2DEG);
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
    static uint8_t lbuf[32] = {0};
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
        float yaw = euler[2]*RAD2DEG;
        logging.encodeLine(lbuf, gps.getLatitude(), gps.getLongitude(), gps.getDistance(),
                           yaw, roll, pitch, gps.getDirection(),
                           motor.getPwmLeft(), motor.getPwmRight(),
                           mode_now, mode.isStack(),
                           logbuf);
        //        printf("x,x.xx,%d,%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%d,%d,%d,%d\n",
        //               gps.getLatitude(), gps.getLongitude(), gps.getDistance(),
        //               yaw, roll, pitch, gps.getDirection(),
        //               motor.getPwmLeft(), motor.getPwmRight(),
        //               mode.isStack(), mode_now);
        logging.showLine(lbuf);
        logging.addLog(lbuf);
        //        if (rf.is_air_clear()) rf.send(lbuf);
    }
    islog = (islog ? (false) : (true));
    uint32_t et = time_us_32() - before;
    printf("elapsed: %d ms\n", et / 1000);
    printf("\n");
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

//void receiveCallback(uint gpio, uint32_t events) {
//    rf.receive(rsvdata);
//    logging.showLine(rsvdata);
//}

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
    motor.setDirForward(MLEFT, MRIGHT);

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
    printf("co: %f %f %f %f\n", imu.co[0], imu.co[1], imu.co[2], imu.co[3]);
#endif

    add_repeating_timer_ms(-20, &rtCallback, NULL, &timer);
    //    gpio_set_irq_enabled_with_callback(pin_rf_miso, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_LEVEL_LOW, true, &receiveCallback);

    while (1) {
        if (rt_flag) {
            //aupdate();
            rt_flag = false;
        }   
    }
}
