#include "ulog.h"

//LFS LFS::

#define YRP_DEG2DIV (0.0888888888888) // 32/360
#define YRP_DIV2DEG (11.25) // 360/32

Log::Log(uint8_t code) {
    this->code = code;
    this->time.year  = 2023;
    this->time.month = 06;
    this->time.day   = 17;
    this->time.dotw  = 6; // 0 is Sunday, so 5 is Friday
    this->time.hour  = 00;
    this->time.min   = 00;
    this->time.sec   = 00;

    rtc_init();
    rtc_set_datetime(&time);
    sleep_us(64);
}

bool Log::init() {
    int err;
    err = LFS::init();
    //printf("init: %d: %s\n", err, (err>=0)?"ok":"fail"); 
    if (err < 0) return false;
    // mount the filesystem
    err = LFS::mount();
    //printf("mount: %d: %s\n", err, (err>=0)?"ok":"fail");  
    if (err) {
        printf("Log: no lfs system found, format...\n");
        LFS::format();
        LFS::mount();
    }
    err = LFS::file_open(&file_log, "logging", LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
    //printf("file_open: %d: %s\n", err, (err>=0)?"ok":"fail"); 
    if (err < 0) return false;
    err = LFS::file_close(&file_log);
    //printf("format: %d: %s\n", err, (err>=0)?"ok":"fail"); 
    return (err >= 0);
}

bool Log::addLog(int32_t lat, int32_t lon, float dist,
                 float yaw, float roll, float pitch, float yaw_goal,// deg -180~180
                 int16_t motor_left, int16_t motor_right,
                 uint8_t seq, bool stack,
                 uint8_t buf[17]) {
    encodeLine(wbuf, lat, lon, dist, yaw, roll, pitch, yaw_goal,
               motor_left, motor_right, seq, stack, buf);
    bool err = addLog(wbuf);
    return (err);
}

bool Log::addLog(uint8_t src[32]) {
    static bool is_first = true;
    memcpy(wbuf, src, 32);
    LFS::file_open(&file_log, "logging", LFS_O_RDWR | LFS_O_APPEND);
    if (is_first) {
        LFS::file_rewind(&file_log);
        is_first = false;
    }
    // write to LFS file
    assert(sizeof(wbuf) == 32);
    int err = LFS::file_write(&file_log, wbuf, sizeof(wbuf));
    LFS::file_close(&file_log);
    return (err > 0);
}

void Log::encodeLine(uint8_t dst[32],
                     int32_t lat, int32_t lon, float dist,
                     float yaw, float roll, float pitch, float yaw_goal,// deg -180~180
                     int16_t motor_left, int16_t motor_right,
                     uint8_t seq, bool stack,
                     uint8_t buf[17]) {
    uint8_t mleft = (uint8_t)((float)abs(motor_left) * (4.0/1023.0)) & 0b0011;
    uint8_t mright = (uint8_t)((float)abs(motor_right) * (4.0/1023.0)) & 0b0011;
    //uint8_t buf[12] = {' '};
    rtc_get_datetime(&time);
    // 機体コード
    dst[0] = (code << 4) | ((time.min&0xf0) >> 4);
    // rtc
    // time
    dst[1] = ((time.min&0x0f) << 4) | ((time.sec&0xf0) >> 4);
    dst[2] = ((time.sec&0x0f) << 4) | (mleft << 2) | (mright);
    // gps
    // latitude
    dst[3] = (lat >> 24) & 0xff;
    dst[4] = (lat >> 16) & 0xff;
    dst[5] = (lat >> 8) & 0xff;
    dst[6] = (lat & 0xff);
    // longitude
    dst[7] = (lon >> 24) & 0xff;
    dst[8] = (lon >> 16) & 0xff;
    dst[9] = (lon >> 8) & 0xff;
    dst[10] = (lon & 0xff);
    // yaw
    // y5 y4 y3 y2 y1 r5 r4 r3
    yaw += 180.0;
    uint8_t tmp = (uint8_t)(yaw*YRP_DEG2DIV+0.5);
    dst[11] = (tmp & 0b00011111) << 3;
    // roll
    tmp = (uint8_t)(roll*YRP_DEG2DIV+0.5);
    dst[11] |= (tmp & 0b00011100) >> 2;
    // r2 r1 p5 p4 p3 p2 p1 stack
    dst[12] = (tmp & 0b00000011) << 6;
    // pitch
    tmp = (uint8_t)(pitch*YRP_DEG2DIV+0.5);
    dst[12] |= (tmp & 0b00011111) << 1;
    dst[12] |= (stack?1:0) & 0x01;
    // goal yaw
    // y5 y4 y3 y2 y1 s3 s2 s1
    tmp = (uint8_t)(yaw_goal*YRP_DEG2DIV+0.5);
    dst[13] = (tmp & 0b00011111) << 3;
    // seq number
    dst[13] |= (seq & 0b0111);
    // dist from goal
    dst[14] = (uint8_t)dist;
    // data
    for (int i = 0; i < 17; i++) dst[i+15] = buf[i];
}

void Log::showAll() {
    uint8_t buf[32] {'\0'};
    int32_t head = 0;
    LFS::file_open(&file_log, "logging", LFS_O_RDONLY);
    printf("code,min.sec,lat,lon,dist,yaw,roll,pitch,yaw_to_goal,motor_left,motor_right,stack,seq,data\n");
    int32_t size = LFS::file_size(&file_log);
    while (head < size) {
        LFS::file_read(&file_log, buf, sizeof(buf));
        this->showLine(buf);
        head += 32;
        LFS::file_seek(&file_log, head, LFS_SEEK_SET);
    }
}

void Log::showLine(uint8_t buf[32]) {
    int32_t lat, lon;
    uint8_t code, min, sec, mleft, mright, seq, dist;
    uint8_t cdata[17];
    float yaw, roll, pitch, yaw_goal;
    bool stack;
    this->decodeLine(buf, &code, &min, &sec, &lat, &lon, &dist, 
                     &yaw, &roll, &pitch, &yaw_goal,
                     &mleft, &mright, &seq, &stack, cdata);
    printf("%u,%u.%u,%d,%d,%u,%3.2f,%3.1f,%3.1f,%3.1f,%u,%u,%d,%u,",
           code, min, sec, lat, lon, dist, yaw, roll, pitch, yaw_goal,
           mleft, mright, stack, seq);
    for (int8_t i = 0; i < 17; i++) printf("%c", cdata[i]);
    printf("\n");

}

void Log::decodeLine(uint8_t raw[32],
                     uint8_t* code, uint8_t* min, uint8_t* sec,
                     int32_t* lat, int32_t* lon, uint8_t* dist,
                     float* yaw, float* roll, float* pitch, float* yaw_goal,
                     uint8_t* motor_left, uint8_t* motor_right,
                     uint8_t* seq, bool* stack,
                     uint8_t buf[17]) {
    uint8_t tmp = 0;
    *code = raw[0] >> 4;
    *min = (raw[0] << 4) | (raw[1] >> 4);
    *sec = (raw[1] << 4) | (raw[2] >> 4);
    *motor_left = (raw[2] & 0b00001100) >> 2;
    *motor_right = (raw[2] & 0b00000011);
    *lat = (raw[3] << 24) | (raw[4] << 16) | (raw[5] << 8) | raw[6];
    *lon = (raw[7] << 24) | (raw[8] << 16) | (raw[9] << 8) | raw[10];
    tmp = (raw[11] >> 3) & 0b00011111;
    *yaw = YRP_DIV2DEG*(float)tmp;
    tmp = ((raw[11] << 2) & 0b00011100) | ((raw[12] >> 6) & 0b00000011);
    *roll = YRP_DIV2DEG*(float)tmp;
    tmp = (raw[12] >> 1) & 0b00011111;
    *pitch = YRP_DIV2DEG*(float)tmp;
    *stack = (raw[12]&0x01)?true:false;
    tmp = (raw[13] >> 3) & 0b00011111;
    *yaw_goal = YRP_DIV2DEG*(float)tmp;
    *seq = (raw[13] & 0b0111);
    *dist = raw[14];
    for (int8_t i = 0; i < 17; i++) buf[i] = raw[i+15];
}

bool Log::storeCalibData(double co[4]) {
    int ret;
    ret = LFS::file_open(&file_calib, "calib", LFS_O_RDWR | LFS_O_CREAT);
    if (ret < 0) return false;
    ret = LFS::file_rewind(&file_calib);
    if (ret < 0) return false;
    ret = LFS::file_write(&file_calib, co, sizeof(double)*4);
    if (ret < 0) return false;
    ret = LFS::file_close(&file_calib);
    return (ret>=0);
}

bool Log::readCalibData(double co[4]) {
    int ret = LFS::file_open(&file_calib, "calib", LFS_O_RDONLY);
    printf("Log: calib data %s\n", (ret>=0)?"found":"not found");
    if (ret < 0) return false;
    ret = LFS::file_read(&file_calib, co, sizeof(double)*4);
    if (ret < 0) return false;
    ret = LFS::file_close(&file_calib);
    return (ret>=0);
}

bool Log::storeImg(uint8_t* img, int32_t size) {
    int ret;
    ret = LFS::file_open(&file_img, "img", LFS_O_RDWR | LFS_O_CREAT);
    if (ret < 0) return false;
    ret = LFS::file_write(&file_img, img, size);
    if (ret < 0) return false;
    ret = LFS::file_close(&file_img);
    return (ret >= 0);
}

bool Log::readImg(uint8_t* img, int16_t* read_size, int32_t max_size) {
    int16_t ret;
    ret = LFS::file_open(&file_img, "img", LFS_O_RDWR | LFS_O_CREAT);
    if (ret < 0) return false;
    ret = LFS::file_read(&file_img, img, max_size);
    if (ret < 0) return false;
    else {
        *read_size = ret;
    }
    ret = LFS::file_close(&file_img);
    return (ret >= 0);
}
