#include "ulog.h"

Log::Log(uint8_t code) {
    this->code = code;
    this->head = 0;
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

bool Log::addLog(int32_t lat, int32_t lon,
                float yaw, float roll, float pitch,
                uint8_t seq, 
                uint8_t buf[12]) {
    //uint8_t buf[12] = {' '};
    rtc_get_datetime(&time);
    // 機体コード
    bufw[0] = code;
    // rtc
    // time
    bufw[1] = time.hour;
    bufw[2] = time.min;
    bufw[3] = time.sec;
    // gps
    // latitude
    bufw[4] = (lat >> 24) & 0xff;
    bufw[5] = (lat >> 16) & 0xff;
    bufw[6] = (lat >> 8) & 0xff;
    bufw[7] = (lat & 0xff);
    // longitude
    bufw[8] = (lon >> 24) & 0xff;
    bufw[9] = (lon >> 16) & 0xff;
    bufw[10] = (lon >> 8) & 0xff;
    bufw[11] = (lon & 0xff);
    // yaw
    int16_t tmp = (int16_t)(yaw * 100);
    bufw[12] = (tmp << 8);
    bufw[13] = (tmp & 0xff);
    // roll
    tmp = (int16_t)(roll * 100);
    bufw[14] = (tmp << 8);
    bufw[15] = (tmp & 0xff);
    // pitch
    tmp = (int16_t)(pitch * 100);
    bufw[16] = (tmp << 8);
    bufw[17] = (tmp & 0xff);
    // seq number
    bufw[18] = seq;
    // data
    for (int i = 0; i < 12; i++) bufw[i+19] = buf[i];
    // separator
    bufw[31] = '\n';

    return true;
}

void Log::showAll() {
    uint32_t now = 0;
    uint8_t buf[32] = {0};
    printf("code,hour.min.sec,lat,lon,yaw,roll,pitch,seq,data\n");
    while (now < head) {
        int ret = 0; // tmp
        now += ((ret > 0) ? ret : 0);
        if (buf[31] != '\n') {
            printf("error reading log: %d\n", now);
        }
        int32_t lat = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
        int32_t lon = (buf[8] << 24) | (buf[9] << 16) | (buf[10] << 8) | buf[11];
        printf("%c,%d.%d.%d,%d,%d,%3.2f,%3.2f,%3.2f,%d,", 
                    buf[0], // code
                    buf[1], // hour
                    buf[2], // min
                    buf[3], // sec
                    lat,
                    lon,
                    (float)((buf[12] << 8) | buf[13]) / 100.0f,
                    (float)((buf[14] << 8) | buf[15]) / 100.0f,
                    (float)((buf[16] << 8) | buf[17]) / 100.0f,
                    buf[18]
        );
        for (int i = 0; i < 12; i++) printf("%c", buf[i+19]);
        for (int i = 0; i < 32; i++) buf[i] = ' ';
        printf("\n");
    }
}
