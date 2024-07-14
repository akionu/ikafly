#pragma once
#include "tjpgd.h"
#include "tjpgdcnf.h"

#define TJPG_SZ_WORK 3100

class TJPGD {
public:
    JDEC jdec;
    JRESULT prepare(uint8_t* img, uint16_t size_img);
    JRESULT decomp(int (*outfunc)(JDEC*,void*,JRECT*), uint8_t scale);
private:
    uint8_t* fp;
    uint8_t work_area[TJPG_SZ_WORK];
    static uint16_t in_func_head;
    static uint16_t size_img;
    static size_t in_func(JDEC* jd, uint8_t* buff, size_t nbyte);
};
