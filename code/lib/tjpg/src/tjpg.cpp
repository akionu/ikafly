#include "tjpg.hpp"
#include "../../freertos/config/FreeRTOSConfig.h"
#include "../../freertos/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "../../freertos/FreeRTOS-Kernel/include/task.h"


uint16_t TJPGD::size_img = 0;
uint16_t TJPGD::in_func_head = 0;

// public:
JRESULT TJPGD::prepare(uint8_t* img, uint16_t size_img) {
    memset((void*)&jdec, 0, sizeof(jdec));
    this->fp = img;
    TJPGD::size_img = size_img;
    TJPGD::in_func_head = 0;
    JRESULT res = jd_prepare(&jdec, TJPGD::in_func, (void*)work_area, TJPG_SZ_WORK, fp);
    return res;
}

JRESULT TJPGD::decomp(int (*outfunc)(JDEC*,void*,JRECT*), uint8_t scale) {
//JRESULT TJPGD::decomp(uint8_t scale) {
    JRESULT res = jd_decomp(&jdec, outfunc, scale);
    return res;
}

// private
size_t TJPGD::in_func(JDEC* jd, uint8_t* buff, size_t nbyte) {
    uint8_t* fp = (uint8_t*)jd->device;

    if (buff) { 
        memcpy(buff, (uint8_t*)fp+in_func_head, nbyte);
        in_func_head += nbyte;
        return nbyte;
    } else {
        if (in_func_head+nbyte <= TJPGD::size_img) {
            in_func_head += nbyte;
            return nbyte;
        } else {
            return 0;
        }
    }
}

