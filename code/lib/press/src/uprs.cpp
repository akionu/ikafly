#include "uprs.h"
#include "hardware/i2c.h"

Press::Press(i2c_inst_t* i2c) {
	Press(i2c, 0x77);
}

Press::Press(i2c_inst_t* i2c, uint8_t addr) {
    config.addr = addr;
    config.i2c = i2c;
    config.mode = BGD_PRS_TEMP;
    config.prs_config[0] = 2;
    config.prs_config[1] = 128;
    config.temp_config[0] = 2;
    config.temp_config[1] = 8;
}

bool Press::init() {
    int8_t ret = spl06_init(&config, &coef);
	return ((ret==0)?true:false);
}

float Press::getAltM() {
    spl06_read_press_cal(&config, &coef, &prs);
    alt = spl06_calc_alt(prs);
    return alt;
}

float Press::getPressHpa() {
   	spl06_read_press_cal(&config, &coef, &prs);
	return prs;
}

float Press::getTempDegC() {
    spl06_read_temp_cal(&config, &coef, &temp);
	return temp;
}
