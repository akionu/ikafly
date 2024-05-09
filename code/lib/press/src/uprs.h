#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

extern "C" {
    #include "./pico-spl06-001/src/pico-spl06-001.h"
}

class Press {
    public:
        Press(i2c_inst_t* i2c);
        Press(i2c_inst_t* i2c, uint8_t addr);
		bool init(); // return true if success, false fail.
		float getPressHpa();
        float getAltM();
		float getTempDegC();

    private:
        float prs;
        float alt;
		float temp;
        spl06_config_t config;
        spl06_coef_t coef;
};
