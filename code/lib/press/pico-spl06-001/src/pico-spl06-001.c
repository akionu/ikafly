#include "pico-spl06-001.h"

int8_t spl06_init(spl06_config_t *config, spl06_coef_t *coef) {
    uint8_t tmp[2], buf;
    int8_t ret;

    // check if sensor could be accesed
    tmp[0] = SPL06_ID;
    i2c_write_blocking(config->i2c, config->addr, &tmp[0], 1, true);
    i2c_read_blocking(config->i2c, config->addr, &buf, 1, false);
    if (buf != 0x10) {
#ifndef NDEBUG
        printf("spl06: no spl06-001 found\n");
#endif
        return PICO_ERROR_GENERIC;
    }

    // read coef before measure
    ret = spl06_read_coef(config, coef);
    if (ret < 0) {
#ifndef NDEBUG
        printf("spl06: coef not ready\n");
#endif
        return PICO_ERROR_GENERIC;
    }

    // 104 ms per measure, 5 cm presision
    if (config->mode == CMD_PRS || config->mode == BGD_PRS || config->mode == BGD_PRS_TEMP)
        ret = spl06_config_prs(config, config->prs_config[0], config->prs_config[1]);
    if (config->mode == CMD_TEMP || config->mode == BGD_TEMP || config->mode == BGD_PRS_TEMP) 
        ret = spl06_config_temp(config, config->temp_config[0], config->temp_config[1]);

    // start measuring
    ret = spl06_set_mode(config, config->mode);
    if (ret < 0) {
#ifndef NDEBUG
        printf("spl06: init fails");
#endif
        return PICO_ERROR_GENERIC;
    }

    return 0;
}

int8_t spl06_set_mode(spl06_config_t *config, spl06_mode_t mode) {
    uint8_t tmp[2];

    tmp[0] = SPL06_MEAS_CFG;
    switch (mode) {
    case STANDBY:
        tmp[1] = 0x00;
        break;

    case CMD_PRS:
        tmp[1] = 0x01;
        break;
        
    case CMD_TEMP:
        tmp[1] = 0x02;
        break;

    case BGD_PRS:
        tmp[1] = 0x05;
        break;
    
    case BGD_TEMP:
        tmp[1] = 0x06;
        break;
    
    case BGD_PRS_TEMP:
        tmp[1] = 0x07;
    
    default:
        break;
    }
    int8_t ret = i2c_write_blocking(config->i2c, config->addr, tmp, 2, false);
    return ((ret > 0) ? (0) : (PICO_ERROR_GENERIC));
}

int8_t spl06_config_prs(spl06_config_t *config, uint8_t rate, uint8_t oversample) {
    uint8_t tmp[2];
    int16_t time; // mesurement time(ms)

    tmp[0] = SPL06_PRS_CFG;
    tmp[1] = oversample2config(oversample, &time);

    if ((time * (int16_t)rate) > 1000) return PICO_ERROR_GENERIC;
#ifndef NDEBUG
    printf("spl06: vaild combination of rate & oversample\n");
#endif

    tmp[1] |= rate2config(rate);

    i2c_write_blocking(config->i2c, config->addr, tmp, 2, false);

    if (oversample > 8) {
#ifndef NDEBUG
    printf("spl06: enable P shift\n");
#endif
        tmp[0] = SPL06_CFG_REG;
        i2c_write_blocking(config->i2c, config->addr, &tmp[0], 1, true);;
        i2c_read_blocking(config->i2c, config->addr, &tmp[1], 1, false);
        tmp[1] = 0x04;
        i2c_write_blocking(config->i2c, config->addr, tmp, 2, false);
    }

    return 0;
}

static int8_t oversample2config(uint8_t oversample, int16_t *time) {
    switch (oversample) {
    // measurement per second
    case 1:
	    *time = 4;
        return 0x00;
    case 2:
	    *time = 5;
        return 0x01;
    case 4:
	    *time = 8;
        return 0x02;
    case 8:
	    *time = 15;
        return 0x03;
    case 16:
	    *time = 28;
        return 0x04;
    case 32:
	    *time = 53;
        return 0x05;
    case 64:
	    *time = 104;
        return 0x06;  
    case 128:
	    *time = 207;
        return 0x07;
    default:
#ifndef NDEBUG
    printf("spl06: invaild oversample config\n");
#endif
        return PICO_ERROR_GENERIC;
    }
}

static int8_t rate2config(uint8_t rate) {
    switch (rate) {
    // mesurements per second
    case 1:
        return 0x00;
    case 2:
        return 0x10;
    case 4:
        return 0x20;
    case 8:
        return 0x30;
    case 16:
        return 0x40;
    case 32:
        return 0x50;
    case 64:
        return 0x60;
    case 128:
        return 0x70;
    default:
#ifndef NDEBUG
    printf("spl06: invaild rate config\n");
#endif
	    return PICO_ERROR_GENERIC;
    }
}

int8_t spl06_config_temp(spl06_config_t *config, uint8_t rate, uint8_t oversample) {
    uint8_t tmp[2];
    int16_t time;

    tmp[0] = SPL06_TMP_CFG;
    tmp[1] = oversample2config(oversample, &time);
    if ((time * (int16_t)rate) > 1000) return PICO_ERROR_GENERIC;

    tmp[1] |= rate2config(rate);
    tmp[1] |= (1<<7);
    i2c_write_blocking(config->i2c, config->addr, tmp, 2, false);

    if (oversample > 8) {
#ifndef NDEBUG
    printf("spl06: enable E shift\n");
#endif  
        tmp[0] = SPL06_CFG_REG;
        i2c_write_blocking(config->i2c, config->addr, &tmp[0], 1, true);;
        i2c_read_blocking(config->i2c, config->addr, &tmp[1], 1, false);
        tmp[1] |= (1<<3);
        i2c_write_blocking(config->i2c, config->addr, tmp, 2, false);
    }

}

int8_t spl06_read_coef(spl06_config_t *config, spl06_coef_t *coef) {
    uint8_t reg;
    uint8_t buf[18];

    reg = SPL06_MEAS_CFG;
    i2c_write_blocking(config->i2c, config->addr, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr, &buf[0], 1, false);
    if (!(buf[0] & 0x80)) return PICO_ERROR_GENERIC;
#ifndef NDEBUG
    printf("spl06: coef ready!\n");
#endif


    reg = SPL06_COEF;
    i2c_write_blocking(config->i2c, config->addr, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr, buf, 18, false);

    // temp
    // c0: 12bit
    coef->c0  = (((uint16_t)buf[1] >> 4) & 0x0F)
              | ((uint16_t)buf[0] << 4)
              | ((buf[0] & 0x80) ? 0xF000 : 0);

    // c1: 12bit
    coef->c1  = (uint16_t)buf[2]
              | (((uint16_t)(buf[1] & 0x0F)) << 8)
              | ((buf[1] & 0x08) ? 0xF000 : 0);

    // c00: 20bit
    coef->c00  = (((uint32_t)buf[3]) << 12)
               | (((uint32_t)buf[4]) << 4)
               | (((uint32_t)buf[5] >> 4) & 0x0F)
               | ((buf[3] & 0x80) ? 0xFFF00000 : 0);

    // c10: 20bit
    coef->c10  = (((uint32_t)(buf[5] & 0x0F)) << 16)
               | (((uint32_t)buf[6]) << 8)
               | (uint32_t)buf[7]
               | ((buf[5] & 0x08) ? 0xFFF00000 : 0);

    // c01: 16bit
    coef->c01  = (((uint16_t)buf[8]) << 8)
               | (uint16_t)buf[9];

    // c11: 16bit
    coef->c11  = (((uint16_t)buf[10]) << 8)
               | (uint16_t)buf[11];

    // c20: 16bit
    coef->c20  = (((uint16_t)buf[12]) << 8)
               | (uint16_t)buf[13];

    // c21: 16bit
    coef->c21  = (((uint16_t)buf[14]) << 8)
               | (uint16_t)buf[15];

    // c30: 16bit
    coef->c30  = (((uint16_t)buf[16]) << 8)
               | (uint16_t)buf[17];

/*
    printf("coef buf(0 to 17):\n");
    for (int8_t i = 0; i < 18; i++) {
        printf("%x ", buf[i]);
    }
    printf("\n");
*/

#ifndef NDEBUG
    printf("coef:\nc0: %d\nc1: %d\nc00: %d\nc10: %d\nc01: %d\nc11: %d\nc20: %d\nc21: %d\nc30: %d\n", 
            coef->c0, coef->c1, coef->c00, coef->c10, coef->c01, coef->c11, coef->c20, coef->c21, coef->c30);
#endif
}

static int32_t oversample2k(uint8_t oversample) {
    int32_t k;
    switch (oversample)
    {
    case 1:
        k = 524288;
        break;
    case 2:
        k = 1572864;
        break;
    case 4:
        k = 3670016;
        break;
    case 8:
        k = 7864320;
        break;
    case 16:
        k = 253952;
        break;
    case 32:
        k = 516096;
        break;
    case 64:
        k = 1040384;
        break;
    case 128:
        k = 2088960;
        break;
    default:
#ifndef NDEBUG
    printf("spl06: invaild oversample 2k\n");
#endif
        break;
    }
    return k;
}

static void spl06_read_press_raw(spl06_config_t *config, int32_t *prs) {
    uint8_t reg;
    uint8_t buf[3];
    *prs = 0;

    reg = SPL06_PRS_B2;
    i2c_write_blocking(config->i2c, config->addr, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr, buf, 3, false);

    *prs = (int32_t)((uint32_t)buf[2]
         | (((uint32_t)buf[1]) << 8)
         | (((uint32_t)buf[0]) << 16)
         | ((buf[0] & 0x80) ? 0xFF000000 : 0));
//printf("prs buf: %x %x %x\n", buf[0], buf[1], buf[2]);
}

void spl06_read_press_cal(spl06_config_t *config, spl06_coef_t *coef, float *prs) {
    int32_t prs_raw, temp_raw;
    spl06_read_press_raw(config, &prs_raw);
    spl06_read_temp_raw(config, &temp_raw);

    float prs_sc = (float)prs_raw / (float)oversample2k(config->prs_config[1]); // scaled pressure
    float temp_sc = (float)temp_raw / (float)oversample2k(config->temp_config[1]);
//printf("prs_raw: %d, temp_raw: %d\n", prs_raw, temp_raw);
//printf("prs_sc: %d, temp_sc: %d\n", prs_sc, temp_sc);
    *prs = (float)(coef->c00) + prs_sc * ((float)(coef->c10) + prs_sc * ((float)(coef->c20) + prs_sc * (float)(coef->c30)));
//printf("prs cal1: %f\n", *prs);
    *prs += temp_sc * (float)(coef->c01) + temp_sc * prs_sc * (((float)(coef->c11) + prs_sc * (float)(coef->c21)));
    *prs /= 100.0f;
}

static void spl06_read_temp_raw(spl06_config_t *config, int32_t *temp) {
    uint8_t reg;
    uint8_t buf[3];
    *temp = 0;

    reg = SPL06_TMP_B2;
    i2c_write_blocking(config->i2c, config->addr, &reg, 1, true);
    i2c_read_blocking(config->i2c, config->addr, buf, 3, false);

    *temp = (int32_t)((uint32_t)buf[2]
          | (((uint32_t)buf[1]) << 8)
          | (((uint32_t)buf[0]) << 16)
          | ((buf[0] & 0x80) ? 0xFF000000 : 0));
//printf("temp buf: %x %x %x\n", buf[0], buf[1], buf[2]);
}

void spl06_read_temp_cal(spl06_config_t *config, spl06_coef_t *coef, float *temp) {
    int32_t temp_raw;
    spl06_read_temp_raw(config, &temp_raw);

    float temp_sc = (float)temp_raw / (float)oversample2k(config->temp_config[1]);
//printf("temp_raw: %d / 0x%x, kt: %d, temp_sc: %f\n", temp_raw, temp_raw, kt, temp_sc);
    *temp = (float)(coef->c0) * 0.5 + (float)(coef->c1) * temp_sc;
}

float spl06_calc_alt(float prs) {
    return (44330 * (1 - pow((prs / 1013.25), 0.190294957183634)));
}