#pragma once

// Pressure Value
// 24bit(3bytes) 2's complement value
#define SPL06_PRS_B2 0x00 // MSB
#define SPL06_PRS_B1 0x01 // LSB
#define SPL06_PRS_B0 0x02 // XLSB

// Pressure Config
// 7   X
// 6:4 PM_RATE(2bytes)
// 3:0 PM_PRC (3bytes)
#define SPL06_PRS_CFG 0x06 

// Temperature Value
// 24bit(3bytes) 2's complement value
#define SPL06_TMP_B2 0x03 // MSB
#define SPL06_TMP_B1 0x04 // LSB
#define SPL06_TMP_B0 0x05 // XLSB

// Temperature Config
// 7   TMP_EXT(1byte)
// 6:4 TMP_RATE(2bytes)
// 3   X
// 2:0 TMP_PRC(3bytes)
#define SPL06_TMP_CFG 0x07

// Sensor Operating Mode and Status
// 7   COEF_RDY
// 6   SENSOR_RDY
// 5   TMP_RDY
// 4   PRS_RDY
// 3   X
// 2:0 MEAS_CTRL
#define SPL06_MEAS_CFG 0x08

// Interrupt and FIFO configuration
// 7   INT_HL
// 6   INT_FIFO
// 5   INT_PRS
// 4   INT_TMP
// 3   T_SHIFT
// 2   P_SHIFT
// 1   FIFO_EN
// 0   X
#define SPL06_CFG_REG 0x09

// Interrupt Status
// 7:3 X
// 2   INT_FIFO_FULL
// 1   INT_TMP
// 0   INT_PRS
#define SPL06_INT_STS 0x0A

// FIFO Status
// 7:2 X
// 1   FIFO_FULL
// 0   FIFO_EMPTY
#define SPL06_FIFO_STS 0x0B

// Soft Reset and FIFO flush
// 7   FIFO_FLUSH
// 6:4 X
// 3:0 SOFT_RST
#define SPL06_RESET 0x0C

// Product and Revision ID
// 7:4 PROD_ID
// 3:0 REV_ID
#define SPL06_ID 0x0D

// Calibration Coefficients
// 0x10 7:0 c0[11:4] 12bits
// 0x11 7:4 c0[3:0]
// 0x11 3:0 c1[11:8] 12bits
// 0x12 7:0 c1[7:0]
// 0x13 7:0 c00[19:12] 20bits
// 0x14 7:0 c00[11:4]
// 0x15 7:4 c00[3:0]
// 0x15 3:1 c10[19:16] 20bits
// 0x16 7:0 c10[15:8]
// 0x17 7:0 c10[7:0]
// 0x18 7:0 c01[15:8] 16bits
// 0x19 7:0 c01[7:0]
// 0x1A 7:0 c11[15:8] 16bits
// 0x1B 7:0 c11[7:0]
// 0x1C 7:0 c20[15:8] 16bits
// 0x1D 7:0 c20[7:0]
// 0x1E 7:0 c21[15:8] 16bits
// 0x1F 7:0 c21[7:0]
// 0x20 7:0 c30[15:8] 16bits
// 0x21 7:0 c30[7:0]
#define SPL06_COEF 0x10

