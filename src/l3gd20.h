#define L3GD20_WRITE 0x0000
#define L3GD20_READ 0x8000

#define L3GD20_ADDR_INC_ON 0x4000
#define L3GD20_ADDR_INC_OFF 0x0000

#define L3GD20_WHO_AM_I 0x0F00 // default 11010100
#define L3GD20_CTRL_REG1 0x2000 // default 00000111
#define L3GD20_CTRL_REG2 0x2100 // default 00000000
#define L3GD20_CTRL_REG3 0x2200 // default 00000000
#define L3GD20_CTRL_REG4 0x2300 // default 00000000
#define L3GD20_CTRL_REG5 0x2400 // default 00000000
#define L3GD20_REFERENCE 0x2500 // default 00000000
#define L3GD20_OUT_TEMP 0x2600 // default output
#define L3GD20_STATUS_REG 0x2700 // default output
#define L3GD20_OUT_X_L 0x2800 // default output
#define L3GD20_OUT_X_H 0x2900 // default output
#define L3GD20_OUT_Y_L 0x2A00 // default output
#define L3GD20_OUT_Y_H 0x2B00 // default output
#define L3GD20_OUT_Z_L 0x2C00 // default output
#define L3GD20_OUT_Z_H 0x2D00 // default output
#define L3GD20_FIFO_CTRL_REG 0x2E00 // default 00000000
#define L3GD20_FIFO_SRC_REG 0x2F00 // default output
#define L3GD20_INT1_CFG 0x3000 // default 00000000
#define L3GD20_INT1_SRC 0x3100 // default output
#define L3GD20_INT1_TSH_XH 0x3200 // default 00000000
#define L3GD20_INT1_TSH_XL 0x3300 // default 00000000
#define L3GD20_INT1_TSH_YH 0x3400 // default 00000000
#define L3GD20_INT1_TSH_YL 0x3500 // default 00000000
#define L3GD20_INT1_TSH_ZH 0x3600 // default 00000000
#define L3GD20_INT1_TSH_ZL 0x3700 // default 00000000
#define L3GD20_INT1_DURATION 0x3800 // default 00000000
