/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux)
* proximity detection (prox), and color temperature functionality within the
* AMS-TAOS TMD3702 family of devices.
*/

#ifndef __TMD3702_H
#define __TMD3702_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/interrupt.h>

#include <linux/time.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
//#define LOG_TAG "TMD3702"
//?????
//nubia 2018-528 tmd3702 chip id
#define DEVICE_CHIP_NAME "pa224:tmd3702"
//#define TMD3702_DRV_VERSION "Tmd 3702 driver version 2018-5-22 10:30:42"
//#define TMD3702_APC_ENABLE


/* POWER SUPPLY VOLTAGE RANGE */
#define POWER_VDD_MIN_UV  3050000
#define POWER_VDD_MAX_UV  3350000
#define POWER_VIO_MIN_UV  1750000
#define POWER_VIO_MAX_UV  1950000

/*
enum {
    ERR_NAKED_CAL = 1,
    ERR_THRES_CAL,
    ERR_FILE_OPS,
    ERR_DEV_OPS,
    ERR_OTHER,
};

*/


/* Default Params */

#define COEF_SCALE_DEFAULT 1000
#define DGF_DEFAULT        1195
#define CLR_COEF_DEFAULT   (1120)
#define RED_COEF_DEFAULT   (-1330)
#define GRN_COEF_DEFAULT  (4470)
#define BLU_COEF_DEFAULT   (-3890)
#define CT_COEF_DEFAULT    (5302)
#define CT_OFFSET_DEFAULT  (1673)

#define PROX_MAX_THRSHLD   (0xff)		//0x3FFF
// fraction out of 10
#define TENTH_FRACTION_OF_VAL(v, x) ({ \
  int __frac = v; \
  if (((x) > 0) && ((x) < 10)) __frac = (__frac*(x)) / 10 ; \
  __frac; \
})
enum tmd3702_prox_distance {
    PROX_NONE              = 0,
    PROX_NEAR              = 3,
    PROX_FAR               = 10,
};
enum tmd3702_oil_state {
	OIL_NONE = 0,
	OIL_STATE = 1,
};
enum tmd3702_regs {
    TMD3702_REG_ENABLE       = 0x80,
    TMD3702_REG_ATIME        = 0x81,
    TMD3702_REG_PRATE        = 0x82,
    TMD3702_REG_WTIME        = 0x83,
    TMD3702_REG_AILTL        = 0x84,
    TMD3702_REG_AILTH        = 0x85,
    TMD3702_REG_AIHTL        = 0x86,
    TMD3702_REG_AIHTH        = 0x87,
    TMD3702_REG_PILTL        = 0x88,
    TMD3702_REG_PILTH        = 0x89,
    TMD3702_REG_PIHTL        = 0x8A,
    TMD3702_REG_PIHTH        = 0x8B,
    TMD3702_REG_PERS         = 0x8C,
    TMD3702_REG_CFG0         = 0x8D,
    TMD3702_REG_PGCFG0       = 0x8E,
    TMD3702_REG_PGCFG1       = 0x8F,
    TMD3702_REG_CFG1         = 0x90,
    TMD3702_REG_REVID        = 0x91,
    TMD3702_REG_ID           = 0x92,
    TMD3702_REG_STATUS       = 0x93,
    TMD3702_REG_CDATAL       = 0x94,
    TMD3702_REG_CDATAH       = 0x95,
    TMD3702_REG_RDATAL       = 0x96,
    TMD3702_REG_RDATAH       = 0x97,
    TMD3702_REG_GDATAL       = 0x98,
    TMD3702_REG_GDATAH       = 0x99,
    TMD3702_REG_BDATAL       = 0x9A,
    TMD3702_REG_BDATAH       = 0x9B,
    TMD3702_REG_PDATAL       = 0x9C,
    TMD3702_REG_PDATAH       = 0x9D,

    TMD3702_REG_REVID2       = 0x9E,
    TMD3702_REG_CFG2         = 0x9F,
    TMD3702_REG_SOFTRST      = 0xA0,
    TMD3702_REG_CFG3         = 0xAB,
    TMD3702_REG_CFG4         = 0xAC,
    TMD3702_REG_CFG5         = 0xAD,
    TMD3702_REG_CFG6         = 0xAE,

    TMD3702_REG_POFFSET_L    = 0xC0,
    TMD3702_REG_POFFSET_H    = 0xC1,

    TMD3702_REG_CALIB        = 0xD7,
    TMD3702_REG_CALIBCFG     = 0xD9,
    TMD3702_REG_CALIBSTAT    = 0xDC,
    TMD3702_REG_INTENAB      = 0xDD,

    TMD3702_REG_FAC          = 0xE6,
    TMD3702_REG_FAC_HI       = 0xE7,

    /* TODO remove me after new datasheet rev */
    TMD3702_REG_TEST3        = 0xF2,
};

#define PROX_PERS_1_CONSECUTIVE_PROX_VALUE 0x10 
#define PROX_PERS_2_CONSECUTIVE_PROX_VALUE 0xF0 

enum tmd3702__reg {
    TMD3702_MASK_INT_RD_CLR = 0x80,
    TMD3702_SHIFT_INT_RD_CLR = 7,

    TMD3702_MASK_SAI = 0x10,
    TMD3702_SHIFT_SAI = 4,

    TMD3702_MASK_APC = 0x40,
    TMD3702_SHIFT_APC = 6,

    TMD3702_MASK_PILTH  = 0x3F,
    TMD3702_SHIFT_PILTH = 0,

    TMD3702_MASK_PIHTH  = 0x3F,
    TMD3702_SHIFT_PIHTH = 0,

    TMD3702_MASK_START_OFFSET_CALIB = 0x01,
    TMD3702_SHIFT_START_OFFSET_CALIB = 0,

    TMD3702_MASK_ELEC_CALIB  = 0x20,
    TMD3702_SHIFT_ELEC_CALIB = 5,

    TMD3702_MASK_PROX_PERS = 0xf0,
    TMD3702_SHIFT_PROX_PERS = 4,

    TMD3702_MASK_PDRIVE = 0x1f,
    TMD3702_SHIFT_PDRIVE = 0,

    TMD3702_MASK_PGAIN = 0xC0,
    TMD3702_SHIFT_PGAIN = 6,

    TMD3702_MASK_IR_MUX  = 0x40,
    TMD3702_SHIFT_IR_MUX = 6,

    TMD3702_MASK_AGAIN = 0x1F,
    TMD3702_SHIFT_AGAIN = 0,

    TMD3702_MASK_APERS = 0x0f,
    TMD3702_SHIFT_APERS = 0,

    TMD3702_MASK_WLONG = 0x04,
    TMD3702_SHIFT_WLONG = 2,

    TMD3702_MASK_PPULSE_LEN_16  = 0x01,
    TMD3702_SHIFT_PPULSE_LEN_16 = 0,

    TMD3702_MASK_PPULSE_LEN  = 0xC0,
    TMD3702_SHIFT_PPULSE_LEN = 6,

    TMD3702_MASK_PPULSE  = 0x3F,
    TMD3702_SHIFT_PPULSE = 0,

    TMD3702_MASK_POFFSET_H = 0x01,
    TMD3702_SHIFT_POFFSET_H = 0,

    TMD3702_MASK_BINSRCH_TARGET = 0xE0,
    TMD3702_SHIFT_BINSRCH_TARGET = 5,

    TMD3702_MASK_PROX_AUTO_OFFSET_ADJUST = 0x08,
    TMD3702_SHIFT_PROX_AUTO_OFFSET_ADJUST = 3,

    TMD3702_MASK_PROX_DATA_AVG = 0x07,
    TMD3702_SHIFT_PROX_DATA_AVG = 0,

    TMD3702_MASK_CALIB_FINISH = 0x01,
    TMD3702_SHIFT_CALIB_FINISH = 0,
};

enum tmd3702_en_reg {
    TMD3702_PON  = (1 << 0),
    TMD3702_AEN  = (1 << 1),
    TMD3702_PEN  = (1 << 2),
    TMD3702_WEN  = (1 << 3),
    TMD3702_EN_ALL = (TMD3702_AEN |
                          TMD3702_PEN |
                          TMD3702_WEN),
};

enum tmd3702_status {
    TMD3702_ST_PGSAT_AMBIENT  = (1 << 0),
    TMD3702_ST_PGSAT_RELFLECT = (1 << 1),
    TMD3702_ST_ZERODET    = (1 << 1),
    TMD3702_ST_CAL_IRQ    = (1 << 3),
    TMD3702_ST_ALS_IRQ    = (1 << 4),
    TMD3702_ST_PRX_IRQ    = (1 << 5),
    TMD3702_ST_PRX_SAT    = (1 << 6),
    TMD3702_ST_ALS_SAT    = (1 << 7),
};

enum tmd3702_intenab_reg {
    TMD3702_ZIEN = (1 << 2),
    TMD3702_CIEN = (1 << 3),
    TMD3702_AIEN = (1 << 4),
    TMD3702_PIEN = (1 << 5),
    TMD3702_PSIEN = (1 << 6),
    TMD3702_ASIEN = (1 << 7),
};

#define MAX_REGS 256
struct device;
//enum tmd3702_pwr_state {
//	POWER_OFF       = (0 << 0),
//	POWER_PS_ON     = (1 << 0),
//	POWER_ALS_ON    = (2 << 0),
//	POWER_ON        = (3 << 0),
//};
enum tmd3702_pwr_state {
    POWER_ON= (3 << 0),
    POWER_OFF= (0 << 0),
    POWER_PS_ON     = (1 << 0),
    POWER_ALS_ON    = (2 << 0),
    POWER_STANDBY= (4 << 0),
};

enum tmd3702_prox_state {
    PROX_STATE_NONE = 0,
    PROX_STATE_INIT,
    PROX_STATE_CALIB,
    PROX_STATE_WAIT_AND_CALIB
};
//TMD3702_CFG0_REG @0x8D
#define PPULSE_LEN_16X                (0x01 << 0)
#define WLONG                         (0x01 << 2)
#define CFG0_RESERVED                 (0x08 << 3)

//TMD3702_CFG1_REG @0x90
#define AGAIN_MASK                    (0x1F << 0)
#define AGAIN_1X                      (0x01 << 0)
#define AGAIN_4X                      (0x03 << 0)
#define AGAIN_16X                     (0x05 << 0)
#define AGAIN_64X                     (0x07 << 0)
#define AGAIN_128X                    (0x08 << 0)
#define AGAIN_256X                    (0x09 << 0)
#define AGAIN_493X                    (0x0A << 0)
#define CFG1_RESERVED                 (0x01 << 5)
#define IR_MUX                        (0x01 << 6)

//#define CFG1_RESERVED                 (0x01 << 5)
//TMD3702_CFG6_REG @0xAE
#define CFG6_RESERVED                 (0x3F << 0)
#define APC_DISABLE                   (0x01 << 6)
enum tmd3702_ctrl_reg {
    AGAIN_1        = (1 << 0),
    AGAIN_4        = (3 << 0),
    AGAIN_16       = (5 << 0),
    AGAIN_64       = (7 << 0),
    AGAIN_128      = (8 << 0),
    AGAIN_256      = (9 << 0),
    AGAIN_512      = (10 << 0),
    PGAIN_1        = (0 << TMD3702_SHIFT_PGAIN),
    PGAIN_2        = (1 << TMD3702_SHIFT_PGAIN),
    PGAIN_4        = (2 << TMD3702_SHIFT_PGAIN),
    PGAIN_8        = (3 << TMD3702_SHIFT_PGAIN),
    PG_PULSE_4US   = (0 << 6),
    PG_PULSE_8US   = (1 << 6),
    PG_PULSE_16US  = (2 << 6),
    PG_PULSE_32US  = (3 << 6),
};

// pldrive
#define PDRIVE_MA(p) ({ \
        u8 __reg = (((u8)((p) - 2) / 2) & 0xf); \
        /* artf24717 limit PLDRIVE to 19mA */ \
        __reg = (__reg > 0x08) ? 0x08 : __reg; \
        __reg; \
})
#define P_TIME_US(p)   ((((p) / 88) - 1.0) + 0.5)
#define PRX_PERSIST(p) (((p) & 0xf) << 4)

#define INTEGRATION_CYCLE 2780
#define TMD3702_PROXMITY_TIME 3000
#define TMD3702_WAIT_TIME_MS 10
#define AW_TIME_MS(p)  ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define WAIT_TIME_MS(p)   ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

// lux
#define INDOOR_LUX_TRIGGER    6000
#define OUTDOOR_LUX_TRIGGER    10000
#define TMD3702_MAX_LUX        0xffff
#define TMD3702_MAX_ALS_VALUE    0xffff
#define TMD3702_MIN_ALS_VALUE    10

struct tmd3702_lux_segment {
	int64_t ch0_coef;
	int64_t ch1_coef;

	int64_t   c_coef;
	int64_t   r_coef;
	int64_t   g_coef;
	int64_t   b_coef;
};

struct tmd3702_als_parameters {
        long which_seg;
	long d_factor;
	long seg0_ch0_coef;
	long seg0_ch1_coef;
	long seg1_ch0_coef;
	long seg1_ch1_coef;

	long seg0_c_coef;
	long seg0_r_coef;
	long seg0_g_coef;
	long seg0_b_coef;

	long seg1_c_coef;
	long seg1_r_coef;
	long seg1_g_coef;
	long seg1_b_coef;
};
struct tp_als_parameter{
	long tp_module_id;
	struct tmd3702_als_parameters  tp_als_param;
	struct tmd3702_als_parameters  gold_lux_cal_parameter;
	struct tmd3702_als_parameters  white_lux_cal_parameter;
	struct tmd3702_als_parameters  black_lux_cal_parameter;
	struct tmd3702_als_parameters  blue_lux_cal_parameter;
};
struct tmd3702_parameters {
    /* Common */
	u8  persist;

	u8 prox_thres_near;
	u8 prox_thres_far;
	u8 prox_thres_oil_near;
	u8 prox_thres_oil_far;
	u16 prox_raw;
	u8 prox_oil_state;

    /* Prox */
	u16 prox_th_contaminated;
	u16 prox_th_verynear;
	u16 prox_th_min;
	u16 prox_th_max;
    bool prox_persist_update;
	u8  prox_apc;
	u8  prox_pulse_cnt;
	u8  prox_pulse_len;
	u8  prox_pulse_16x;
	u8  prox_gain;
	s16 poffset;
	u8  prox_drive;

	u8  wait_time;
	int64_t d_factor;
	u16 als_th_low;
	u16 als_th_high;
	u8  which_seg;
	u8  panel_id;
	u8  tp_color;

	u8 cfg3;
	struct tmd3702_lux_segment lux_segment[2];

	/* ALS / Color */
	u8  als_gain;
	u16 als_deltaP;
	u8  als_time;
	u32 dgf;
	u32 ct_coef;
	u32 ct_offset;
	u32 c_coef;
	u32 r_coef;
	u32 g_coef;
	u32 b_coef;
	u32 coef_scale;
};

struct tmd3702_als_info {
	u16 als_ch0; // photopic channel
	u16 als_ch1; // ir channel
	int64_t lux1_ch0_coef;
	int64_t lux1_ch1_coef;
	int64_t lux2_ch0_coef;
	int64_t lux2_ch1_coef;
//nubia add for am3702 code light 2018-05-23
	int64_t lux1_c_coef;
	int64_t lux1_r_coef;
	int64_t lux1_g_coef;
	int64_t lux1_b_coef;

	int64_t lux2_c_coef;
	int64_t lux2_r_coef;
	int64_t lux2_g_coef;
	int64_t lux2_b_coef;

	u16 cal_lux;
	u16 prev_lux;

	u32 cpl;
	u32 saturation;
	u16 clear_raw;
	u16 red_raw;
	u16 green_raw;
	u16 blue_raw;
	u16 cct;
	s16 ir;
	u16 lux;
};
/* For contamination issue */
#define	CONTAMINATED_COUNT            16
#define VERY_NEAR_COUNT               4
#define	DATA_JITTER_TH                	3
/* Proximity thresholds */
#define AMS_PROX_THRESH_NEAR          60  /* unit of ADC count */
#define AMS_PROX_THRESH_FAR           40  /* unit of ADC count */
#define AMS_PROX_THRESH_VERY_NEAR     200 /* unit of ADC count */
#define AMS_PROX_THRESH_CONTAMINATED  100 /* unit of ADC count */
#define AMS_PROX_MAX_VALUE            255

/* Max offset limitation, must < 30  && > -10 */
#define TMD3702_MAX_OFFSET            30
#define TMD3702_MIN_OFFSET            -10

struct tmd3702_prox_info {
	u16 raw;
	u8  prox_raw_debug;
	int last_detected;
	u8  very_near_count;
	u8  data_count;
	int detected;
	u8  data_buf[CONTAMINATED_COUNT];
};

struct light_parameter {
	int lux;
};
struct tmd3702_als_cal_data{
	struct light_parameter base;
	struct light_parameter cur;
	int flag;
};

struct tmd3702_chip {
	struct device *als_dev;
	struct device *ps_dev;
	struct workqueue_struct *ps_workqueue;
	struct work_struct  work_prox;
	struct work_struct  ps_work;
	struct work_struct  ps_irq_work;
	struct delayed_work als_work;
	struct mutex ps_lock;
	struct mutex als_lock;
	struct wakeup_source ps_wlock;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct mutex lock;
	struct i2c_client *client;
	struct gpio_desc *gpiod_interrupt;
	struct tmd3702_prox_info prx_inf;
	struct tmd3702_als_info als_inf;
	struct tmd3702_parameters params;
	struct tmd3702_i2c_platform_data *pdata;

	struct tmd3702_als_parameters als_params;
	struct tmd3702_als_cal_data als_cal_data;

	u8 shadow[MAX_REGS];

	struct input_dev *p_idev;
	struct input_dev *a_idev;
#ifdef ABI_SET_GET_REGISTERS
	struct input_dev *d_idev;
#endif // #ifdef ABI_SET_GET_REGISTERS
	int irq_gpio;
	int irq;
	bool irq_enabled;

	int in_suspend;
	int wake_irq;
	int irq_pending;
	int  als_poll_delay;
	int  alsps_poll_delay;

	bool  has_als;
	bool  has_ps;

	bool unpowered;
	bool als_enabled;
	bool als_gain_auto;
	bool prx_enabled;
	bool amsCalComplete;
	bool amsFirstProx;
	bool amsIndoorMode;

	bool prox_enabled;
	bool wakeup_from_suspend;
	atomic_t			state;
	enum tmd3702_prox_state prox_state;
	u8 device_index;
};

struct tmd3702_platfotm_ops {
	int (*platform_power)(struct tmd3702_chip *chip, bool on);
	int (*platform_init)(struct tmd3702_chip *chip);
	int (*platform_exit)(struct tmd3702_chip *chip);
};
// Must match definition in ../arch file
struct tmd3702_i2c_platform_data {
    /* The following callback for power events received and handled by
       the driver.  Currently only for SUSPEND and RESUME */
 //   int (*platform_power)(struct device *dev, enum tmd3702_pwr_state state);
//    int(*platform_power)(struct tmd3702_chip *chip, bool on);
//    int (*platform_init)(void);
//    void (*platform_teardown)(struct device *dev);

	struct  tmd3702_platfotm_ops  *ops;
	struct mutex lock;
	struct mutex i2c_lock;
	struct regulator *vdd;
	struct regulator *vio;
	u8    power_state;
	u8    power_always_on;
	u8    debug_level;
	u8    offset_first_event;
	u8    offset_data_count;
	u8 	  offset_l;
	u8    offset_h;
	u8   calib_cfg;
	bool  has_als;
	bool  has_ps;

	char const *prox_name;
	char const *als_name;
	struct tmd3702_parameters parameters;
	bool proximity_can_wake;
	bool als_can_wake;
#ifdef CONFIG_OF
	struct device_node  *of_node;
#endif

};

int sensor_create_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
void sensor_remove_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
int sensor_write_file(char *file_path, const char *write_buf, int count);
int sensor_read_file(char *file_path, char *read_buf ,int count);
int sensor_regulator_power_on(struct tmd3702_chip *chip, bool on);
int sensor_regulator_configure(struct tmd3702_chip *chip, bool on);
int sensor_hw_pinctrl_init(struct tmd3702_chip *chip, struct device *dev);
void sensor_irq_enable(struct tmd3702_chip *chip, bool enable, bool flag_sync);
void sensor_quick_sort(int *pdata, int len, int left, int right);
//int ams_i2c_modify_nubia(struct tmd3702_chip *chip, u8 *shadow, u8 reg, u8 mask, u8 val);
int tmd3702_flush_debugregs(struct tmd3702_chip *chip);
int tmd3702_read_regs(struct tmd3702_chip *chip);
int tmd3702_init(void);
void tmd3702_exit(void);
int sensor_set_debug_level(u8 data);

#endif /* __TMD3702_H */
