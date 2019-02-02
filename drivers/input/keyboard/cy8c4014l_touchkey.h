#ifndef _CYPRESS_KEYS_H
#define _CYPRESS_KEYS_H

/* CYPRESS4000 Touchkey */
#define CYPRESS4000_TOUCHKEY_KEYVAL		0x00  //key value reg
#define CYPRESS4000_TOUCHKEY_FW             0x04   //firmware ver reg

/* Cy8c4044lqi-421 input key code */
#define CYPRESS_KEY_LEFT                    KEY_GAMEFIRE_LEFT
#define CYPRESS_KEY_RIGHT                   KEY_GAMEFIRE_RIGHT
#define CYPRESS_LEFT_BIT                    0x2
#define CYPRESS_RIGHT_BIT                   0x1


struct cypress_platform_data{
    struct input_dev *input_dev;
    int irq_gpio;
    int irq_flag;
    int rst_gpio;
};
struct cypress_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct cypress_platform_data	 *platform_data;
//	struct mutex		mutex;
    struct delayed_work cypress_work;
	int irq;
};

#endif
