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
* proximity detection (prox) functionality within the
* AMS-TAOS TMD3702 family of devices.
*/

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>

//#include <linux/i2c/ams/ams_tmd3702.h>
//#include "ams_i2c.h"
#include "ams_tmd3702.h"
#include "ams_tmd3702_i2c.h"
#include "virtual_proximity.h"

#undef LOG_TAG
#define LOG_TAG "TMD3702-ALS"

//#define LUX_DBG
 int tmd3702_configure_als_mode(struct tmd3702_chip *chip, u8 state);
 int tmd3702_get_lux(struct tmd3702_chip *chip);
int tmd3702_read_als(struct tmd3702_chip *chip);
 void tmd3702_report_als(struct tmd3702_chip *chip);
 static void tmd3702_als_parameter_init(struct tmd3702_chip *chip);
 static int tmd3702_update_als(struct tmd3702_chip *chip);
#define DEV_ALS_NAME         "light"
#define DEV_ALS_NAME2         "light2"
#define ALS_CAL_PATH		 "/persist/sensors/als_cal_data"
#define ALS_CAL_PATH2		 "/persist/sensors/als_cal_data2"
#define COLOR_CONFIG_PATH2	 "/persist/sensors/rgb_color_cfg2"
#define PARSE_DTSI_NUMBER    7
#define MODULE_MANUFACTURE_NUMBER 3
#define VALID_FLAG							0x5555

#define GAIN1         1
#define GAIN4         3
#define GAIN16        5
#define GAIN64        7
#define GAIN128       8
#define GAIN256       9
#define GAIN512       10
#define ALS_NUM_CH    4
#define ALS_CH_SIZE   (sizeof(u8) * 2)


static dev_t tmd3702_als_dev_t;
static struct class *als_class;
#define SCALE_FACTOR(x, y) (x)/(y)


static u16 const als_gains[] = {
    0,    1,
    0,    4,
    0,    16,
    0,    64,
    128,  256,
    512,
};

static u8 const restorable_als_regs[] = {
    TMD3702_REG_ATIME,
    TMD3702_REG_WTIME,
    TMD3702_REG_PERS,
    TMD3702_REG_CFG0,
    TMD3702_REG_CFG1,
};
enum tp_color_id{
	GOLD = 0,
	WHITE,
	BLACK,
	BLUE,
	TP_COLOR_NUMBER,
};
static const char *dts_array_name[MODULE_MANUFACTURE_NUMBER] = {
	"tmd,tp0",
	"tmd,tp1",
	"tmd,tp2",
};
static struct tp_als_parameter tp_module_parameter[MODULE_MANUFACTURE_NUMBER] = {
	{.tp_module_id = 0x00},
	{.tp_module_id = 0x01},
	{.tp_module_id = 0x02}
};
static int tmd3702_flush_als_regs(struct tmd3702_chip *chip)
{
    unsigned i;
    int rc;
    u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_als_regs); i++) {
		reg = restorable_als_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow,
		reg, chip->shadow[reg]);
		if (rc) {
			//   dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",   __func__, reg);
			SENSOR_LOG_ERROR("err on reg 0x%02x\n", reg);
			break;
		}
	}

    return rc;
}

static void tmd3702_get_als(struct tmd3702_chip *chip)
{
    u8 *sh = chip->shadow;

    /* extract raw channel data */
    chip->als_inf.clear_raw =
      le16_to_cpup((const __le16 *)&sh[TMD3702_REG_CDATAL]);
    chip->als_inf.red_raw =
      le16_to_cpup((const __le16 *)&sh[TMD3702_REG_RDATAL]);
    chip->als_inf.green_raw =
      le16_to_cpup((const __le16 *)&sh[TMD3702_REG_GDATAL]);
    chip->als_inf.blue_raw =
      le16_to_cpup((const __le16 *)&sh[TMD3702_REG_BDATAL]);
    chip->als_inf.ir =
        (chip->als_inf.red_raw + chip->als_inf.green_raw +
        chip->als_inf.blue_raw - chip->als_inf.clear_raw) >> 1;
    if (chip->als_inf.ir < 0)
        chip->als_inf.ir = 0;

//	SENSOR_LOG_INFO("als_inf als_inf:%d,red_raw:%d,green_raw:%d,blue_raw:%d,ir:%d\n",
//		chip->als_inf.clear_raw, chip->als_inf.red_raw,chip->als_inf.green_raw ,chip->als_inf.blue_raw,chip->als_inf.ir );
	SENSOR_LOG_ERROR("als_inf.clear_raw = %d ,als_inf.red_raw = %d ,als_inf.green_raw = %d,als_inf.blue_raw = %d ir:%d\n",
		chip->als_inf.clear_raw, chip->als_inf.red_raw,
		chip->als_inf.green_raw,chip->als_inf.blue_raw,chip->als_inf.ir );
}

int tmd3702_read_als(struct tmd3702_chip *chip)
{
    int ret;

    ret = ams_i2c_blk_read(chip->client, TMD3702_REG_CDATAL,
            &chip->shadow[TMD3702_REG_CDATAL], ALS_NUM_CH * ALS_CH_SIZE);

    if (ret >= 0) {
        tmd3702_get_als(chip);
        ret = 0;
    }

    return ret;
}

static void tmd3702_calc_cpl(struct tmd3702_chip *chip)
{
    u32 cpl;
    u32 sat;
    u8 atime;

    atime = chip->shadow[TMD3702_REG_ATIME];

    cpl = atime;
    cpl *= INTEGRATION_CYCLE;
    cpl *= als_gains[(chip->shadow[TMD3702_REG_CFG1] & TMD3702_MASK_AGAIN)];
    //Optimization if coefficients are scaled by 1000,
    if (chip->params.coef_scale != 1000) {
      // Possible overflow if coefficients are scaled > 1000
      cpl *= chip->params.coef_scale; //rgb coeff scaling factor
      cpl /= (chip->params.dgf * 1000); //Atime usec -> msec
    } else {
      cpl /= chip->params.dgf;
    }

    sat = min_t(u32, TMD3702_MAX_ALS_VALUE, (u32) atime << 10);

    chip->als_inf.cpl = (u32) cpl;
    chip->als_inf.saturation = TENTH_FRACTION_OF_VAL(sat, 8);
    SENSOR_LOG_ERROR("TMD3702_atime:%d,Again:%d",chip->shadow[TMD3702_REG_ATIME],
       als_gains[(chip->shadow[TMD3702_REG_CFG1] & TMD3702_MASK_AGAIN)] );
}

int tmd3702_configure_als_mode(struct tmd3702_chip *chip, u8 state)
{

    u8 *sh = chip->shadow;

    if (state) // Turning on ALS
    {
        chip->shadow[TMD3702_REG_ATIME] = chip->params.als_time;
        tmd3702_calc_cpl(chip);

        /* set PERS.apers to 2 consecutive ALS values out of range */
        chip->shadow[TMD3702_REG_PERS] &= (~TMD3702_MASK_APERS);
        chip->shadow[TMD3702_REG_PERS] |= 0x02;

        tmd3702_flush_als_regs(chip);

        ams_i2c_modify(chip, sh, TMD3702_REG_INTENAB,
                TMD3702_AIEN, TMD3702_AIEN);
//       ams change code this 20180523
		if (chip->prox_enabled)
		    ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
			    TMD3702_WEN | TMD3702_AEN | TMD3702_PEN | TMD3702_PON,
			    TMD3702_AEN | TMD3702_PEN | TMD3702_PON);
		else
		    ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
			    TMD3702_WEN | TMD3702_AEN | TMD3702_PEN | TMD3702_PON,
			    TMD3702_AEN | TMD3702_PON);


/*
	//	ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
    //            TMD3702_WEN | TMD3702_AEN | TMD3702_PON,
    //            TMD3702_WEN | TMD3702_AEN | TMD3702_PON);
*/
        chip->als_enabled = true;
    }
    else  // Turning off ALS
    {
        // Disable ALS, Wait and ALS Interrupt
        ams_i2c_modify(chip, sh, TMD3702_REG_INTENAB,
                TMD3702_AIEN, 0);
//       ams change code this 20180523
		if (chip->prox_enabled)
		    ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
				TMD3702_WEN | TMD3702_AEN | TMD3702_PEN | TMD3702_PON,
				TMD3702_WEN | TMD3702_PEN | TMD3702_PON);
		else
		    ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
				TMD3702_WEN | TMD3702_AEN | TMD3702_PEN | TMD3702_PON,
				TMD3702_PON);
/*
        ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
                TMD3702_WEN | TMD3702_AEN, 0);
 */
        chip->als_enabled = false;

        // If nothing else is enabled set PON = 0;
        if(!(sh[TMD3702_REG_ENABLE] & TMD3702_EN_ALL))
            ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
            TMD3702_PON, 0);
    }

    return 0;
}
#ifdef nubia_code
#endif
/******************************
add function list
*****************************/
static int tmd3702_als_set_enable(struct tmd3702_chip *chip, u8 enable, bool enable_poll)
{
	int ret;
	if (enable) {
		if (!chip->pdata->power_always_on) {
			if (chip->pdata->power_state == POWER_OFF)
					sensor_regulator_power_on(chip, true);
			chip->pdata->power_state |= POWER_ALS_ON;
		}
		ret = tmd3702_configure_als_mode(chip, 1);
		if (ret < 0) {
			SENSOR_LOG_ERROR("als turn %s failed\n", enable?"on":"off");
			return ret;
		}
		if (enable_poll) {
			mutex_lock(&chip->als_lock);
			chip->als_enabled = 1;
			mutex_unlock(&chip->als_lock);
			schedule_delayed_work(&chip->als_work, msecs_to_jiffies(200));
		}
	}
	else {
		if (enable_poll) {
			cancel_delayed_work_sync(&chip->als_work);
			mutex_lock(&chip->als_lock);
			chip->als_enabled = 0;
			mutex_unlock(&chip->als_lock);
		}
		ret = tmd3702_configure_als_mode(chip, 0);
		if (ret < 0) {
			SENSOR_LOG_ERROR("als turn %s failed\n", enable?"on":"off");
			return ret;
		}

		if (!chip->pdata->power_always_on) {
			chip->pdata->power_state &= ~POWER_ALS_ON;
			if (!chip->prox_enabled && !chip->als_enabled && chip->pdata->power_state == POWER_OFF)
				sensor_regulator_power_on(chip, false);
		}
	}
	SENSOR_LOG_INFO("als turn %s\n", enable?"on":"off");
	return 0;
}
static int tmd3702_set_als_gain(struct tmd3702_chip *chip, int gain)
{
    int rc;
    u8 ctrl_reg;
    u8 saved_enable;

    switch (gain) {
    case 1:
        ctrl_reg = AGAIN_1;
        break;
    case 4:
        ctrl_reg = AGAIN_4;
        break;
    case 16:
        ctrl_reg = AGAIN_16;
        break;
    case 64:
        ctrl_reg = AGAIN_64;
        break;
    case 128:
        ctrl_reg = AGAIN_128;
        break;
    case 256:
        ctrl_reg = AGAIN_256;
        break;
    case 512:
        ctrl_reg = AGAIN_512;
        break;
    default:
    //    dev_err(&chip->client->dev, "%s: wrong als gain %d\n",  __func__, gain);
	SENSOR_LOG_ERROR("wrong als gain %d\n", gain);
        return -EINVAL;
    }

    // Turn off ALS, so that new ALS gain value will take effect at start of
    // new integration cycle.
    // New ALS gain value will then be used in next lux calculation.
    ams_i2c_read(chip->client, TMD3702_REG_ENABLE, &saved_enable);
    ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_ENABLE, 0);
    rc = ams_i2c_modify(chip, chip->shadow, TMD3702_REG_CFG1,
            TMD3702_MASK_AGAIN, ctrl_reg);
    ams_i2c_write(chip->client, chip->shadow,
                  TMD3702_REG_ENABLE, saved_enable);

    if (rc >= 0) {
        chip->params.als_gain =
          chip->shadow[TMD3702_REG_CFG1] & TMD3702_MASK_AGAIN;
    //    dev_info(&chip->client->dev, "%s: new als gain %d\n",
        //        __func__, ctrl_reg);
		SENSOR_LOG_INFO("new als gain %d\n", ctrl_reg);
    }

    return rc;
}

static void tmd3702_inc_gain(struct tmd3702_chip *chip)
{
    int rc;
    u8 gain = (chip->shadow[TMD3702_REG_CFG1] & TMD3702_MASK_AGAIN);
    s8 idx;

    if ((gain >= als_gains[(ARRAY_SIZE(als_gains) - 1)]) || (gain == 0))
        return;
    for (idx = 0; idx <= (ARRAY_SIZE(als_gains) - 1); idx++) {
        if ((als_gains[idx] == 0) || (idx <= gain)) continue;
        else if (idx > gain) {
            gain = idx;
            break;
        }
    }

    rc = tmd3702_set_als_gain(chip, als_gains[gain]);
    if (rc == 0)
        tmd3702_calc_cpl(chip);
}

static void tmd3702_dec_gain(struct tmd3702_chip *chip)
{
    int rc;
    u8 gain = (chip->shadow[TMD3702_REG_CFG1] & TMD3702_MASK_AGAIN);
    s8 idx;

    if ((gain <= als_gains[0]) || (gain == 0))
        return;
    for (idx = (ARRAY_SIZE(als_gains) - 1); idx >= 0; idx--) {
        if ((als_gains[idx] == 0) || (idx >= gain)) continue;
        else if (idx < gain) {
            gain = idx;
            break;
        }
    }

    rc = tmd3702_set_als_gain(chip, als_gains[gain]);
    if (rc == 0)
        tmd3702_calc_cpl(chip);
}

static int tmd3702_max_als_value(struct tmd3702_chip *chip)
{
    int val;

    val = chip->shadow[TMD3702_REG_ATIME];
    if (val > 63)
        val = 0xffff;
    else
        val = ((val * 1024) - 1);
    return val;
}

int tmd3702_get_lux(struct tmd3702_chip *chip)
{
	s32 rp1, gp1, bp1, cp1;
	s32 lux = 0;
	s32 cct = 0;
	s32 sf;

	int quintile ;
	quintile = (tmd3702_max_als_value(chip) / 5);
	//quintile = 200;
	//nubia add read als  2018-05-22
	tmd3702_read_als(chip);

    /* Auto gain moved to end of function */

    /* use time in ms get scaling factor */
	tmd3702_calc_cpl(chip);

	/* remove ir from counts*/
	rp1 = chip->als_inf.red_raw - chip->als_inf.ir;
	gp1 = chip->als_inf.green_raw - chip->als_inf.ir;
	bp1 = chip->als_inf.blue_raw - chip->als_inf.ir;
	cp1 = chip->als_inf.clear_raw - chip->als_inf.ir;

	if (!chip->als_inf.cpl) {

		SENSOR_LOG_INFO("zero cpl. Setting to 1\n");
		chip->als_inf.cpl = 1;
	}

	lux += chip->params.c_coef * chip->als_inf.clear_raw;
	lux += chip->params.r_coef * chip->als_inf.red_raw;
	lux += chip->params.g_coef * chip->als_inf.green_raw;
	lux += chip->params.b_coef * chip->als_inf.blue_raw;

	sf = chip->als_inf.cpl;

	lux /= sf;
	if (lux < 0) {

		SENSOR_LOG_INFO("lux < 0 use prev.\n");
		return chip->als_inf.lux; // use previous value
	}
	else {
		lux = min(TMD3702_MAX_LUX, max(0, lux));
	}

	SENSOR_LOG_INFO("get lux %d sf:%d\n",lux,sf);
	if (rp1 == 0)
	{
		rp1 = 1;
	}

	cct = ((chip->params.ct_coef * bp1) / rp1) + chip->params.ct_offset;

	SENSOR_LOG_INFO("get lux %d sf:%d cct:%d als_gain_auto:%d \n",lux,sf,cct,chip->als_gain_auto);

	if (!chip->als_gain_auto)
	{
		if (chip->als_inf.clear_raw <= TMD3702_MIN_ALS_VALUE)
		{
			lux = 0;
			SENSOR_LOG_INFO( "darkness (%d <= %d)\n", chip->als_inf.clear_raw,
				TMD3702_MIN_ALS_VALUE);

		}
		else if (chip->als_inf.clear_raw >= chip->als_inf.saturation) {
			lux = TMD3702_MAX_LUX;
			SENSOR_LOG_INFO( "saturation (%d >= %d\n",
				chip->als_inf.clear_raw, chip->als_inf.saturation);
		}
	}
	else
	{
		SENSOR_LOG_INFO("clear_raw:%d quintile:%d saturation:%d  \n",
			chip->als_inf.clear_raw,quintile,chip->als_inf.saturation);
		if (chip->als_inf.clear_raw < quintile) {
			//    dev_info(&chip->client->dev, "%s: AUTOGAIN INC\n", __func__);
			SENSOR_LOG_INFO("AUTOGAIN INC\n");
			tmd3702_inc_gain(chip);
			tmd3702_flush_als_regs(chip);
		}
		else if (chip->als_inf.clear_raw >= chip->als_inf.saturation) {
			//dev_info(&chip->client->dev, "%s: AUTOGAIN DEC\n", __func__);
			SENSOR_LOG_INFO("AUTOGAIN DEC\n");
			tmd3702_dec_gain(chip);
			tmd3702_flush_als_regs(chip);
		}
	}

	chip->als_inf.lux = lux;
	chip->als_inf.cct = (u16) cct;
//nubia add code prex_lux report input event 	2018-05-24
	chip->als_inf.prev_lux = lux;

    return 0;
}

int tmd3702_update_als_thres(struct tmd3702_chip *chip, bool on_enable)
{
    s32 ret;
    u16 deltaP = chip->params.als_deltaP;
    u16 from, to, cur;
    u16 saturation = chip->als_inf.saturation;

    cur = chip->als_inf.clear_raw;

    if (on_enable) {
        /* move deltaP far away from current position to force an irq */
        from = to = cur > (saturation / 2) ? 0 : saturation;
    } else {
        deltaP = cur * deltaP / 100;
        if (!deltaP)
            deltaP = 1;

        if (cur > deltaP)
            from = cur - deltaP;
        else
            from = 0;

        if (cur < (saturation - deltaP))
            to = cur + deltaP;
        else
            to = saturation;
    }
    *((__le16 *) &chip->shadow[TMD3702_REG_AILTL]) = cpu_to_le16(from);
    *((__le16 *) &chip->shadow[TMD3702_REG_AIHTL]) = cpu_to_le16(to);

 //   dev_info(&chip->client->dev,
   //          "%s: low:0x%x  hi:0x%x, oe:%d cur:%d deltaP:%d (%d) sat:%d\n",
     //        __func__, from, to, on_enable, cur, deltaP,
     //        chip->params.als_deltaP, saturation);

	SENSOR_LOG_INFO( " low:0x%x  hi:0x%x, oe:%d cur:%d deltaP:%d (%d) sat:%d\n",
              	    from, to, on_enable, cur, deltaP,   chip->params.als_deltaP, saturation);

    ret = ams_i2c_reg_blk_write(chip->client, TMD3702_REG_AILTL,
            &chip->shadow[TMD3702_REG_AILTL],
            (TMD3702_REG_AIHTH - TMD3702_REG_AILTL) + 1);

    return (ret < 0) ? ret : 0;
}

void tmd3702_report_als(struct tmd3702_chip *chip)
{
 //  int lux;
 //   int rc;

	if (chip->als_cal_data.flag) {
		//SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "cal_lux:%d\n", chip->als_inf.cal_lux + 1);
		SENSOR_LOG_INFO( "cal_lux:%d\n", chip->als_inf.cal_lux + 1);
		input_report_rel(chip->a_idev, REL_X, chip->als_inf.cal_lux + 1);
	} else {
		//SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "lux:%d\n", chip->als_inf.prev_lux + 1);
		SENSOR_LOG_INFO("lux:%d\n", chip->als_inf.prev_lux + 1);
		input_report_rel(chip->a_idev, REL_X, chip->als_inf.prev_lux + 1);
	}
	input_sync(chip->a_idev);

}
#ifdef nubia_code_list
#endif
static void tmd3702_als_report_work(struct work_struct *work)
{
	struct tmd3702_chip *chip = container_of((struct delayed_work *)work, struct tmd3702_chip, als_work);
	tmd3702_update_als(chip);
	tmd3702_report_als(chip);
	schedule_delayed_work(&chip->als_work, msecs_to_jiffies(chip->als_poll_delay));
}
 int tmd3702_update_als(struct tmd3702_chip *chip)
{
	int rc;
	rc = tmd3702_get_lux(chip);
	if (rc < 0) {
		return -EINVAL;
	}
	if (chip->als_cal_data.flag) {
		chip->als_inf.cal_lux = chip->als_inf.prev_lux  * SCALE_FACTOR(chip->als_cal_data.base.lux,
			chip->als_cal_data.cur.lux);
		if (chip->pdata->debug_level)
		    SENSOR_LOG_INFO("cal_lux:%d lux:%d [base:cur]=[%d:%d]\n",
					chip->als_inf.cal_lux + 1,chip->als_inf.prev_lux + 1,
					chip->als_cal_data.base.lux,chip->als_cal_data.cur.lux);
	}
	return 0;
}
 static int  tmd3702_config_tp_parameter(struct tmd3702_chip *chip)
{
	int i = 0;
	int err = -1;
	for (i = 0;i < MODULE_MANUFACTURE_NUMBER; i++)
	{
		if (chip->params.panel_id == tp_module_parameter[i].tp_module_id)
		{
			chip->params.d_factor= tp_module_parameter[i].tp_als_param.d_factor;
			chip->params.which_seg= tp_module_parameter[i].tp_als_param.which_seg;
			chip->params.lux_segment[0].c_coef = tp_module_parameter[i].tp_als_param.seg0_c_coef;
			chip->params.lux_segment[0].r_coef = tp_module_parameter[i].tp_als_param.seg0_r_coef;
			chip->params.lux_segment[0].g_coef = tp_module_parameter[i].tp_als_param.seg0_g_coef;
			chip->params.lux_segment[0].b_coef = tp_module_parameter[i].tp_als_param.seg0_b_coef;

			chip->params.lux_segment[1].c_coef = tp_module_parameter[i].tp_als_param.seg1_c_coef;
			chip->params.lux_segment[1].r_coef = tp_module_parameter[i].tp_als_param.seg1_r_coef;
			chip->params.lux_segment[1].g_coef = tp_module_parameter[i].tp_als_param.seg1_g_coef;
			chip->params.lux_segment[1].b_coef = tp_module_parameter[i].tp_als_param.seg1_b_coef;
			switch(chip->params.tp_color)
			{
				case GOLD :
					chip->params.lux_segment[0].c_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg0_c_coef;
					chip->params.lux_segment[0].r_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg0_r_coef;
					chip->params.lux_segment[0].g_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg0_g_coef;
					chip->params.lux_segment[0].b_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg0_b_coef;

					chip->params.lux_segment[1].c_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg1_c_coef;
					chip->params.lux_segment[1].r_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg1_r_coef;
					chip->params.lux_segment[1].g_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg1_g_coef;
					chip->params.lux_segment[1].b_coef = tp_module_parameter[i].gold_lux_cal_parameter.seg1_b_coef;
					break;
				case WHITE:
					chip->params.lux_segment[0].c_coef = tp_module_parameter[i].white_lux_cal_parameter.seg0_c_coef;
					chip->params.lux_segment[0].r_coef = tp_module_parameter[i].white_lux_cal_parameter.seg0_r_coef;
					chip->params.lux_segment[0].g_coef = tp_module_parameter[i].white_lux_cal_parameter.seg0_g_coef;
					chip->params.lux_segment[0].b_coef = tp_module_parameter[i].white_lux_cal_parameter.seg0_b_coef;

					chip->params.lux_segment[1].c_coef = tp_module_parameter[i].white_lux_cal_parameter.seg1_c_coef;
					chip->params.lux_segment[1].r_coef = tp_module_parameter[i].white_lux_cal_parameter.seg1_r_coef;
					chip->params.lux_segment[1].g_coef = tp_module_parameter[i].white_lux_cal_parameter.seg1_g_coef;
					chip->params.lux_segment[1].b_coef = tp_module_parameter[i].white_lux_cal_parameter.seg1_b_coef;
					break;
				case BLACK:
					chip->params.lux_segment[0].c_coef = tp_module_parameter[i].black_lux_cal_parameter.seg0_c_coef;
					chip->params.lux_segment[0].r_coef = tp_module_parameter[i].black_lux_cal_parameter.seg0_r_coef;
					chip->params.lux_segment[0].g_coef = tp_module_parameter[i].black_lux_cal_parameter.seg0_g_coef;
					chip->params.lux_segment[0].b_coef = tp_module_parameter[i].black_lux_cal_parameter.seg0_b_coef;

					chip->params.lux_segment[1].c_coef = tp_module_parameter[i].black_lux_cal_parameter.seg1_c_coef;
					chip->params.lux_segment[1].r_coef = tp_module_parameter[i].black_lux_cal_parameter.seg1_r_coef;
					chip->params.lux_segment[1].g_coef = tp_module_parameter[i].black_lux_cal_parameter.seg1_g_coef;
					chip->params.lux_segment[1].b_coef = tp_module_parameter[i].black_lux_cal_parameter.seg1_b_coef;
					break;
				case BLUE:
					chip->params.lux_segment[0].c_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg0_c_coef;
					chip->params.lux_segment[0].r_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg0_r_coef;
					chip->params.lux_segment[0].g_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg0_g_coef;
					chip->params.lux_segment[0].b_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg0_b_coef;

					chip->params.lux_segment[1].c_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg1_c_coef;
					chip->params.lux_segment[1].r_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg1_r_coef;
					chip->params.lux_segment[1].g_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg1_g_coef;
					chip->params.lux_segment[1].b_coef = tp_module_parameter[i].blue_lux_cal_parameter.seg1_b_coef;
					break;
				default:

					break;
			}
			  err = 0;
		}
	}
	return err;

}
static int tmd3702_als_calibrate_work(struct tmd3702_chip *chip, const char *cal_data)
{
	int err = 0;

	if (unlikely(IS_ERR_OR_NULL(cal_data))) {
		SENSOR_LOG_ERROR("NULL\n");
		return -1;
	}

	/*copy mem directly instead of parse string*/
	memcpy(&chip->als_cal_data.base, cal_data, sizeof(chip->als_cal_data.base));
	memcpy(&chip->als_cal_data.cur, cal_data, sizeof(chip->als_cal_data.cur));

	if (!chip->als_enabled) {
		err = tmd3702_als_set_enable(chip, 1, false);
		if (err < 0) {
			SENSOR_LOG_ERROR("enable failed.\n");
			goto als_cal_exit;
		}
	}
	msleep(2 * chip->als_poll_delay);
	err = tmd3702_get_lux(chip);
	if (err < 0) {
		SENSOR_LOG_ERROR("get mean lux value error\n");
		goto als_cal_exit;
	}
	chip->als_cal_data.cur.lux = chip->als_inf.prev_lux;
	chip->als_cal_data.cur.lux = (chip->als_cal_data.cur.lux > 0) ? chip->als_cal_data.cur.lux : 1;

	SENSOR_LOG_INFO("als_cal_data.base.lux = %d\n", chip->als_cal_data.base.lux);
	SENSOR_LOG_INFO("als_cal_data.cur.lux = %d\n", chip->als_cal_data.cur.lux);
	if (chip->als_cal_data.base.lux / chip->als_cal_data.cur.lux > 6) {
		err = -EINVAL;
		chip->als_cal_data.flag = 0;
		SENSOR_LOG_ERROR("calibrate param invalid; data->rgb_cal_data.flag=%d\n",
				chip->als_cal_data.flag);
		goto als_cal_exit;
	}

	chip->als_cal_data.flag = (chip->als_cal_data.base.lux > 0) ? 1 : 0;

	if (chip->als_cal_data.flag) {
		mutex_lock(&chip->als_lock);
		chip->als_inf.cal_lux= chip->als_inf.prev_lux * SCALE_FACTOR(chip->als_cal_data.base.lux,
			chip->als_cal_data.cur.lux);
		mutex_unlock(&chip->als_lock);
	} else {
		tmd3702_als_parameter_init(chip);
	}

	SENSOR_LOG_INFO("rgb_cal_data.flag = %d\n", chip->als_cal_data.flag);

	err = sensor_write_file(ALS_CAL_PATH2,
								(const char *)&(chip->als_cal_data),
								sizeof(struct tmd3702_als_cal_data));
	if (err < 0) {
		SENSOR_LOG_ERROR("save rgb cal parameters failed\n");
		goto als_cal_exit;
	}

als_cal_exit:
	err = tmd3702_als_set_enable(chip, chip->als_enabled, false);
	if (err < 0) {
		SENSOR_LOG_ERROR("disable failed.\n");
	}
	return err;

}
/*****************/
/* ABI Functions */
/*****************/
/*
static ssize_t tmd3702_device_als_lux(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    AMS_MUTEX_LOCK(&chip->lock);

    tmd3702_read_als(chip);
    tmd3702_get_lux(chip);

    AMS_MUTEX_UNLOCK(&chip->lock);

    return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}*/

static ssize_t tmd3702_lux_coef_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    int k;

    AMS_MUTEX_LOCK(&chip->lock);

    k =  snprintf(buf, PAGE_SIZE,
      "dgf:%d, ct_coef:%d, ct_offset:%d, c_coef:%d, r_coef:%d, g_coef:%d, \
      b_coef:%d\n",
      chip->params.dgf,
      chip->params.ct_coef,
      chip->params.ct_offset,
      chip->params.c_coef,
      chip->params.r_coef,
      chip->params.g_coef,
      chip->params.b_coef);

    AMS_MUTEX_UNLOCK(&chip->lock);

    return k;
}

static ssize_t tmd3702_lux_coef_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    u32 dgf, ct_coef, ct_offset, r_coef, g_coef, b_coef;

    if (6 != sscanf(buf, "%10d,%10d,%10d,%10d,%10d,%10d",
                    &dgf, &ct_coef, &ct_offset,
                    &r_coef, &g_coef, &b_coef));
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);

    chip->params.dgf       = dgf;
    chip->params.ct_coef   = ct_coef;
    chip->params.ct_offset = ct_offset;
    chip->params.r_coef    = r_coef;
    chip->params.g_coef    = g_coef;
    chip->params.b_coef    = b_coef;

    AMS_MUTEX_UNLOCK(&chip->lock);
    return size;
}

static ssize_t tmd3702_als_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmd3702_als_enable_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
        return -EINVAL;

 //   if (value)
 //       tmd3702_configure_als_mode(chip, 1);
 //  else
  //     tmd3702_configure_als_mode(chip, 0);

	if (value)
		tmd3702_als_set_enable(chip, 1, true);
	else
		tmd3702_als_set_enable(chip, 0, true);

    return size;
}

static ssize_t tmd3702_auto_gain_enable_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%s\n",
            chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmd3702_auto_gain_enable_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
        return -EINVAL;

    if (value)
        chip->als_gain_auto = true;
    else
        chip->als_gain_auto = false;

    return size;
}

static ssize_t tmd3702_als_gain_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
            als_gains[(chip->params.als_gain & TMD3702_MASK_AGAIN)],
            chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmd3702_als_gain_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    unsigned long gain;
    int i = 0;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 10, &gain);

    if (rc)
        return -EINVAL;
    if (gain != 0   && gain != 1   && gain != 4  &&
        gain != 16  && gain != 60  && gain != 64 &&
        gain != 128 && gain != 256 && gain != 512)
        return -EINVAL;

    while (i < ARRAY_SIZE(als_gains)) {
        if (gain == als_gains[i])
            break;
        i++;
    }

    if (i >= ARRAY_SIZE(als_gains)) {
        dev_err(&chip->client->dev, "%s: wrong als gain %d\n",   __func__, (int)gain);
	SENSOR_LOG_ERROR("wrong als gain %d\n",	(int)gain);
        return -EINVAL;
    }

    AMS_MUTEX_LOCK(&chip->lock);

    if (gain) {
        chip->als_gain_auto = false;
        rc = tmd3702_set_als_gain(chip, als_gains[i]);
        if (!rc)
            tmd3702_calc_cpl(chip);
    } else {
        chip->als_gain_auto = true;
    }
    tmd3702_flush_als_regs(chip);

    AMS_MUTEX_UNLOCK(&chip->lock);

    return rc ? -EIO : size;
}

static ssize_t tmd3702_als_red_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tmd3702_als_green_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tmd3702_als_cpl_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t tmd3702_als_blue_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tmd3702_als_clear_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
}

static ssize_t tmd3702_als_cct_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
/*	if (!chip->als_enabled) {
			SENSOR_LOG_INFO("disabled, need enable and waiting for updating data\n");
			tmd3702_als_set_enable(chip, 1, false);
			msleep(chip->als_poll_delay);
		ret = tmd3702_update_als(chip);
		if (ret < 0)
			return ret;
		tmd3702_als_set_enable(chip, chip->als_enabled, false);
	} else {
		SENSOR_LOG_INFO("enabled and just waiting for updating data\n");
		msleep(chip->als_poll_delay);
	}*/
	tmd3702_read_als(chip);
	tmd3702_get_lux(chip);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static ssize_t tmd3702_als_persist_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n",
            (((chip->shadow[TMD3702_REG_PERS]) & TMD3702_MASK_APERS)));
}

static ssize_t tmd3702_als_persist_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    long persist;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 10, &persist);
    if (rc)
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);
    chip->shadow[TMD3702_REG_PERS] &= ~TMD3702_MASK_APERS;
    chip->shadow[TMD3702_REG_PERS] |= ((u8)persist & TMD3702_MASK_APERS);

    tmd3702_flush_als_regs(chip);

    AMS_MUTEX_UNLOCK(&chip->lock);
    return size;
}

static ssize_t tmd3702_als_atime_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    int t;

    t = chip->shadow[TMD3702_REG_ATIME];
    t *= INTEGRATION_CYCLE;
    return snprintf(buf, PAGE_SIZE, "%dms (%dus)\n", t / 1000, t);
}

static ssize_t tmd3702_als_atime_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    long atime;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 10, &atime);
    if (rc)
        return -EINVAL;
    atime = AW_TIME_MS(atime); //Assume value in ms

    AMS_MUTEX_LOCK(&chip->lock);

    chip->shadow[TMD3702_REG_ATIME] = (u8) atime;
    chip->params.als_time = chip->shadow[TMD3702_REG_ATIME];
    tmd3702_calc_cpl(chip);
    tmd3702_flush_als_regs(chip);

    AMS_MUTEX_UNLOCK(&chip->lock);

    return size;
}

static ssize_t tmd3702_als_wtime_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int t;
    u8 wlongcurr;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    AMS_MUTEX_LOCK(&chip->lock);

    t = chip->shadow[TMD3702_REG_WTIME];

    wlongcurr = chip->shadow[TMD3702_REG_CFG0] & TMD3702_MASK_WLONG;
    if (wlongcurr)
        t *= 12;

    t *= INTEGRATION_CYCLE;
    t /= 1000;

    AMS_MUTEX_UNLOCK(&chip->lock);

    return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmd3702_als_wtime_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    unsigned long wtime;
    int wlong;
    int rc;

    rc = kstrtoul(buf, 10, &wtime);
    if (rc)
        return -EINVAL;

    wtime *= 1000;
    if (wtime > (256 * INTEGRATION_CYCLE))
    {
        wlong = 1;
        wtime /= 12;
    }
    else
        wlong = 0;
    wtime /= INTEGRATION_CYCLE;

    AMS_MUTEX_LOCK(&chip->lock);

    chip->shadow[TMD3702_REG_WTIME] = (u8) wtime;
    if (wlong)
        chip->shadow[TMD3702_REG_CFG0] |= TMD3702_MASK_WLONG;
    else
        chip->shadow[TMD3702_REG_CFG0] &= ~TMD3702_MASK_WLONG;

    tmd3702_flush_als_regs(chip);

    AMS_MUTEX_UNLOCK(&chip->lock);
    return size;
}


static ssize_t tmd3702_als_deltaP_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE,
            "%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tmd3702_als_deltaP_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    unsigned long deltaP;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 10, &deltaP);
    if (rc || deltaP > 100)
        return -EINVAL;
    AMS_MUTEX_LOCK(&chip->lock);
    chip->params.als_deltaP = deltaP;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return size;
}

#ifdef LUX_DBG
static ssize_t tmd3702_als_adc_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    tmd3702_get_lux(chip);

    return snprintf(buf, PAGE_SIZE, "lux: %d, cct: %d, clr: %d, red:%d, \
        grn:%d, blu:%d\n",
        chip->als_inf.lux,
        chip->als_inf.cct,
        chip->als_inf.clear_raw,
        chip->als_inf.red_raw,
        chip->als_inf.green_raw,
        chip->als_inf.blue_raw);
}

static ssize_t tmd3702_als_adc_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    u32 clr, red, grn, blu;

    if (4 != sscanf(buf, "%10d,%10d,%10d,%10d", &clr, &red, &grn, &blu))
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);

    chip->als_inf.clear_raw = clr;
    chip->als_inf.red_raw = red;
    chip->als_inf.green_raw = grn;
    chip->als_inf.blue_raw = blu;
    chip->als_inf.ir =
        (chip->als_inf.red_raw + chip->als_inf.green_raw +
        chip->als_inf.blue_raw - chip->als_inf.clear_raw + 1) >> 1;
    if (chip->als_inf.ir < 0)
        chip->als_inf.ir = 0;

    AMS_MUTEX_UNLOCK(&chip->lock);
    return size;
}
#endif // #ifdef LUX_DBG

static ssize_t tmd3702_rgb_config_tpinfo_show(struct device *dev,
                       struct device_attribute *attr, char *buf)
{
	int err;
	char cfg = 0;
	err = sensor_read_file(COLOR_CONFIG_PATH2, &cfg, sizeof(cfg));
	if (err < 0) {
		SENSOR_LOG_ERROR("read tpcolor parameters failed\n");
	}
	return sprintf(buf, "%x\n", cfg);
}
static ssize_t tmd3702_rgb_config_tpinfo_store(struct device *dev,
                 struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	int cfg = 0;
	int valid_flag = 0;
//	struct rgb_bh1745_data *data = dev_get_drvdata(dev);
//	struct rgb_bh1745_platform_data *pdata = data->platform_data;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);

	err = kstrtoint(buf, 0, &cfg);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}
	valid_flag = cfg & 0x80;
	if (!valid_flag) {
		SENSOR_LOG_ERROR("valid flag error\n");
		return -EINVAL;
	}
	//pdata->tp_color = cfg & 0x0f;
	//pdata->panel_id = (cfg>>4) & 0x07;
	chip->params.panel_id = (cfg>>4) & 0x07;
	chip->params.tp_color =cfg & 0x0f;
	SENSOR_LOG_INFO("panel_id =%d, tp_color =%d",chip->params.panel_id, chip->params.tp_color);

	if ((chip->params.tp_color < 0) && (chip->params.tp_color >= TP_COLOR_NUMBER)) {
		SENSOR_LOG_ERROR("TP_COLOR_NUMBER invalid\n");
		return -EINVAL;
	}
	//err = rgb_bh1745_config_tp_parameter(pdata);
	err = tmd3702_config_tp_parameter(chip);
	if (err < 0) {
		SENSOR_LOG_ERROR("init cofficient by defalut\n");
		return err;
	}
	err = sensor_write_file(COLOR_CONFIG_PATH2, (const char *)&cfg, sizeof(cfg));
	if (err < 0) {
		SENSOR_LOG_ERROR("save tpcolor parameters failed\n");
		return err;
	}
	return count;
}
static ssize_t tmd3702_read_tp_parameters(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return 1;

	//return snprintf(buf,2048,"test code ");
}

static ssize_t tmd3702_write_module_tpcolor(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{

	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	int err;
	u32 val;
	int valid_flag;
	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return count;
	}
	valid_flag = val & 0xffff;
	//chip->panel_id = (val >> 16) & 0xff;
	//pdata->tp_color = (val >> 24) & 0xff;

	chip->params.panel_id = (val >> 16) & 0xff;
	chip->params.tp_color = (val >> 24) & 0xff;

	if (valid_flag != VALID_FLAG){
		SENSOR_LOG_ERROR("valid flag error\n");
		return count;
	}
	SENSOR_LOG_INFO("panel_id = %d pdata->tp_color = %d\n",chip->params.panel_id, chip->params.tp_color );
	//err = rgb_bh1745_config_tp_parameter(pdata);
	err=tmd3702_config_tp_parameter(chip);
	if (err < 0) {
		SENSOR_LOG_ERROR("init cofficient by defalut\n");
	}

	SENSOR_LOG_INFO("lux cal  parameter from dtsi  is c[%lld] red[%lld], green[%lld], blue[%lld], c[%lld] , red[%lld], green[%lld], blue[%lld]\n",
		chip->params.lux_segment[0].c_coef, chip->params.lux_segment[0].r_coef,
		chip->params.lux_segment[0].g_coef,chip->params.lux_segment[0].b_coef,
		chip->params.lux_segment[1].c_coef, chip->params.lux_segment[1].r_coef,
		chip->params.lux_segment[1].g_coef,chip->params.lux_segment[1].b_coef);

	return count;
}

/*****************/
/* ABI Functions */
/*****************/
static ssize_t tmd3702_als_flush_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	int count;
	AMS_MUTEX_LOCK(&chip->als_lock);

	tmd3702_report_als(chip);
	if (chip->als_cal_data.flag) {
		count = sprintf(buf, "%d", chip->als_inf.cal_lux);
	} else {
		count = sprintf(buf, "%d",  chip->als_inf.prev_lux);
	}

	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return count;
}
static ssize_t tmd3702_als_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	int count, ret;

	if (!chip->als_enabled) {
		SENSOR_LOG_INFO("disabled, need enable and waiting for updating data\n");
		tmd3702_als_set_enable(chip, 1, false);
		msleep(chip->als_poll_delay);
		ret = tmd3702_update_als(chip);
		if (ret < 0)
			return ret;
		tmd3702_als_set_enable(chip, chip->als_enabled, false);
	} else {
		SENSOR_LOG_INFO("enabled and just waiting for updating data\n");
		msleep(chip->als_poll_delay);
	}
	if (chip->als_cal_data.flag) {
		count = sprintf(buf, "%d", chip->als_inf.cal_lux);
	} else {
		count = sprintf(buf, "%d",  chip->als_inf.prev_lux);
	}
	SENSOR_LOG_INFO("cal_lux = %d ,als_lux =%d\n",chip->als_inf.cal_lux, chip->als_inf.prev_lux);
	return count;
}

static ssize_t tmd3702_als_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->als_poll_delay);
}

static ssize_t tmd3702_als_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoint(buf, 0, &val);
	if (val < 500)
		val = 500;
	AMS_MUTEX_LOCK(&chip->als_lock);
	chip->als_poll_delay = val;
	AMS_MUTEX_UNLOCK(&chip->als_lock);

	return size;
}
static ssize_t tmd3702_als_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	chip->pdata->debug_level = !chip->pdata->debug_level;
	return sprintf(buf, "%d\n", chip->pdata->debug_level);
}

static ssize_t tmd3702_als_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	AMS_MUTEX_LOCK(&chip->als_lock);
	chip->pdata->debug_level = val;
	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return size;
}
static ssize_t tmd3702_als_calibrate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->als_cal_data.flag);
}

static ssize_t tmd3702_als_calibrate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	int val = 1;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("NULL.\n");
		return -EINVAL;
	}

	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}

	err = tmd3702_als_calibrate_work(chip, (const char *)&val);
	if (err < 0) {
		SENSOR_LOG_ERROR("als calibrate work failed.\n");
	}
	return size;
}
static ssize_t tmd3702_als_fac_calibrate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->als_cal_data.flag);
}

static ssize_t tmd3702_als_fac_calibrate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("NULL.\n");
		return -EINVAL;
	}

	err = tmd3702_als_calibrate_work(chip, buf);
	if (err < 0) {
		SENSOR_LOG_ERROR("als calibrate work failed.\n");
	}
	return size;
}

static ssize_t tmd3702_als_chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}

static ssize_t tmd3702_als_dev_init_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t tmd3702_als_dev_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	char cfg = 0;

	err = sensor_read_file(COLOR_CONFIG_PATH2,  &cfg,  sizeof(cfg));
	if (err < 0) {
		SENSOR_LOG_ERROR("read tpcolor parameters failed\n");
	}
	if (cfg & 0x80) {
		//pdata->tp_color = (cfg) & 0x0f;
		//pdata->panel_id = (cfg>>4) & 0x07;
		chip->params.panel_id =  (cfg) & 0x0f;
		chip->params.tp_color=(cfg>>4) & 0x07;
	} else {
		//pdata->tp_color = 0;
		//pdata->panel_id = 0;
		chip->params.panel_id = 0;
		chip->params.tp_color=0;
	}

	SENSOR_LOG_INFO("panel_id =%d, tp_color =%d",chip->params.panel_id, chip->params.tp_color);

	err=tmd3702_config_tp_parameter(chip);

	SENSOR_LOG_INFO("config seg paramter is 0 [c:%lld r:%lld g:%lld b:%lld]\n",
					chip->params.lux_segment[0].c_coef,
					chip->params.lux_segment[0].r_coef,
					chip->params.lux_segment[0].g_coef,
					chip->params.lux_segment[0].b_coef);
	SENSOR_LOG_INFO("config seg paramter is 1 [c:%lld r:%lld g:%lld b:%lld]\n",
					chip->params.lux_segment[1].c_coef,
					chip->params.lux_segment[1].r_coef,
					chip->params.lux_segment[1].g_coef,
					chip->params.lux_segment[1].b_coef);
	err = sensor_read_file(ALS_CAL_PATH2,
							(char *)&(chip->als_cal_data),
							sizeof(struct tmd3702_als_cal_data));
	if (err < 0) {
		SENSOR_LOG_ERROR("read factory cal parameters failed\n");
	}
	if (chip->als_cal_data.cur.lux == 0) {
		tmd3702_als_parameter_init(chip);
	}
	return size;
}
struct device_attribute tmd3702_als_attrs[] = {
	__ATTR(enable,        	  0644, tmd3702_als_enable_show,     tmd3702_als_enable_store),//ok
//    __ATTR(als_enable,        0664, tmd3702_als_enable_show,        tmd3702_als_enable_store),
	__ATTR(delay,         	  0644, tmd3702_als_delay_show,        tmd3702_als_delay_store), //ok
	__ATTR(debug,         	  0644, tmd3702_als_debug_show,       tmd3702_als_debug_store),//ok
	__ATTR(flush,   	          0444, tmd3702_als_flush_show, 	   NULL),//ok
	__ATTR(light_value,   	  0444, tmd3702_als_lux_show, 	   	   NULL),//ok
	//	__ATTR(als_lux,           0444, tmd3702_device_als_lux,        NULL),
	__ATTR(dev_init,      	  0664, tmd3702_als_dev_init_show,     tmd3702_als_dev_init_store),//ok
	__ATTR(chip_name,     	  0444, tmd3702_als_chip_id_show, 	   NULL), //ok
	__ATTR(calibrate,     	  0644, tmd3702_als_calibrate_show,    tmd3702_als_calibrate_store), //ok
	__ATTR(fac_calibrate, 	  0644, tmd3702_als_fac_calibrate_show,tmd3702_als_fac_calibrate_store),//ok
	//	__ATTR(als_ch0,           0444, tmd3702_als_red_show,          NULL),   //delete code
	//	__ATTR(als_ch1,           0444, tmd3702_als_green_show,          NULL),  //
	//nubia add rgb sensor
	__ATTR(module_tpcolor ,0644, tmd3702_read_tp_parameters, tmd3702_write_module_tpcolor),
	__ATTR(tp_cfg, 0644, tmd3702_rgb_config_tpinfo_show, tmd3702_rgb_config_tpinfo_store),
	//	__ATTR(als_Itime,     	  0644, tmd3702_als_itime_show,        tmd3702_als_itime_store),//ok   als_atime
	__ATTR(als_atime,         0664, tmd3702_als_atime_show,        tmd3702_als_atime_store),
	__ATTR(als_wtime,         0664, tmd3702_als_wtime_show,        tmd3702_als_wtime_store),
// delet code
	__ATTR(als_gain,          0664, tmd3702_als_gain_show,        tmd3702_als_gain_store), //ok
	__ATTR(als_cpl,           0444, tmd3702_als_cpl_show,        NULL), //ok
	__ATTR(als_cct,           0444, tmd3702_als_cct_show,        NULL),//ams new add code
	__ATTR(als_clr,           0444, tmd3702_als_clear_show,        NULL),//ams new add code
	__ATTR(als_red,           0444, tmd3702_als_red_show,        NULL),//ams new add code
	__ATTR(als_grn,           0444, tmd3702_als_green_show,        NULL),//ams new add code
	__ATTR(als_blu,           0444, tmd3702_als_blue_show,        NULL),//ams new add code
	__ATTR(als_thresh_deltaP, 0664, tmd3702_als_deltaP_show,        tmd3702_als_deltaP_store),//ams new add code
	__ATTR(als_auto_gain,     0664, tmd3702_auto_gain_enable_show,        tmd3702_auto_gain_enable_store),//ams new add code
	__ATTR(als_lux_coef,      0664, tmd3702_lux_coef_show,        tmd3702_lux_coef_store),//new code
	__ATTR(als_lux_table,     0644, tmd3702_lux_coef_show,        tmd3702_lux_coef_store),
	__ATTR(als_persist,       0664, tmd3702_als_persist_show,        tmd3702_als_persist_store),
#ifdef LUX_DBG
	__ATTR(als_adc,           0664, tmd3702_als_adc_show,       tmd3702_als_adc_store),
#endif // #ifdef LUX_DBG

};

int tmd3702_als_attrs_size = ARRAY_SIZE(tmd3702_als_attrs);

static int tmd3702_als_parse_dt(struct device *dev) {

	struct device_node *np = dev->of_node;

	unsigned int tp_moudle_count = 0;
	int rc = 0, i, array_len, retval;
	long *ptr;
	int index;
	const char *raw_data0_dts = NULL;

	/* ps tuning data*/
	rc = of_property_read_u32(np, "tmd,tp_moudle_count", &tp_moudle_count);
	SENSOR_LOG_INFO("tp_module_count is %d\n", tp_moudle_count);

	for (i=0; i < tp_moudle_count; i++)
	{
		array_len = of_property_count_strings(np, dts_array_name[i]);
		if (array_len != PARSE_DTSI_NUMBER) {
			SENSOR_LOG_ERROR("tmd2725,length invaild or dts number is larger than:%d\n",array_len);
			return -array_len;
		}
		SENSOR_LOG_INFO("read lux cal parameter count from dtsi  is %d\n", array_len);

		ptr = (long *)&tp_module_parameter[i];

		for(index = 0; index < array_len; index++){
			retval = of_property_read_string_index(np, dts_array_name[i], index, &raw_data0_dts);
			if (retval) {
				SENSOR_LOG_ERROR("read index = %d,raw_data0_dts = %s,retval = %d error,\n",index, raw_data0_dts, retval);
				return -retval;
			}
			ptr[index] = simple_strtoul(raw_data0_dts, NULL, 10);
			SENSOR_LOG_DEBUG("lux cal parameter from dtsi  is %ld\n",ptr[index]);
		}
	}

	return 0;
}
static void tmd3702_als_parameter_init(struct tmd3702_chip *chip)
{
	chip->als_cal_data.base.lux = 1;
	chip->als_cal_data.cur.lux = 1;
	chip->als_cal_data.flag = 0;
}
static int tmd3702_als_input_device_init(struct tmd3702_chip *chip)
{
	int ret;
	if (IS_ERR_OR_NULL(chip))
		return -ENODEV;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		SENSOR_LOG_ERROR("no memory for input_dev '%s'\n",
				chip->pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}

	chip->a_idev->name = chip->pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_REL, chip->a_idev->evbit);
	set_bit(REL_X, chip->a_idev->relbit);

	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		SENSOR_LOG_ERROR("cant register input '%s'\n",
				chip->pdata->als_name);
		ret = -ENODEV;
		goto input_a_register_failed;
	}
	return 0;
input_a_register_failed:
	input_free_device(chip->a_idev);
input_a_alloc_failed:
	return ret;
}


int tmd3702_als_device_register(struct tmd3702_chip *chip, struct i2c_driver *driver)
{
	int ret;
	mutex_init(&chip->als_lock);
	INIT_DELAYED_WORK(&chip->als_work, tmd3702_als_report_work);
	chip->als_poll_delay = 1000;

	tmd3702_als_parameter_init(chip);

	ret = tmd3702_als_parse_dt(&chip->client->dev);
	if (ret < 0) {
		SENSOR_LOG_ERROR("parse dts error\n");
		goto exit;
	}
	ret = tmd3702_als_input_device_init(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("intput device init fail\n");
		goto exit;
	}

	als_class = class_create(THIS_MODULE, DEV_ALS_NAME);
	alloc_chrdev_region(&tmd3702_als_dev_t, 0, 1, DEV_ALS_NAME);
	chip->als_dev = device_create(als_class, 0, tmd3702_als_dev_t, driver, DEV_ALS_NAME);
	if (IS_ERR_OR_NULL(chip->als_dev)) {
		SENSOR_LOG_ERROR("als device create fail\n");
		ret = -PTR_ERR(chip->als_dev);
		goto exit_input_dev;
	}

	ret = sensor_create_sysfs_interfaces(chip->als_dev, tmd3702_als_attrs, ARRAY_SIZE(tmd3702_als_attrs));
	if (ret < 0) {
		goto exit_remove_device;
	}
	dev_set_drvdata(chip->als_dev, chip);
	return 0;
exit_remove_device:
	device_destroy(als_class, tmd3702_als_dev_t);
	class_destroy(als_class);
exit_input_dev:
	if (chip->a_idev) {
		input_unregister_device(chip->a_idev);
		input_free_device(chip->a_idev);
	}
exit:
	mutex_destroy(&chip->als_lock);
	return ret;
}
void tmd3702_als_device_unregister(struct tmd3702_chip *chip)
{
	input_unregister_device(chip->a_idev);
	input_free_device(chip->a_idev);
	sensor_remove_sysfs_interfaces(chip->als_dev, tmd3702_als_attrs, ARRAY_SIZE(tmd3702_als_attrs));
	mutex_destroy(&chip->als_lock);
}

