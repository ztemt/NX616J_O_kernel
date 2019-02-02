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
#include <linux/delay.h>

//#include <ams_tmd3702.h>
//#include "ams_i2c.h"
#include "ams_tmd3702.h"
#include "ams_tmd3702_i2c.h"
#include "ams_tmd3702_als.h"
#include "ams_tmd3702_prox.h"
#include "virtual_proximity.h"

//#define PRX_DBG

#define FEATURE_NUBIA_SENSOR_INFRARED
// local function prototypes
void tmd3702_init_prox_mode(struct tmd3702_chip *chip,bool first);
int tmd3702_offset_calibration(struct tmd3702_chip *chip);
static int tmd3702_irq_handler_nubia(struct tmd3702_chip *chip);

static dev_t tmd3702_ps_dev_t;
static struct class *ps_class;

void tmd3702_do_prox_state(struct tmd3702_chip *chip)
{


	switch (chip->prox_state) {
		case PROX_STATE_INIT:
			tmd3702_init_prox_mode(chip,false);
			break;
		case PROX_STATE_CALIB:
			SENSOR_LOG_ERROR("test code chip->prox_state:%d \n",chip->prox_state);
			tmd3702_offset_calibration(chip);
			break;
		case PROX_STATE_WAIT_AND_CALIB:
			// TODO: change trigger from wait to looking for a certain
			// number of readings that are stable (delta <5 counts)
			msleep(100);

			tmd3702_offset_calibration(chip);
			tmd3702_init_prox_mode(chip,false);
			break;
		default:
			SENSOR_LOG_ERROR("test code %d \n",chip->prox_state);
		break;
	}

	SENSOR_LOG_IF(chip->pdata->debug_level,"test code %d \n",
        chip->prox_state);

	chip->prox_state = PROX_STATE_NONE;
}

void tmd3702_prox_thread(struct work_struct *work)
{
//    struct tmd3702_chip *chip
  //      = container_of(work, struct tmd3702_chip, work_prox);
	struct tmd3702_chip *chip = container_of(work, struct tmd3702_chip, ps_work);

//	SENSOR_LOG_IF(chip->pdata->debug_level,"test code %d \n",
//        chip->prox_state);
	tmd3702_do_prox_state(chip);
}
static void tmd3702_prox_irq_work(struct work_struct *work)
{
	struct tmd3702_chip *chip
		= container_of(work, struct tmd3702_chip, ps_irq_work);
	tmd3702_irq_handler_nubia(chip);
}
void tmd3702_schedule_prox_work(struct tmd3702_chip *chip,
                                enum tmd3702_prox_state prox_state)
{
	chip->prox_state = prox_state;

	schedule_work(&chip->ps_work);

	return;
}

void tmd3702_do_prox_work(struct tmd3702_chip *chip,
                          enum tmd3702_prox_state prox_state)
{
    chip->prox_state = prox_state;
    tmd3702_do_prox_state(chip);
}
static void tmd3702_do_prox_offset_cal(struct tmd3702_chip *chip, enum tmd3702_prox_state prox_state)
{
	chip->prox_state = prox_state;
	tmd3702_offset_calibration(chip);
	chip->prox_state = PROX_STATE_NONE;
}
/**************************/
/* General Prox Functions */
/**************************/

void tmd3702_read_prox(struct tmd3702_chip *chip)
{

#ifdef TMD3702_APC_ENABLE
        ams_i2c_blk_read(chip->client, TMD3702_REG_PDATAL,
                   &chip->shadow[TMD3702_REG_PDATAL], sizeof(chip->prx_inf.raw));
        chip->prx_inf.raw = le16_to_cpu(*((const __le16 *)&chip->shadow[TMD3702_REG_PDATAL]));
#else
   ams_i2c_blk_read(chip->client, TMD3702_REG_PDATAL,
            &chip->shadow[TMD3702_REG_PDATAL],1);
    chip->prx_inf.raw = chip->shadow[TMD3702_REG_PDATAL];
#endif
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "chip->prx_inf.raw = %d\n",chip->prx_inf.raw);

}
 void tmd3702_insert_sort(u8 data[], u8 count)
{
	int i, j;
	u8  temp;
	for (i = 1; i < count; i++)
	{
		temp = data[i];
		j = i-1;

		while(data[j] > temp && j >= 0)
		{
			data[j+1] = data[j];
			j--;
		}

		if (j != (i-1))
		{
			data[j+1] = temp;
		}
	}
}
void tmd3702_get_prox_ams2(struct tmd3702_chip *chip)
{
	  u8 *sh = chip->shadow;
	//struct i2c_client *client = chip->client;
	bool update_thresholds = false;
//	bool update_offset = false;
//	bool update_release_offset=false;
	u8 prox_th_low = 0, prox_th_high = 0;
	u8 pdata[CONTAMINATED_COUNT];
	u8  i, delta;
    u8 persist_l;
	bool ret = false;
	bool do_calibration = false;
    bool update_persist=false;

	prox_th_low = chip->params.prox_th_min;
	prox_th_high	=chip->params.prox_th_max;
	chip->params.prox_th_contaminated = chip->params.prox_thres_oil_far;
	chip->params.prox_th_verynear = chip->params.prox_thres_oil_near;

	if (chip->amsFirstProx)
	{
		/* First triggered proximity event */
		chip->amsFirstProx = false;
		// prox >80 (if (state->chip.prox_inf.raw > state->chip.params.prox_th_high)
		SENSOR_LOG_INFO("prx_inf.raw :%d prox_thres_near :%d\n",chip->prx_inf.raw,chip->params.prox_thres_near );
		if (chip->prx_inf.raw > chip->params.prox_thres_near)
		{
			/* Near by */
			chip->prx_inf.detected = true;
			chip->prx_inf.detected = PROX_NEAR;
			update_thresholds=true;
		    	tmd3702_report_prox(chip);
			//prox_th_low = chip->params.prox_th_low;
			//prox_th_high = chip->params.prox_th_verynear;
			//set threshould [50,250]
		//	prox_th_low = state->chip.params.prox_th_low;
		// nubia fix this bug 2018-05-26 prox th low
			prox_th_low = chip->params.prox_thres_far;
			prox_th_high = chip->params.prox_thres_oil_near;
			SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
		}
		else
		{
			/* Far away */
			chip->prx_inf.detected = false;
			chip->prx_inf.detected = PROX_FAR;
			 update_thresholds=true;
		    	tmd3702_report_prox(chip);
			//	//set far  threshould [1,80]
			prox_th_low = 1;

			// nubia fix this bug 2018-05-26 prox th high
			prox_th_high = chip->params.prox_thres_near;
			SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);

		}
		//do_calibration=true;
		//tmd3702_i2c_modify(state, TMD3702_PERS_REG, PPERS_MASK, PROX_PERSIST(1));
		//   ams_i2c_modify(chip,sh, TMD3702_REG_PERS, TMD3702_MASK_PROX_PERS,
		//                chip->params.persist & TMD3702_MASK_PROX_PERS);
		  ret = true;
	}
	else
	{
		SENSOR_LOG_INFO("prx_inf.raw :%d detected:%d \n",chip->prx_inf.raw,chip->prx_inf.detected );
		if (chip->prx_inf.detected==PROX_NEAR)
		{
			/* Near by state */
			SENSOR_LOG_INFO("prx_inf.raw :%d prox_th_low:%d \n",chip->prx_inf.raw,prox_th_low);
			if (chip->prx_inf.raw < prox_th_low)
			{
				/* Will change to Far away state */
				SENSOR_LOG_INFO("chip->prx_inf.raw :%d prox_thres_far:%d \n",chip->prx_inf.raw ,chip->params.prox_thres_far);
				if (chip->prx_inf.raw < chip->params.prox_thres_far)
				{
					chip->prx_inf.detected = false;
					//nubia add code
					chip->prx_inf.detected = PROX_FAR;
					update_thresholds=true;
		    			tmd3702_report_prox(chip);

					chip->prx_inf.data_count = 0;
					chip->prx_inf.very_near_count = 0;
					ret = true;
					////set far  threshould [1,80]
					prox_th_low = 1;
					prox_th_high = chip->params.prox_thres_near;
					SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
				}
				else
				{
					/* SNS_DD_PROX_THRESH_FAR < Raw data < SNS_DD_PROX_THRESH_CONTAMINATED,
						might have contamination issue, set same high and low threshold to collect raw data */
					//chip->params.prox_th_contaminated = chip->params.prox_thres_oil_far;
					prox_th_low = (chip->prx_inf.raw + 10) > chip->params.prox_th_contaminated ?
									chip->params.prox_th_contaminated : (chip->prx_inf.raw + 10);
					prox_th_high = prox_th_low;
					SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d data_count:%d \n",prox_th_low,prox_th_high,chip->prx_inf.data_count);
					chip->prx_inf.data_buf[chip->prx_inf.data_count%CONTAMINATED_COUNT] = chip->prx_inf.raw;
					if (++chip->prx_inf.data_count >= CONTAMINATED_COUNT)
					{
                        update_persist=true;
						/* Ring buffer is full, copy raw data to temp buffer */
						if (chip->prx_inf.data_count == (CONTAMINATED_COUNT*2))
						{
							chip->prx_inf.data_count = CONTAMINATED_COUNT;
						}
						for (i = 0; i < CONTAMINATED_COUNT; i++)
						{
							pdata[i] = chip->prx_inf.data_buf[i];
						}
						/* Do data sorting */
						tmd3702_insert_sort(pdata, CONTAMINATED_COUNT);

						/* Get delta, and judge whether it meet contamination issue or not */
						delta = pdata[CONTAMINATED_COUNT-3] - pdata[2];
						SENSOR_LOG_INFO("delta:%d\n",delta);
						if (delta < DATA_JITTER_TH)
						{
							/* Need to do offset calibration, reset ring buffer and flags */
							chip->prx_inf.detected = false;

							chip->prx_inf.detected = PROX_FAR;
							chip->prx_inf.data_count = 0;
							chip->prx_inf.very_near_count = 0;
							ret = true;
							tmd3702_report_prox(chip);
							do_calibration = true;
							SENSOR_LOG_INFO("do_calibration prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
						}
					}
				}
			}
			else
			{
				SENSOR_LOG_INFO("prox_th_low:%d prx_inf.raw:%d very_near_count:%d\n",
				        prox_th_low,chip->prx_inf.raw,chip->prx_inf.very_near_count);
				/* Judge whether the very near condition is triggered or not */
				if (chip->prx_inf.raw > chip->params.prox_th_verynear
					&& chip->prx_inf.very_near_count < 4)
				{
				    update_persist=true;
					if (++chip->prx_inf.very_near_count == 4)
					{
						/* Very near condition is triggered */
						prox_th_low = chip->params.prox_th_contaminated;
						prox_th_high = AMS_PROX_MAX_VALUE;
						update_thresholds=true;
						SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
					}
				}
				else
				{
					/* Raw data is bigger than high thresh, stop to collect
						raw data, and reset the ring buffer */
					prox_th_high = AMS_PROX_MAX_VALUE;
					update_thresholds=true;
					chip->prx_inf.data_count = 0;
					SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
				}
			}
		}
		else
		{
			SENSOR_LOG_INFO("prx_inf.raw :%d prox_th_high:%d prox_th_low:%d \n",chip->prx_inf.raw ,prox_th_high,prox_th_low);
			/* Far away state Prox >prox_th_max */
			if (chip->prx_inf.raw > prox_th_high)
			{
				/* Change to Near by state */
				chip->prx_inf.detected = true;
				chip->prx_inf.detected = PROX_NEAR;
			//	SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
				if (prox_th_low == 1)
				{
					/* It means raw data can be lower than SNS_DD_PROX_THRESH_FAR */
					//set thres  Low 50
					prox_th_low = chip->params.prox_thres_far;
				}
				else
				{
					/* Otherwise set threshold higher to guarantee the Far away state
						can be triggered*/
					prox_th_low = (prox_th_high - 5) > 1 ? (prox_th_high - 5) : 1;
				}
				prox_th_high = chip->params.prox_th_verynear;
				ret = true;
				update_thresholds=true;
				SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
				tmd3702_report_prox(chip);
			}
			else if (chip->prx_inf.raw < prox_th_low)
			{
				/* Might just after offset calibration */
				if (chip->prx_inf.raw == 0)
				{
					//nubia add code change this 2018-05-18
					do_calibration = true;
					SENSOR_LOG_INFO("do_calibration prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
				}
				else if (chip->prx_inf.raw < chip->params.prox_thres_far) //0<prox<50
				{
					/* Raw data is low enough, set threshold */
					prox_th_low = 1;
					//set thresholds [1,80]
					prox_th_high = chip->params.prox_thres_near;
					update_thresholds=true;
					SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
				}
				else //prox >50
				{
					/* Raw data is still high, set proper threshold to guarantee the Near by and
					Far away events can be triggered, and call offset calibration again */
					prox_th_low = (chip->prx_inf.raw - 10) > 1 ? (chip->prx_inf.raw - 10) : 1;
					prox_th_high = (chip->prx_inf.raw + 10) < chip->params.prox_thres_near ?
									chip->params.prox_thres_near : (chip->prx_inf.raw + 10);
					//max(prox-10,1)
					update_thresholds=true;
					SENSOR_LOG_INFO("prox_th_low:%d prox_th_high:%d\n",prox_th_low,prox_th_high);
				}
			}
		}
	}

	SENSOR_LOG_INFO("do_calibration:%d update_thresholds:%d\n",do_calibration,update_thresholds);
	chip->params.prox_th_min=prox_th_low;
	chip->params.prox_th_max=prox_th_high;

    chip->params.prox_persist_update = update_persist;
    SENSOR_LOG_INFO("persist_l 1prox_persist_update:0x%x\n",chip->params.prox_persist_update);  
    if(chip->params.prox_persist_update)
    {
        ams_i2c_modify(chip, chip->shadow, TMD3702_REG_PERS,TMD3702_MASK_PROX_PERS,    
            PROX_PERS_1_CONSECUTIVE_PROX_VALUE & TMD3702_MASK_PROX_PERS);
    }
    else
    {
        ams_i2c_modify(chip, chip->shadow, TMD3702_REG_PERS,TMD3702_MASK_PROX_PERS,    
            PROX_PERS_2_CONSECUTIVE_PROX_VALUE & TMD3702_MASK_PROX_PERS);
    }
    ams_i2c_read(chip->client,  TMD3702_REG_PERS,  &persist_l);
    SENSOR_LOG_INFO("persist_l read1:0x%x\n",persist_l);
	if (do_calibration)
	{
		/* Do calibration again */
		//tmd3702_i2c_write_byte(state, TMD3702_ENABLE_REG, PON);
		//tmd3702_i2c_modify(state, TMD3702_INTENAB_REG, CIEN, CIEN);
		//tmd3702_i2c_write_byte(state, TMD3702_CALIB_REG, START_OFFSET_CALIB);
		//state->chip.in_calib = true;
		//state->chip.calib_count = 0;
		/* Start timer */
		//sns_ddf_timer_start(state->calib_timer, 1000*INTERVAL_IN_CALIB);
		SENSOR_LOG_INFO("Do calibration again   !!form OIL FAR--> to FAR\n");
		//chip->params.prox_oil_state = OIL_NONE;
		tmd3702_schedule_prox_work(chip, PROX_STATE_CALIB);

	}

	if(update_thresholds)//	if (int_trigger)
	{
		//tmd3702_i2c_write_byte(state, TMD3702_PILTL_REG, prox_th_low);
		//tmd3702_i2c_write_byte(state, TMD3702_PIHTL_REG, prox_th_high);
		ams_i2c_write(chip->client, sh, TMD3702_REG_PILTL,( prox_th_low & 0xFF));
		ams_i2c_write(chip->client, sh, TMD3702_REG_PIHTL, (prox_th_high & 0xFF));
	}

}
void tmd3702_get_prox_nubia(struct tmd3702_chip *chip)
{
	 u8 *sh = chip->shadow;
	struct i2c_client *client = chip->client;
	bool update_thresholds = false;
	bool update_offset = false;
	bool update_release_offset=false;
	//add code first 0
	/* state machine */
	if (chip->prx_inf.detected == PROX_NONE) {
		if (chip->prx_inf.raw < chip->params.prox_th_min) {
		    SENSOR_LOG_INFO(" prox detect PROX_FAR!\n" );
		    chip->prx_inf.detected = PROX_FAR;
		    tmd3702_report_prox(chip);
		    chip->params.prox_th_min = 0;
		    chip->params.prox_th_max = chip->params.prox_thres_near;
		    update_thresholds = true;
		   SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
		}
		else if (chip->prx_inf.raw > chip->params.prox_th_max){
		    chip->prx_inf.detected = PROX_NEAR;
		    SENSOR_LOG_INFO("prox detect PROX_NEAR!\n" );
		    tmd3702_report_prox(chip);
		    chip->params.prox_th_min = chip->params.prox_thres_far;
		    chip->params.prox_th_max = chip->params.prox_thres_oil_near;
		    update_thresholds = true;
		     update_release_offset = true;
		    SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
		}
	}

	//tmd3702_read_regs(chip);
	switch(chip->prx_inf.detected)
	{
		case PROX_FAR:

			if (chip->prx_inf.raw > chip->params.prox_th_max) {
				SENSOR_LOG_INFO("prox detect\n");
				if (chip->prox_state == PROX_STATE_NONE) {
					chip->prx_inf.detected = PROX_NEAR;
					chip->params.prox_oil_state = OIL_NONE;
					SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!FAR-->NEAR\n");
					SENSOR_LOG_INFO("!!Form FAR--> to NEAR\n");
					tmd3702_report_prox(chip);
					chip->params.prox_th_min = chip->params.prox_thres_far;
					chip->params.prox_th_max = chip->params.prox_thres_oil_near;
					update_thresholds = true;
					update_release_offset=true;
					SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
				}
			}
			if (chip->prx_inf.raw < chip->params.prox_th_min)
			{
				if (chip->prox_state == PROX_STATE_NONE)
				{
					SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!FAR-->FAR\n");
					SENSOR_LOG_INFO("!!form FAR--> to FAR\n");
					chip->prx_inf.detected = PROX_FAR;
					tmd3702_report_prox(chip);
					chip->params.prox_th_min = 0;
					chip->params.prox_th_max = chip->params.prox_thres_near;
					update_thresholds = true;

					SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
					if (chip->params.prox_oil_state) {
						SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!OIL FAR-->FAR\n");

						SENSOR_LOG_INFO("!!form OIL FAR--> to FAR\n");
					    chip->params.prox_oil_state = OIL_NONE;
					    tmd3702_schedule_prox_work(chip, PROX_STATE_CALIB);
					}
				}//end of prox_state:PROX_STATE_NONE
			}
			break;
		case PROX_NEAR:
			if (chip->prx_inf.raw < chip->params.prox_th_min)
			{
				SENSOR_LOG_INFO("prox release\n");
				if (chip->prox_state == PROX_STATE_NONE)
				{
					chip->prx_inf.detected = PROX_FAR;
					tmd3702_report_prox(chip);
					if (chip->params.prox_th_max != PS_DATA_MAX) {
						//chip->params.poffset = min(chip->params.poffset + 10, 255);
						SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!NEAR-->FAR\n");

						SENSOR_LOG_INFO("!!NEAR-->FAR\n");
						update_thresholds = true;

						chip->params.prox_th_min = 0;
						chip->params.prox_th_max = chip->params.prox_thres_near;
						SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
					}
					else {
						update_thresholds = true;
						SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!OIL NEAR-->FAR\n");
						SENSOR_LOG_INFO("!!OIL NEAR-->FAR\n");
						chip->params.prox_th_min = chip->params.prox_thres_near - 10;
						chip->params.prox_th_max = chip->params.prox_thres_oil_far + 10;
						SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
					}
				}
			}
			else if (chip->prx_inf.raw > chip->params.prox_th_max)
			{
				if (chip->prox_state == PROX_STATE_NONE) {
					chip->prx_inf.detected = PROX_NEAR;
					chip->params.prox_oil_state = OIL_STATE;
					SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!OIL NEAR->NEAR\n");

					SENSOR_LOG_INFO("!!OIL NEAR->NEAR\n");
					tmd3702_report_prox(chip);
					chip->params.prox_th_min = chip->params.prox_thres_oil_far;
					chip->params.prox_th_max = PS_DATA_MAX;
					update_thresholds = true;
					SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
				}
			}
			break;
		default:
			//SENSOR_LOG_DEBUG_IF("test othe\n");
			break;
	}

	if(!chip->prx_inf.prox_raw_debug)
	{
	    // If proximity is detected, set the lower limit to find
            // proximity release and set the higher limit to avoid
            // repeated interrupts.
		//SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max);

		if (update_release_offset)
		{
			ams_i2c_write(chip->client, sh, TMD3702_REG_PILTL, chip->params.prox_th_min & 0xFF);
			ams_i2c_write(chip->client, sh, TMD3702_REG_PIHTL, (PROX_MAX_THRSHLD & 0xFF));
		}
		if(update_thresholds)
		{
			ams_i2c_write(chip->client, sh, TMD3702_REG_PILTL, chip->params.prox_th_min & 0xFF);
			ams_i2c_write(chip->client, sh, TMD3702_REG_PIHTL,  chip->params.prox_th_max & 0xFF);
		}

		SENSOR_LOG_INFO("debug mode or not update threshold %d,prox_raw_debug:%d \n",
			update_thresholds,chip->prx_inf.prox_raw_debug);
	}

	SENSOR_LOG_INFO(" prox_th_min:%d prox_th_max:%d\n", chip->params.prox_th_min, chip->params.prox_th_max);

	if (update_offset) {
		SENSOR_LOG_INFO("update_offset: %d\n", chip->params.poffset);
		if (chip->params.poffset <= 0) {
			ams_i2c_write(client, sh, TMD3702_REG_POFFSET_L,chip->params.poffset * (-1));
		}
		else
		{
			ams_i2c_write(client, sh, TMD3702_REG_POFFSET_L,chip->params.poffset );
		}
		ams_i2c_write(client, sh, TMD3702_REG_POFFSET_H,chip->params.poffset < 0 ? 1 : 0);
	}
}
void tmd3702_get_prox(struct tmd3702_chip *chip)
{

    if(1){
	//	SENSOR_LOG_INFO("debug_level:%d\n",chip->pdata->debug_level);
		SENSOR_LOG_INFO("prox_thres_far:%d prox_thres_near:%d prox_thres_oil_far:%d prox_thres_oil_near:%d\n",
			chip->params.prox_thres_far,
			chip->params.prox_thres_near,
			chip->params.prox_thres_oil_far,
			chip->params.prox_thres_oil_near);
		SENSOR_LOG_INFO("th_min:%d th_max:%d raw:%d state:%d detected:%d detected:%s\n",
			chip->params.prox_th_min,
			chip->params.prox_th_max,
			chip->prx_inf.raw,
			chip->prox_state,
			chip->prx_inf.detected,
			chip->prx_inf.detected == PROX_NEAR ? "NEAR" : "FAR");
   	}

	if (chip->prox_state != PROX_STATE_NONE) {
		SENSOR_LOG_INFO("prox state is %d, calibration is pending.\n",(int) chip->prox_state);
	        return;
	}

	if (1)
	{
		tmd3702_get_prox_ams2(chip);
		return;
	}
	else if (0)
	{
		tmd3702_get_prox_nubia(chip);
		return;
	}
	return;


}

void tmd3702_report_prox(struct tmd3702_chip *chip)
{
    if (chip->p_idev)
    {
		SENSOR_LOG_INFO("detect:%d,last:%d\n",chip->prx_inf.detected,chip->prx_inf.last_detected );
		if (chip->prx_inf.detected != chip->prx_inf.last_detected)
		{
			//input_report_abs(chip->p_idev, ABS_DISTANCE,    chip->prx_inf.detected ? 1 : 0);
			input_report_rel(chip->p_idev, REL_RZ, chip->prx_inf.detected);
			input_sync(chip->p_idev);

            //virtual_proximity_report_event(chip->prx_inf.detected, REL_RZ);
			SENSOR_LOG_INFO("prox status: %s(%d)\n",chip->prx_inf.detected == PROX_NEAR ? "NEAR" : "FAR",
				chip->prx_inf.detected);
			chip->prx_inf.last_detected = chip->prx_inf.detected;
		}
    }
}

void tmd3702_set_prox_mode(struct tmd3702_chip *chip)
{
    u8 *sh = chip->shadow;
    u8 rPGCFG0;
    u8 rPGCFG1;
    u16 lux = chip->als_inf.lux;

    if (lux <= INDOOR_LUX_TRIGGER) // Indoor light conditions
    {
        chip->amsIndoorMode = true;
        rPGCFG0 = PG_PULSE_32US;       // Pulse len
        rPGCFG1 = PGAIN_4 | PDRIVE_MA(75); // Gain and drive current
    }
    if (lux >= OUTDOOR_LUX_TRIGGER) // Outdoor (bright sun) light conditions
    {
        chip->amsIndoorMode = false;
        rPGCFG0 = PG_PULSE_4US;       // Pulse len
        rPGCFG1 = PGAIN_2 | PDRIVE_MA(75); // Gain and drive current
    }

    // If a change was made then push it to the device
    if (rPGCFG0 != sh[TMD3702_REG_PGCFG0])
        ams_i2c_write(chip->client, sh, TMD3702_REG_PGCFG0, rPGCFG0);
    if (rPGCFG1 != sh[TMD3702_REG_PGCFG1])
        ams_i2c_write(chip->client, sh, TMD3702_REG_PGCFG1, rPGCFG1);
}
static int tmd3702_mean_prox_calc(struct tmd3702_chip *chip)
{
	int mean = 0, i;
	int tmp[PS_AVG_TIME] = { 0 };
	for(i = 0; i < PS_AVG_TIME; i++)
	{
		tmd3702_read_prox(chip);//?????
		SENSOR_LOG_ERROR("chip->prx_inf.raw = %d\n",chip->prx_inf.raw);
		tmp[i] += chip->prx_inf.raw;
		msleep(30);
	}
	sensor_quick_sort(tmp, PS_AVG_TIME, 0, PS_AVG_TIME - 1);
	for (i = PS_AVG_TIME / 4; i < 3 * PS_AVG_TIME / 4; i++)
		mean += tmp[i];
	mean = 2 * mean / PS_AVG_TIME;
	return mean;
}
int tmd3702_offset_calibration(struct tmd3702_chip *chip)
{
    u8 *sh = chip->shadow;
    u8 saveenab, saveint;
    int calwait = 0;
    int ret;
    u8 offset_l;
    s16 offset;

    SENSOR_LOG_INFO("enter\n");

    /* save PEN state */
    ams_i2c_read(chip->client, TMD3702_REG_ENABLE, &saveenab);

    /* save prox intr state */
    ams_i2c_read(chip->client, TMD3702_REG_INTENAB, &saveint);

    /* turn on power, disable prox */
//ams 2018-05-17 change this register TMD3702_EN_ALL  0xDD
      ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
           TMD3702_EN_ALL | TMD3702_PON,			//TMD3702_PEN | TMD3702_PON
            TMD3702_PON);

        //add nubia delay
	mdelay(5);
    /* enable calib intr, disable prox intr */
//ams 2018-05-17 change TMD3702_AIEN  this register 0xDD
    ams_i2c_modify(chip, sh, TMD3702_REG_INTENAB,
          TMD3702_AIEN |TMD3702_PIEN|TMD3702_CIEN, TMD3702_CIEN);	//add TMD3702_AIEN

    /*
    ** Prox Offset calibration
    **   binsrch_target (7 counts)
    **   prox averaging (2 reading window)
    **   prox_auto_offset_adjust
    */
//ams 2018-05-17 change this register 0xD9 calibcfg    bit 1->2 -> bit 1-2
    ams_i2c_modify(chip, sh, TMD3702_REG_CALIBCFG,
            TMD3702_MASK_BINSRCH_TARGET |
                TMD3702_MASK_PROX_DATA_AVG |
                TMD3702_MASK_PROX_AUTO_OFFSET_ADJUST,
            (0x04 << TMD3702_SHIFT_BINSRCH_TARGET) |	// (0x01 << TMD3702_SHIFT_BINSRCH_TARGET) |
                (2 << TMD3702_SHIFT_PROX_DATA_AVG) |	//(1 << TMD3702_SHIFT_PROX_DATA_AVG) |
                (1 << TMD3702_SHIFT_PROX_AUTO_OFFSET_ADJUST));

    /* trigger calibration sequence */
 //   dev_info(&chip->client->dev, "offset calibration started.\n");
    chip->amsCalComplete = false;

	SENSOR_LOG_INFO("offset calibration started.\n");

    ams_i2c_modify(chip, sh, TMD3702_REG_CALIB,
            TMD3702_MASK_START_OFFSET_CALIB,
            0x01 << TMD3702_SHIFT_START_OFFSET_CALIB);

    /* amsCalComplete set true in IRQ Handler */
    for (calwait = 0; (!chip->amsCalComplete) && (calwait < 10); calwait++) {
        // TODO: change to signal and eliminate amsCalComplete & sleep
        msleep(10);
    }

    if (calwait < 10) {
        ret = 0;
    } else {
        SENSOR_LOG_ERROR("No Calibration IRQ, exiting spin loop\n");
        ret = -1;
    }

    // get updated prox offset
    /* Check if proximity is saturated */
    ams_i2c_blk_read(chip->client, TMD3702_REG_POFFSET_L,
            &chip->shadow[TMD3702_REG_POFFSET_L], 2);

	chip->params.poffset = chip->shadow[TMD3702_REG_POFFSET_L];
	if (chip->shadow[TMD3702_REG_POFFSET_H] & TMD3702_MASK_POFFSET_H)
	{
		chip->params.poffset *= -1;
	}

	SENSOR_LOG_INFO("*#777#   chip->params.poffset = %d POFFSET_H:%d\n",chip->params.poffset,
		chip->shadow[TMD3702_REG_POFFSET_H]);

	offset = chip->params.poffset;
	if ((offset <=  TMD3702_MAX_OFFSET) && (offset >= TMD3702_MIN_OFFSET))
	{
		SENSOR_LOG_ERROR("calibrate_prox() offset:%d ok \n",offset);
		ret =0;
	}
	else
	{
		/* Limit the max offset ams add code  */

		if (offset > TMD3702_MAX_OFFSET)
		{
			offset_l = TMD3702_MAX_OFFSET;
			ams_i2c_write(chip->client, sh,TMD3702_REG_POFFSET_L, offset_l);
		}
		if (offset < TMD3702_MIN_OFFSET)
		{
			offset_l = -(TMD3702_MIN_OFFSET);
			ams_i2c_write(chip->client, sh,TMD3702_REG_POFFSET_L, offset_l);
		}
		 ret = -1;
	}
	/// ams change code 2018-05-17
	ams_i2c_write(chip->client, sh,TMD3702_REG_ENABLE, saveenab);
	ams_i2c_write(chip->client, sh, TMD3702_REG_INTENAB,saveint);

	SENSOR_LOG_INFO("Optical Crosstalk calibration complete.\n");

	return ret;
}

void tmd3702_init_prox_mode(struct tmd3702_chip *chip,bool first)
{

///	add nubia code
	chip->params.prox_th_min = 0;
	chip->params.prox_th_max = chip->params.prox_thres_near;
	if (chip->prx_inf.prox_raw_debug || first){
		chip->params.prox_th_min = PS_DEFAULT_THRES;
		chip->params.prox_th_max = PS_DEFAULT_THRES;
	}

   SENSOR_LOG_IF (chip->pdata->debug_level,"prox_th_min:%d prox_th_max:%d \n",
        chip->params.prox_th_min,chip->params.prox_th_max);

    //Init Low THRSHLD to 0,
    //(we are only looking for detect events after init)
  #ifdef TMD3702_APC_ENABLE

    ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_PILTL, 0);
    ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_PILTH, 0);
    ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_PIHTL,
		(chip->params.prox_th_max & 0xFF));
    ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_PIHTH,
		((chip->params.prox_th_max >> 8) & TMD3702_MASK_PIHTH));

  #else
  //nubia code 2018-05-16 code ams change code
    ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_PILTL,
	    (chip->params.prox_th_min& 0xFF));
   ams_i2c_write(chip->client, chip->shadow, TMD3702_REG_PIHTL,
		(chip->params.prox_th_max & 0xFF));

 #endif
}

int tmd3702_configure_prox_mode(struct tmd3702_chip *chip, u8 state)
{
    extern void tmd3702_reg_log(struct tmd3702_chip *chip);
    struct i2c_client *client = chip->client;
    static int first_open_ps = 1;
    u8 *sh = chip->shadow;

    if (state) // Turning on prox
    {
        // Configure default proximity settings
        tmd3702_init_prox_mode(chip,true);

        if (1 == first_open_ps)
        {
            tmd3702_offset_calibration(chip);
            first_open_ps--;
        }
	//IRQ close
      //  tmd3702_offset_calibration(chip);

        ams_i2c_modify(chip, sh, TMD3702_REG_PERS,
                TMD3702_MASK_PROX_PERS,
                chip->params.persist & TMD3702_MASK_PROX_PERS);
        ams_i2c_write(client, sh, TMD3702_REG_PGCFG0,
                chip->params.prox_pulse_cnt |
                chip->params.prox_pulse_len<< TMD3702_SHIFT_PPULSE_LEN);

        ams_i2c_modify(chip, sh, TMD3702_REG_PGCFG1,TMD3702_MASK_PGAIN|TMD3702_MASK_PDRIVE,
                  ((chip->params.prox_gain << TMD3702_SHIFT_PGAIN)& TMD3702_MASK_PGAIN)   |
                  (chip->params.prox_drive & TMD3702_MASK_PDRIVE)   );

 //ams add code WTIME TMD3702_REG_WTIME 100ms to
    	ams_i2c_write(client, sh,  TMD3702_REG_PRATE, P_TIME_US(TMD3702_PROXMITY_TIME));
    	ams_i2c_write(client, sh,  TMD3702_REG_WTIME, WAIT_TIME_MS(TMD3702_WAIT_TIME_MS));

        // Enable Proximity and Proximity Interrupt
//ams add code  als_enable and prox_enable
	if (chip->als_enabled)
	    ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
			    TMD3702_WEN | TMD3702_AEN | TMD3702_PEN | TMD3702_PON,
			    TMD3702_AEN |TMD3702_PEN | TMD3702_PON);
	else
	    ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
			    TMD3702_WEN | TMD3702_AEN | TMD3702_PEN | TMD3702_PON,
			    TMD3702_WEN |TMD3702_PEN | TMD3702_PON);

        ams_i2c_modify(chip, sh, TMD3702_REG_INTENAB,
                TMD3702_PIEN, TMD3702_PIEN);

        chip->prx_enabled = true;
        chip->amsFirstProx = true;
        chip->amsIndoorMode = true;
    }
    else // Turning off prox
    {
        // Disable Proximity and Proximity Interrupt
		//ams change this code add TMD3702_WEN
		if (chip->als_enabled)
			ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
					TMD3702_WEN | TMD3702_AEN | TMD3702_PEN | TMD3702_PON,
					TMD3702_AEN | TMD3702_PON);
		else
			ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE,
					 TMD3702_WEN | TMD3702_PEN | TMD3702_PON,
					TMD3702_PON);
        ams_i2c_modify(chip, sh, TMD3702_REG_INTENAB,
                TMD3702_PIEN, 0);

        // If nothing else is enabled set PON = 0
        if (!(sh[TMD3702_REG_ENABLE] & TMD3702_EN_ALL))
            ams_i2c_modify(chip, sh, TMD3702_REG_ENABLE, TMD3702_PON, 0x00);

        chip->prx_enabled = false;
        chip->amsFirstProx = true;
	//above nubia add code
	chip->prx_inf.last_detected = PROX_NONE;
	chip->prx_inf.detected = PROX_NONE;
    }

	SENSOR_LOG_INFO("prox_thres_near:%d prox_thres_far:%d \n",
        chip->params.prox_thres_near ,chip->params.prox_thres_far);

    return(0);
}
/************************************************************************************/
/*  Functions  nubia add  */
//1.static int tmd3702_prox_report_first_event(struct tmd3702_chip *chip)
//2. static int tmd3702_prox_thres_calibrate(struct tmd3702_chip *chip)
//3. static int tmd3702_prox_thres_calibrate(struct tmd3702_chip *chip)
//4.
/************************************************************************************/
static int tmd3702_prox_report_first_event(struct tmd3702_chip *chip)
{
	bool update_thresholds = false;
	u8 *sh = chip->shadow;
//	struct i2c_client *client = chip->client;
	/* enabled and in first irq state, we need report distance when enable */
	if (chip->prx_inf.detected == PROX_NONE) {
		mdelay(20);
		tmd3702_read_prox(chip);
		//ams  change  code this
 		if (chip->prx_inf.raw < chip->params.prox_th_min) {
		    chip->prx_inf.detected = PROX_FAR;
		    tmd3702_report_prox(chip);
		    chip->params.prox_th_min = 0;
		    chip->params.prox_th_max = chip->params.prox_thres_near;
			SENSOR_LOG_INFO("min:%d max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
			SENSOR_LOG_INFO("near:%d far:%d \n",chip->params.prox_thres_near ,chip->params.prox_thres_far  );
		    update_thresholds = true;
		} else if (chip->prx_inf.raw > chip->params.prox_th_max){
		    chip->prx_inf.detected = PROX_NEAR;
		    tmd3702_report_prox(chip);
		    chip->params.prox_th_min = chip->params.prox_thres_far;
		    chip->params.prox_th_max = chip->params.prox_thres_oil_near;
			SENSOR_LOG_INFO("min:%d max:%d \n",chip->params.prox_th_min,chip->params.prox_th_max );
			SENSOR_LOG_INFO("near:%d far:%d \n",chip->params.prox_thres_near ,chip->params.prox_thres_far  );
		    update_thresholds = true;
		}

	}
	if (update_thresholds && (!chip->prx_inf.prox_raw_debug)) {


		SENSOR_LOG_INFO("update_thresholds: min:%d max:%d\n", chip->params.prox_th_min, chip->params.prox_th_max);
		SENSOR_LOG_ERROR("add code for this function \n");

        // If proximity is detected, set the lower limit to find
        // proximity release and set the higher limit to avoid
        // repeated interrupts.
  #ifdef TMD3702_APC_ENABLE
        ams_i2c_write(chip->client, sh, TMD3702_REG_PILTL,   (chip->params.prox_th_min & 0xFF));
        ams_i2c_write(chip->client, sh, TMD3702_REG_PILTH,   ((chip->params.prox_th_min >> 8) & TMD3702_MASK_PILTH));

        ams_i2c_write(chip->client, sh, TMD3702_REG_PIHTL,   (PROX_MAX_THRSHLD & 0xFF));
        ams_i2c_write(chip->client, sh, TMD3702_REG_PIHTH,  ((PROX_MAX_THRSHLD >> 8) & TMD3702_MASK_PIHTH));
#else

		ams_i2c_write(chip->client, sh, TMD3702_REG_PILTL,  (chip->params.prox_th_min & 0xFF));
		ams_i2c_write(chip->client, sh, TMD3702_REG_PIHTL,  (chip->params.prox_th_max & 0xFF));
        // ams_i2c_write(chip->client, sh, TMD3702_REG_PIHTL,  (PROX_MAX_THRSHLD & 0xFF));

#endif

	}
    else {

		SENSOR_LOG_IF(chip->pdata->debug_level,"debug mode, not update threshold\n");
	}
	return 0;
}
static int tmd3702_prox_set_enable(struct tmd3702_chip *chip, u8 enable)
{
	int ret;
	SENSOR_LOG_INFO("enter\n");
	if (!chip->pdata->power_always_on) {
		if (enable) {
			if (chip->pdata->power_state == POWER_OFF)
			    sensor_regulator_power_on(chip, true);
			chip->pdata->power_state |= POWER_PS_ON;
		} else {
			chip->pdata->power_state &= ~POWER_PS_ON;
			if (!chip->prox_enabled && !chip->als_enabled && chip->pdata->power_state == POWER_OFF)
			    sensor_regulator_power_on(chip, false);
		}
	}

	ret = tmd3702_configure_prox_mode(chip, enable);
	if (ret < 0) {
		SENSOR_LOG_ERROR("prox turn %s failed\n", enable?"on":"off");
		return ret;
	}

	sensor_irq_enable(chip, enable, true);

	SENSOR_LOG_INFO("prox_thres_near:%d prox_thres_far:%d \n",
        chip->params.prox_thres_near ,chip->params.prox_thres_far);

	if (enable) {
		tmd3702_prox_report_first_event(chip);

		SENSOR_LOG_INFO("add code for this function chip->prx_inf.raw %d  \n",chip->prx_inf.raw );
		if (chip->prx_inf.raw <= PS_OFFSET_CAL_THRESH  )
		    tmd3702_do_prox_offset_cal(chip, PROX_STATE_CALIB);
	}

	SENSOR_LOG_INFO("prox turn %s\n", enable?"on":"off");
	return 0;
}
static int tmd3702_prox_thres_calibrate(struct tmd3702_chip *chip)
{
	int ret;
	u8 prox_thres_near;
	u8 prox_buf[2];
	SENSOR_LOG_INFO("enter\n");
	/*zhanglizhen charge start for 2018.9.18*/
	prox_thres_near = tmd3702_mean_prox_calc(chip);
	if (prox_thres_near < PS_THRESH_DATA_MIN || prox_thres_near > PS_THRESH_DATA_MAX) {
		SENSOR_LOG_ERROR("thres calibration failed\n");
		return -ERR_THRES_CAL;
	}else{
		chip->params.prox_thres_near = prox_thres_near;
	}
	/*zhanglizhen charge end for 2018.9.13*/
	chip->params.prox_thres_far = chip->params.prox_thres_near / 2 + 10;
	prox_buf[0] = chip->params.prox_thres_near;
	prox_buf[1] = chip->params.prox_thres_far;

	SENSOR_LOG_INFO("chip->params.prox_thres_near = %d prox_thres_far:%d\n",
        chip->params.prox_thres_near,chip->params.prox_thres_far);

	ret = sensor_write_file(PS_CAL_FILE_PATH, prox_buf, sizeof(prox_buf));
	if (ret < 0) {
		SENSOR_LOG_ERROR("write file fail\n");
		return ret;
	}

	SENSOR_LOG_INFO("prox_thres_near:%d prox_thres_far:%d \n",
        chip->params.prox_thres_near ,chip->params.prox_thres_far);

	return chip->params.prox_thres_near;
}
/**********************************************************************************************/
/* ABI Functions */
/********************************************************************************************/

static ssize_t tmd3702_device_prox_raw(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    tmd3702_read_prox(chip);

    return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmd3702_device_prox_detected(struct device *dev,
                                            struct device_attribute *attr,
                                            char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    u8 val;

    // if prox intr enabled, just grab the flag that has been set
    ams_i2c_read(chip->client, TMD3702_REG_INTENAB, &val);
    if (!(val & TMD3702_PIEN)) {
        tmd3702_read_prox(chip);
        tmd3702_get_prox(chip);
    }

    return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t tmd3702_prox_gain_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", 1 << chip->params.prox_gain);
}

static ssize_t tmd3702_prox_gain_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    unsigned long gain;
    u8 regVal = 0;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 10, &gain);

    if (rc)
        return -EINVAL;

    switch(gain)
    {
    case 0:
        regVal = 0;
        break;
    case 1:
        regVal = PGAIN_1;
        break;
    case 2:
        regVal = PGAIN_2;
        break;
    case 4:
        regVal = PGAIN_4;
        break;
    case 8:
        regVal = PGAIN_8;
        break;
    default:
        return -EINVAL;
    }

    AMS_MUTEX_LOCK(&chip->lock);
    rc = ams_i2c_modify(chip, chip->shadow, TMD3702_REG_PGCFG1,
            TMD3702_MASK_PGAIN, regVal);
    chip->params.prox_gain =
      chip->shadow[TMD3702_REG_PGCFG1] & TMD3702_MASK_PGAIN;
    AMS_MUTEX_UNLOCK(&chip->lock);

    return size;
}

static ssize_t tmd3702_prox_offset_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    u8 rPoffsetl = 0;
    u8 rPoffseth = 0;
    int prxofs = 0;

    // must read it from chip to get calibration result
    ams_i2c_read(chip->client, TMD3702_REG_POFFSET_L, &rPoffsetl);
    ams_i2c_read(chip->client, TMD3702_REG_POFFSET_H, &rPoffseth);
    prxofs = (rPoffseth & 0x01) ? -((int)rPoffsetl) : ((int)rPoffsetl);

    return snprintf(buf, PAGE_SIZE, "%d\n", prxofs);
}

static ssize_t tmd3702_prox_persist_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n",
            ((chip->params.persist & 0xf0) >> 4));
}

static ssize_t tmd3702_prox_persist_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    unsigned long persist;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 10, &persist);
    if (rc)
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);
    ams_i2c_modify(chip, chip->shadow,
                   TMD3702_REG_PERS, TMD3702_MASK_PROX_PERS, persist);
    chip->params.persist = chip->shadow[TMD3702_REG_PERS];
    AMS_MUTEX_UNLOCK(&chip->lock);

    return size;
}

static ssize_t tmd3702_prox_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmd3702_prox_enable_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    unsigned long enable;
    int rc;
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 10, &enable);
    if (rc)
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);

    switch(enable)
    {
    case 0:
        // Disable prox
        tmd3702_prox_set_enable(chip, 0);
        //tmd3702_configure_prox_mode(chip, 0);
        break;

    case 1:
        // Enable prox
        tmd3702_prox_set_enable(chip, 1);
       // tmd3702_configure_prox_mode(chip, 1);
        break;
    default:
        return -EINVAL;
    }

    AMS_MUTEX_UNLOCK(&chip->lock);

    return size;
}

static ssize_t tmd3702_prox_regs_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
    struct tmd3702_chip *chip = dev_get_drvdata(dev);

    u8 rEnable = 0;
    u8 rPrate = 0;
    u8 rPiltl = 0;
    u8 rPilth = 0;
    u8 rPihtl = 0;
    u8 rPihth = 0;
    u8 rPers = 0;
    u8 rPgcfg0 = 0;
    u8 rPgcfg1 = 0;
    u8 rCfg1 = 0;
    u8 rStatus = 0;
    u8 rCfg2 = 0;
    u8 rIntenab = 0;

    ams_i2c_read(chip->client, TMD3702_REG_ENABLE, &rEnable);
    ams_i2c_read(chip->client, TMD3702_REG_PRATE, &rPrate);
    ams_i2c_read(chip->client, TMD3702_REG_PILTL, &rPiltl);
    ams_i2c_read(chip->client, TMD3702_REG_PILTH, &rPilth);
    ams_i2c_read(chip->client, TMD3702_REG_PIHTL, &rPihtl);
    ams_i2c_read(chip->client, TMD3702_REG_PIHTH, &rPihth);
    ams_i2c_read(chip->client, TMD3702_REG_PERS, &rPers);
    ams_i2c_read(chip->client, TMD3702_REG_PGCFG0, &rPgcfg0);
    ams_i2c_read(chip->client, TMD3702_REG_PGCFG1, &rPgcfg1);
    ams_i2c_read(chip->client, TMD3702_REG_CFG1, &rCfg1);
    ams_i2c_read(chip->client, TMD3702_REG_STATUS, &rStatus);
    ams_i2c_read(chip->client, TMD3702_REG_CFG2, &rCfg2);
    ams_i2c_read(chip->client, TMD3702_REG_INTENAB, &rIntenab);

    return snprintf(buf, PAGE_SIZE,
        "ENABLE =   %2x\nPRATE  =   %2x\nPILT   =   %4x\nPIHT   =   %4x\n"
        "PERS   =   %2x\nPGCFG0 =   %2x\nPGCFG1 =   %2x\nCFG1   =   %2x\n"
        "CFG2   =   %2x\nSTATUS =   %2x\nINTENAB=   %2x\n"
        "%s\n%s settings\n",
        rEnable,
        rPrate,
        (rPilth << 8) | rPiltl,
        (rPihth << 8) | rPihtl,
        rPers,
        rPgcfg0,
        rPgcfg1,
        rCfg1,
        rCfg2,
        rStatus,
        rIntenab,
        chip->prx_inf.detected ? "Prox Detect" : "Prox Release",
        chip->amsIndoorMode ? "Indoor" : "Outdoor");
}

/************************************************************************************/
/* ABI Functions  nubia add function list  */
//1.static ssize_t tmd2725_prox_offset_show
//2.static ssize_t tmd3702_prox_persist_show(struct device *dev,
//3.static ssize_t tmd3702_prox_persist_store(struct device *dev,
//4.static ssize_t tmd3702_prox_regs_store(struct device *dev,
//5.static ssize_t tmd3702_prox_chip_id_show(struct device *dev,
//6.static ssize_t tmd3702_prox_chip_id_show(struct device *dev,
//7.static ssize_t tmd3702_prox_flush_show(struct device *dev,
//8.static ssize_t tmd3702_prox_dev_init_store(struct device *dev,
//9.static ssize_t tmd3702_prox_uncover_data_min_show(struct device *dev,
//10.static ssize_t tmd3702_prox_uncover_data_max_show(struct device *dev,
//11.static ssize_t tmd3702_prox_min_thres_show(struct device *dev,
//12.static ssize_t tmd3702_prox_max_data_show(struct device *dev,
//13.static ssize_t tmd3702_prox_thres_show(struct device *dev,
//14.static ssize_t tmd3702_prox_thres_store(struct device *dev,
//15.static ssize_t tmd3702_prox_debug_show(struct device *dev,
//16.static ssize_t tmd3702_prox_debug_store(struct device *dev,
//17.static ssize_t tmd3702_prox_uncover_cal_show(struct device *dev,
//18.static ssize_t tmd3702_prox_uncover_cal_store(struct device *dev,
//19.
//20.
/************************************************************************************/

static ssize_t tmd3702_prox_regs_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	u8 *sh = chip->shadow;
	int addr, cmd, ret;

	if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		SENSOR_LOG_ERROR("invalid format: '%s'\n", buf);
		return count;
	}
	ret = ams_i2c_write(chip->client, sh, addr, cmd);
	if(ret < 0)
		SENSOR_LOG_ERROR("yulhan store_reg fail");

	return count;
}



static ssize_t tmd3702_prox_chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}
static ssize_t tmd3702_prox_flush_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);

	input_report_rel(chip->p_idev, REL_RZ, chip->prx_inf.detected);
	input_sync(chip->p_idev);

    //virtual_proximity_report_event(chip->prx_inf.detected, REL_RZ);
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}

static ssize_t tmd3702_prox_dev_init_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t tmd3702_prox_dev_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val = 0;
	int rc;
	u8 data[2];
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		rc = sensor_read_file(PS_CAL_FILE_PATH, data, sizeof(data));
		if (rc < 0) {
			SENSOR_LOG_ERROR("read file error\n");
			return -1;
		}
		if (data[0] >= PS_THRESH_DATA_MIN && data[0] > data[1]) {
			chip->params.prox_thres_near = data[0];
			chip->params.prox_thres_far = data[1];
			SENSOR_LOG_ERROR("prox_thres_near:%d,prox_thres_far:%d",data[0],data[1]);
		}
		SENSOR_LOG_ERROR("chip->params.prox_thres_near = %d\n",
		            chip->params.prox_thres_near);
		SENSOR_LOG_ERROR("chip->params.prox_thres_far = %d\n",
		            chip->params.prox_thres_far);
	}
	return size;
}
static ssize_t tmd3702_prox_uncover_data_min_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_UNCOVER_DATA_MIN);
}
static ssize_t tmd3702_prox_uncover_data_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_UNCOVER_DATA_MAX);
}
static ssize_t tmd3702_prox_min_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_THRESH_DATA_MIN);
}
static ssize_t tmd3702_prox_max_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_THRESH_DATA_MAX);
}
static ssize_t tmd3702_prox_max_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_DATA_MAX);
}
static ssize_t tmd3702_prox_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", tmd3702_prox_thres_calibrate(chip));
}
static ssize_t tmd3702_prox_thres_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		rc = tmd3702_prox_thres_calibrate(chip);
	}
	return rc;
}
static ssize_t tmd3702_offset_first_event_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	//chip->pdata->offset_first_event = !chip->pdata->offset_first_event;
	return sprintf(buf, "%d\n", chip->pdata->offset_first_event);
}
static ssize_t tmd3702_offset_first_event_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	u8 offset_l,offset_h;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	u8 *sh = chip->shadow;
	rc = kstrtoint(buf, 0, &val);
	chip->pdata->offset_first_event = val;

	SENSOR_LOG_IF(chip->pdata->debug_level,"offset_first_event = %d\n",
        chip->pdata->offset_first_event);

	if(val==1)
	{
		ams_i2c_read(chip->client, TMD3702_REG_POFFSET_L, &offset_l);
		ams_i2c_read(chip->client, TMD3702_REG_POFFSET_H, &offset_h);
		SENSOR_LOG_INFO("offset_l:%d,offset_h:%d\n",offset_l,offset_h);
		chip->pdata->offset_h = offset_h;
		chip->pdata->offset_l = offset_l;
		chip->pdata->offset_data_count =0;
		offset_l =0;
		ams_i2c_write(chip->client, sh,TMD3702_REG_POFFSET_L, offset_l);
		ams_i2c_write(chip->client, sh,TMD3702_REG_POFFSET_H, offset_l);
		ams_i2c_read(chip->client, TMD3702_REG_CALIBCFG, &offset_l);
		chip->pdata->calib_cfg= offset_l;

		SENSOR_LOG_INFO("TMD3702_REG_CALIBCFG:0x%x\n",offset_l);
   		ams_i2c_modify(chip, sh, TMD3702_REG_CALIBCFG,
			TMD3702_MASK_PROX_AUTO_OFFSET_ADJUST,  0);

		ams_i2c_read(chip->client, TMD3702_REG_CALIBCFG, &offset_l);
		SENSOR_LOG_INFO("TMD3702_REG_CALIBCFG:0x%x\n",offset_l);
	}
	else if (val ==0)
	{
		chip->pdata->offset_data_count =0;
		SENSOR_LOG_INFO("restore offset_l:%d,offset_h:%d\n",chip->pdata->offset_l ,chip->pdata->offset_h);
		ams_i2c_write(chip->client, sh,TMD3702_REG_POFFSET_L, chip->pdata->offset_l );
		ams_i2c_write(chip->client, sh,TMD3702_REG_POFFSET_H, chip->pdata->offset_h);
		ams_i2c_write(chip->client, sh,TMD3702_REG_CALIBCFG, chip->pdata->calib_cfg);
	}
	return size;
}
static ssize_t tmd3702_prox_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	chip->pdata->debug_level = !chip->pdata->debug_level;
	return sprintf(buf, "%d\n", chip->pdata->debug_level);
}
static ssize_t tmd3702_prox_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoint(buf, 0, &val);
	chip->pdata->debug_level = val;
	SENSOR_LOG_INFO("val = %d\n", val);
	return size;
}
static ssize_t tmd3702_prox_raw_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->prx_inf.prox_raw_debug);
}
static ssize_t tmd3702_prox_raw_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
    u8 *sh = chip->shadow;
	rc = kstrtoint(buf, 0, &val);
	SENSOR_LOG_INFO("val = %d\n", val);
	chip->prx_inf.prox_raw_debug = val & 0xff;
#ifdef FEATURE_NUBIA_SENSOR_INFRARED
    /*zhanglizhen add code for 2018.10.23 start*/
    if (chip->prx_inf.prox_raw_debug) {
	     tmd3702_init_prox_mode(chip, false);
         ams_i2c_modify(chip, chip->shadow, TMD3702_REG_PERS,TMD3702_MASK_PROX_PERS,
            PROX_PERS_1_CONSECUTIVE_PROX_VALUE & TMD3702_MASK_PROX_PERS);
         ams_i2c_write(chip->client, sh,TMD3702_REG_WTIME, WAIT_TIME_MS(100));
	}
    else
    {
        ams_i2c_modify(chip, chip->shadow, TMD3702_REG_PERS,TMD3702_MASK_PROX_PERS,
            PROX_PERS_2_CONSECUTIVE_PROX_VALUE & TMD3702_MASK_PROX_PERS);
       ams_i2c_write(chip->client, sh,TMD3702_REG_WTIME, WAIT_TIME_MS(0));
    }
    /*zhanglizhen add code for 2018.10.23 end*/
#endif 
    return size;
}
static ssize_t tmd3702_prox_uncover_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	SENSOR_LOG_INFO("enter\n");
	chip->params.prox_raw = tmd3702_mean_prox_calc(chip);
	SENSOR_LOG_ERROR("chip->params.prox_raw=%d", chip->params.prox_raw);
	return sprintf(buf, "%d\n", chip->params.prox_raw);
}
static ssize_t tmd3702_prox_uncover_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	SENSOR_LOG_INFO("enter\n");
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		/*@ not use, uncover_cal_show will called */
		rc = tmd3702_offset_calibration(chip);
		if (rc < 0) {
			SENSOR_LOG_ERROR("*#777# calibrate fail\n");
			return size;
		}
	}
	return size;
}

struct device_attribute tmd3702_prox_attrs[] = {
	__ATTR(prox_raw, 0440, tmd3702_device_prox_raw, NULL),
	__ATTR(prox_detect, 0440, tmd3702_device_prox_detected, NULL),
	__ATTR(prox_gain, 0644, tmd3702_prox_gain_show, tmd3702_prox_gain_store),
	__ATTR(prox_offset, 0440, tmd3702_prox_offset_show,NULL),
	__ATTR(prox_persist, 0644, tmd3702_prox_persist_show, tmd3702_prox_persist_store),
	__ATTR(prox_regs, 0640, tmd3702_prox_regs_show, tmd3702_prox_regs_store),
	__ATTR(chip_name, 0440, tmd3702_prox_chip_id_show, NULL),
	__ATTR(enable, 0644, tmd3702_prox_enable_show, tmd3702_prox_enable_store),
	__ATTR(prox_value, 0440, tmd3702_prox_flush_show, NULL),
	__ATTR(prox_init, 0664, tmd3702_prox_dev_init_show, tmd3702_prox_dev_init_store),
	__ATTR(prox_offset_cal, 0644, tmd3702_prox_uncover_cal_show, tmd3702_prox_uncover_cal_store),
	__ATTR(prox_thres_min, 0440, tmd3702_prox_min_thres_show, NULL),
	__ATTR(prox_thres_max,0440, tmd3702_prox_max_thres_show, NULL),
	__ATTR(prox_data_max, 0440, tmd3702_prox_max_data_show, NULL),
	__ATTR(prox_thres, 0644, tmd3702_prox_thres_show, tmd3702_prox_thres_store),
	__ATTR(debug, 0644, tmd3702_prox_debug_show,  tmd3702_prox_debug_store),
	__ATTR(prox_debug, 0644, tmd3702_prox_raw_debug_show,  tmd3702_prox_raw_debug_store),
	__ATTR(prox_uncover_min, 0440, tmd3702_prox_uncover_data_min_show, NULL),
	__ATTR(prox_uncover_max, 0440, tmd3702_prox_uncover_data_max_show, NULL),
	__ATTR(prox_first_event, 0644, tmd3702_offset_first_event_show,  tmd3702_offset_first_event_store),


};
int tmd3702_prox_attrs_size = ARRAY_SIZE(tmd3702_prox_attrs);
/************************************************************************************/
/* proxmity  Functions  nubia add function list  */
//1.static int tmd3702_irq_handler_locked(struct tmd3702_chip *chip, u8 status)
//2.static int tmd3702_irq_handler_nubia(struct tmd3702_chip *chip)
//3.static irqreturn_t tmd3702_irq_nubia(int irq, void *handle)
//4.static int tmd3702_ps_input_device_init(struct tmd2725_chip *chip)
//5.static int tmd3702_prox_irq_init(struct tmd3702_chip *chip)
//6.tatic void tmd3702_ps_parameters_init(struct tmd2725_chip *chip)
//7.int tmd3702_ps_device_register(struct tmd2725_chip *chip, struct i2c_driver *driver)
//8.void tmd3702_ps_device_unregister(struct tmd2725_chip *chip)
//9.
//10.
/************************************************************************************/
static int tmd3702_irq_handler_locked(struct tmd3702_chip *chip, u8 status)
{
	int ret = 1;
//	struct i2c_client *client = chip->client;
//   u8 *sh = chip->shadow;
//	u8 offset_l,offset_h;
 //   u8 persist_l;

/************************************************************************************************/
/* ALS       */
/* Promblem Number: PR000     Author:xuxiaohua,   Date:2018/8/6
Description    : add has_als variable value  */

/************************************************************************************************/
      //  SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "TMD3702_ST_CAL_ chip->has_als:%d \n",chip->has_als);
        if (chip->has_als && (status & TMD3702_ST_ALS_IRQ))
        {
            SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "TMD3702_ST_ALS_IRQ\n");
            tmd3702_read_als(chip);
            tmd3702_report_als(chip);
            // Change the prox settings based on lux (indoor/outdoor
            // light conditions)
            //tmd3702_set_prox_mode(chip);
        }

/************************************************************************************************/
/* Calibration */
/************************************************************************************************/
        if (status & TMD3702_ST_CAL_IRQ)
        {
            SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "TMD3702_ST_CAL_IRQ\n");
            chip->amsCalComplete = true;

            /*
            ** Calibration has completed, no need for more
            **  calibration interrupts. These events are one-shots.
            **  next calibration start will re-enable.
            */

	        ams_i2c_modify(chip, chip->shadow, TMD3702_REG_INTENAB, TMD3702_CIEN, 0);
        }
        
/* Calibration IRQ */
 /****************************************************************************************/
/* Proximity */
/****************************************************************************************/
        if (status & TMD3702_ST_PRX_IRQ)
        {

	        SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "TMD3702_ST_PRX_IRQ\n");
            // Read Prox, determine detect/release, report results
            tmd3702_read_prox(chip);

	    /* Open debug mode, report raw data */
		    if (chip->prx_inf.prox_raw_debug) {
			    SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level,
    				"chip->prx_inf.raw = %d debug:%d offset_data_count:%d status:%d\n",chip->prx_inf.raw,
    			    chip->prx_inf.prox_raw_debug,	chip->pdata->offset_data_count,status);

                input_report_rel(chip->p_idev, REL_MISC, chip->prx_inf.raw);
                input_sync(chip->p_idev);
                //virtual_proximity_report_event(chip->prx_inf.raw, REL_MISC);
                    return ret;
    	    }   
            
        	tmd3702_get_prox(chip);             
            
        }


	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "exit\n");
	return ret;
}
static int tmd3702_irq_handler_nubia(struct tmd3702_chip *chip)
{
	u8 status = 0;
	int ret;
	int retry_times;
	int resched_times = 10;
	//u8 *sh = chip->shadow;
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "enter\n");
	if (chip->wakeup_from_suspend) {
	    SENSOR_LOG_INFO("wakeup from suspend\n");
	    mdelay(30);
	    chip->wakeup_from_suspend = false;
	}
resched:
	retry_times = 3;
	do {
	    ret = ams_i2c_read(chip->client,  TMD3702_REG_STATUS,  &chip->shadow[TMD3702_REG_STATUS]);
	    if (ret >= 0) {
	        break;
	    }
	    resched_times--;
	    mdelay(10);
	    SENSOR_LOG_ERROR("i2c read fail\n");
	} while (--retry_times);

	if (resched_times == 0) {
	    SENSOR_LOG_ERROR("exit irq handler\n");
	    return 1;
	}
	status = chip->shadow[TMD3702_REG_STATUS];
	if (status != 0) {
        /* Clear the interrupts we'll process */
        // ams_i2c_write(chip->client, sh, TMD2725_REG_STATUS, status);
        // Clear the interrupts we'll process
        ams_i2c_write_direct(chip->client, TMD3702_REG_STATUS, status);
        tmd3702_irq_handler_locked(chip, status);
	}

	if (!gpio_get_value(chip->irq_gpio)) {
	    SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "gpio val = %d\n", gpio_get_value(chip->irq_gpio));
	    goto resched;
	}
	/* we handled the interrupt */
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "exit\n");
	return 1;


}
static irqreturn_t tmd3702_irq_nubia(int irq, void *handle)
{
    struct tmd3702_chip *chip = handle;

    int ret;

    SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "enter\n");

    if (chip->in_suspend) {

        SENSOR_LOG_INFO("enter  in suspend\n");
        chip->irq_pending = 1;
        ret = 0;
        goto bypass;
    }

   //add code for nubia
    __pm_wakeup_event(&chip->ps_wlock, msecs_to_jiffies(100));

    ret = tmd3702_irq_handler_nubia(chip);
    SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "exit\n");
bypass:
    return ret ? IRQ_HANDLED : IRQ_NONE;
}

/*
static int tmd3702_ps_input_device_init(struct tmd3702_chip *chip)
{
	int ret;
	if (IS_ERR_OR_NULL(chip))
		return -ENODEV;

	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		SENSOR_LOG_ERROR("no memory for input_dev '%s'\n", chip->pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
#if 0
    chip->p_idev->name = pdata->prox_name;
    chip->p_idev->id.bustype = BUS_I2C;
    set_bit(EV_ABS, chip->p_idev->evbit);
    set_bit(ABS_DISTANCE, chip->p_idev->absbit);
    input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
    chip->p_idev->open = tmd3702_prox_idev_open;
    chip->p_idev->close = tmd3702_prox_idev_close;
    input_set_drvdata(chip->p_idev, chip);
    ret = input_register_device(chip->p_idev);
    if (ret) {
        dev_err(dev, "%s: cant register input '%s'\n",
                __func__, pdata->prox_name);
        goto input_p_alloc_failed;
    }
#endif

	chip->p_idev->name = chip->pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_REL, chip->p_idev->evbit);
	set_bit(REL_RZ,  chip->p_idev->relbit);
	set_bit(REL_MISC,  chip->p_idev->relbit);

	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		SENSOR_LOG_ERROR("cant register input '%s'\n", chip->pdata->prox_name);
		goto input_p_register_failed;
	}
	return 0;
input_p_register_failed:
	input_free_device(chip->p_idev);
input_p_alloc_failed:
	return ret;
}
*/
static int tmd3702_prox_irq_init(struct tmd3702_chip *chip)
{
	int ret;
	if (IS_ERR_OR_NULL(chip)) {
		SENSOR_LOG_ERROR("null exception\n");
		return -EINVAL;
	}
	chip->wake_irq = true;
	if (gpio_is_valid(chip->irq_gpio)) {
		/* configure pa22 irq gpio */
		SENSOR_LOG_INFO("gpio value is %d \n", gpio_get_value(chip->irq_gpio));
		ret = gpio_request_one(chip->irq_gpio,	GPIOF_DIR_IN,"tmd2725_irq_gpio");
		if (ret) {
			SENSOR_LOG_ERROR("unable to request gpio %d\n",chip->irq_gpio);
			ret = -ENODEV;
			goto exit;
		}
		chip->irq = chip->client->irq =	gpio_to_irq(chip->irq_gpio);
	} else {
		SENSOR_LOG_ERROR("irq gpio not provided\n");
	}
	SENSOR_LOG_INFO("init [gpio: %d][irq: %d] success\n", chip->irq_gpio, chip->client->irq);

	if (chip->wake_irq)
		irq_set_irq_wake(chip->client->irq, 1);

	ret = request_threaded_irq(chip->client->irq, NULL, &tmd3702_irq_nubia,IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			dev_name(chip->ps_dev), chip);
	if (ret) {
		SENSOR_LOG_ERROR("Failed to request irq %d\n", chip->client->irq);
		ret = -ENODEV;
		goto exit_request_irq;
	}
	sensor_irq_enable(chip, false, false);
	SENSOR_LOG_INFO("request threaded irq %d success\n", chip->client->irq);
	return 0;
exit_request_irq:
	if (chip->irq_gpio)
		gpio_free(chip->irq_gpio);
exit:
	return ret;
}
static void tmd3702_ps_parameters_init(struct tmd3702_chip *chip)
{
	mutex_init(&chip->ps_lock);
	wakeup_source_init(&chip->ps_wlock, "tmd3702");
	// add work
	INIT_WORK(&chip->ps_work, tmd3702_prox_thread);
	INIT_WORK(&chip->ps_irq_work, tmd3702_prox_irq_work);
	chip->prx_inf.last_detected = PROX_NONE;
	chip->prx_inf.detected = PROX_NONE;
	chip->irq_enabled = true;
	chip->wakeup_from_suspend = false;
	chip->pdata->debug_level = 0;
	chip->params.prox_thres_near = PS_THRES_NEAR;
	chip->params.prox_thres_far = PS_THRES_FAR;
	chip->params.prox_thres_oil_near= PS_THRES_OIL_NEAR;
	chip->params.prox_thres_oil_far= PS_THRES_OIL_FAR;
	SENSOR_LOG_INFO("prox_thres_near:%d prox_thres_far:%d \n",
        chip->params.prox_thres_near ,chip->params.prox_thres_far );
	SENSOR_LOG_INFO("prox_th_min:%d prox_th_max:%d \n",
        chip->params.prox_th_min,chip->params.prox_th_max);
}
int tmd3702_ps_device_register(struct tmd3702_chip *chip, struct i2c_driver *driver)
{
	int ret;
	tmd3702_ps_parameters_init(chip);

    /*
	ret = tmd3702_ps_input_device_init(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("intput device init fail\n");
		goto exit;
	}
	*/

	/* create workqueue */
	chip->ps_workqueue = create_singlethread_workqueue("tmd2725_irq_workqueue");
	if (IS_ERR_OR_NULL(chip->ps_workqueue)) {
		ret = -ENOMEM;
		SENSOR_LOG_ERROR( "cannot create work taos_work_queue, ret = %d",ret);
		goto exit_input_dev;
	}
	/* create sysfs */
	ps_class = class_create(THIS_MODULE, DEV_PS_NAME);
	alloc_chrdev_region(&tmd3702_ps_dev_t, 0, 1, DEV_PS_NAME);
	chip->ps_dev = device_create(ps_class, 0, tmd3702_ps_dev_t, driver, DEV_PS_NAME);
	if (IS_ERR_OR_NULL(chip->ps_dev)) {
		SENSOR_LOG_ERROR("ps device create fail\n");
		ret = -PTR_ERR(chip->ps_dev);
		goto exit_clear_workqueue;
	}
	ret = sensor_create_sysfs_interfaces(chip->ps_dev,tmd3702_prox_attrs, ARRAY_SIZE(tmd3702_prox_attrs));
	if (ret < 0) {
		goto exit_remove_device;
	}

	dev_set_drvdata(chip->ps_dev, chip);
	ret = tmd3702_prox_irq_init(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("irq init fail\n");
		goto exit_remove_sys_interfaces;
	}
	return 0;
exit_remove_sys_interfaces:
	sensor_remove_sysfs_interfaces(chip->ps_dev,
			tmd3702_prox_attrs, ARRAY_SIZE(tmd3702_prox_attrs));
exit_remove_device:
	device_destroy(ps_class, tmd3702_ps_dev_t);
	class_destroy(ps_class);
exit_clear_workqueue:
	destroy_workqueue(chip->ps_workqueue);
exit_input_dev:
	if (chip->p_idev){
		input_unregister_device(chip->p_idev);
		input_free_device(chip->p_idev);
	}
/*
exit:
	//add code for this release code
	//wake_lock_destroy(&chip->ps_wlock);
	wakeup_source_trash(&chip->ps_wlock);
	mutex_destroy(&chip->ps_lock);
*/
	return ret;


}

void tmd3702_ps_device_unregister(struct tmd3702_chip *chip)
{
	input_unregister_device(chip->p_idev);
	input_free_device(chip->p_idev);
	free_irq(chip->client->irq, chip->client);
	sensor_remove_sysfs_interfaces(chip->ps_dev,tmd3702_prox_attrs, ARRAY_SIZE(tmd3702_prox_attrs));
//	wake_lock_destroy(&chip->ps_wlock);
	wakeup_source_trash(&chip->ps_wlock);  //new function wakup
	mutex_destroy(&chip->ps_lock);
	free_irq(chip->irq, chip->client);
}

