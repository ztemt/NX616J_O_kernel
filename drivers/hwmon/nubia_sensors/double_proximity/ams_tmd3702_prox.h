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

#ifndef __AMS_TMD3702_PROX_H
#define __AMS_TMD3702_PROX_H

extern struct device_attribute tmd3702_prox_attrs [];
extern int tmd3702_prox_attrs_size;
#define PS_FAR_DISTANCE_EV              10
#define PS_NEAR_DISTANCE_EV             3

#define DEV_PS_NAME "proximity_front"
#define PS_UNCOVER_DATA_MIN             1
#define PS_UNCOVER_DATA_MAX             200
#define PS_THRESH_DATA_MIN              30
#define PS_THRESH_DATA_MAX              200
#define PS_DATA_MAX                     255
#define PS_CAL_FILE_PATH                "/persist/sensors/xtalk_cal"
#define PS_AVG_TIME                     6
#define PS_THRES_FAR                    50
#define PS_THRES_NEAR                   80
#define PS_THRES_OIL_NEAR               254
#define PS_THRES_OIL_FAR                150
#define PS_OFFSET_CAL_THRESH            240
#define PS_DEFAULT_THRES                PS_OFFSET_CAL_THRESH
extern void tmd3702_read_prox(struct tmd3702_chip *chip);
extern void tmd3702_get_prox(struct tmd3702_chip *chip);
extern void tmd3702_report_prox(struct tmd3702_chip *chip);
extern void tmd3702_set_prox_mode(struct tmd3702_chip *chip);
extern void tmd3702_init_prox_mode(struct tmd3702_chip *chip,bool first);
extern int tmd3702_configure_prox_mode(struct tmd3702_chip *chip, u8 state);
extern void tmd3702_prox_thread(struct work_struct *work);
extern void tmd3702_schedule_prox_work(struct tmd3702_chip *chip, enum tmd3702_prox_state prox_state);
extern int tmd3702_offset_calibration(struct tmd3702_chip *chip);

 #define MODULE_MANUFACTURE_NUMBER		3
extern  int tmd3702_ps_device_register(struct tmd3702_chip *chip, struct i2c_driver *driver);
extern void tmd3702_ps_device_unregister(struct tmd3702_chip *chip);
#endif /*__AMS_TMD3702_PROX_H */
