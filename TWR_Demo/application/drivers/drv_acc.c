/******************************************************************************
 * @file    drv_acc.c
 * @author  Insight SiP
 * @brief   accelerometer driver implementation file.
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "drv_acc.h"
#include "app_scheduler.h"
#include "drv_lis2de12.h"
#include "nrf_delay.h"
#include "sdk_macros.h"
#include <nrf_drv_gpiote.h>

#include "nrf_log.h"

#define RETURN_IF_INV_ERROR(PARAM) \
    if ((PARAM) != INV_SUCCESS) {  \
        return NRF_ERROR_INTERNAL; \
    }

#define ACC_SCALE_2G 0.015625f
#define ACC_SCALE_4G 0.031250f
#define ACC_SCALE_8G 0.062500f
#define ACC_SCALE_16G 0.125000f

/**@brief Motion configuration struct.
 */
typedef struct
{
    bool enabled;               ///< Driver enabled.
    drv_lis2de12_twi_cfg_t cfg; ///< TWI configuraion.
    drv_acc_evt_handler_t evt_handler;
    uint8_t full_scale;
} drv_acc_t;

/**@brief configuration.
 */
static drv_acc_t m_drv_acc;
static drv_acc_evt_t evt;

/**@brief GPIOTE sceduled handler, executed in main-context.
 */
static void gpiote_evt_sceduled(void *p_event_data, uint16_t event_size) {

    m_drv_acc.evt_handler((drv_acc_evt_t*)p_event_data);
}

/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    uint32_t err_code;

    if (pin == m_drv_acc.cfg.pin_int1) {
        evt.type= DRV_ACC_EVT_1;
        err_code = app_sched_event_put(&evt, sizeof(evt), gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
    if (pin == m_drv_acc.cfg.pin_int2) {
        evt.type= DRV_ACC_EVT_2;
        err_code = app_sched_event_put(&evt, sizeof(evt), gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
}

uint32_t drv_acc_init(drv_acc_init_t *p_params) {
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_instance);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_cfg);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler);

    m_drv_acc.evt_handler = p_params->evt_handler;
    m_drv_acc.cfg.twi_addr = p_params->twi_addr;
    m_drv_acc.cfg.pin_int1 = p_params->pin_int1;
    m_drv_acc.cfg.pin_int2 = p_params->pin_int2;
    m_drv_acc.cfg.p_twi_instance = p_params->p_twi_instance;
    m_drv_acc.cfg.p_twi_cfg = p_params->p_twi_cfg;
    m_drv_acc.enabled = false;

    // Configure interrupts
    if ((m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED) || (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED)) {
        if (!nrfx_gpiote_is_init()) {
            err_code = nrfx_gpiote_init();
            VERIFY_SUCCESS(err_code);
        }
    }
    if (m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED) {
        nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
        in_config.pull = NRF_GPIO_PIN_PULLDOWN;
        err_code = nrf_drv_gpiote_in_init(m_drv_acc.cfg.pin_int1, &in_config, gpiote_evt_handler);
        VERIFY_SUCCESS(err_code);
    }
    if (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED) {
        nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
        in_config.pull = NRF_GPIO_PIN_PULLDOWN;
        err_code = nrf_drv_gpiote_in_init(m_drv_acc.cfg.pin_int2, &in_config, gpiote_evt_handler);
        VERIFY_SUCCESS(err_code);
    }

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_verify();
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_reboot();
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t drv_acc_enable(uint8_t full_scale, uint8_t output_data_rate) {
    uint32_t err_code;
        uint8_t tmp;

    if (m_drv_acc.enabled) {
        return NRF_SUCCESS;
    }
    m_drv_acc.enabled = true;

    m_drv_acc.full_scale = full_scale;

    // Enable interrupt pins
    if (m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_enable(m_drv_acc.cfg.pin_int1, true);
    if (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_enable(m_drv_acc.cfg.pin_int2, true);

    const drv_lis2de12_cfg_t lis2de12_cfg =
        {
            .reg_vals =
             {
                    .ctrl_reg1 = BITS_XEN | BITS_YEN | BITS_ZEN | BITS_LPEN | output_data_rate,
                    .ctrl_reg4 = full_scale | BITS_BDU

             },       
            .reg_selects = {
                    .ctrl_reg1 = true,
                    .ctrl_reg2 = false,
                    .ctrl_reg3 = false,
                    .ctrl_reg4 = true,
                    .ctrl_reg5 = false,
                    .ctrl_reg6 = false,
                    .temp_cfg_reg = false,
                    .fifo_ctrl_reg = false,
                    .int1_cfg = false,
                    .int2_cfg = false,
                    .click_cfg = false}};

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_cfg_set(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);

    // read register to remove previous interrupts
    err_code = reg_read(INT1_CFG, &tmp);
    VERIFY_SUCCESS(err_code);
    NRF_LOG_INFO("INT1_SRC(0x%x): 0x%x",INT1_SRC, tmp);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_acc_check_reg(void) {
    uint32_t err_code;
    uint8_t threshold;
    uint8_t duration;

    drv_lis2de12_cfg_t lis2de12_cfg =
        {
    
            .reg_selects = {
                    .ctrl_reg1 = true,
                    .ctrl_reg2 = true,
                    .ctrl_reg3 = true,
                    .ctrl_reg4 = true,
                    .ctrl_reg5 = true,
                    .ctrl_reg6 = true,
                    .temp_cfg_reg = true,
                    .fifo_ctrl_reg = true,
                    .int1_cfg = true,
                    .int2_cfg = true,
                    .click_cfg = true}};

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code =  drv_lis2de12_get_threshold_and_duratio_wakeup(&threshold,&duration);
    VERIFY_SUCCESS(err_code);
  

    err_code = drv_lis2de12_cfg_get(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


static uint8_t convert_to_register_value(uint8_t full_scale,uint32_t threshold){
  
  uint32_t divide_factor;
  // the values are defined in the datasheet of lis2dh
  switch (full_scale) {
        case BITS_FS_2G:
          divide_factor = 16;
          break;
    case BITS_FS_4G:
          divide_factor = 32;
          break;
    case BITS_FS_8G:
          divide_factor = 62;
          break;
    case BITS_FS_16G:
          divide_factor = 186;
          break;
    defualt:
        // this is an invalid value
        VERIFY_SUCCESS(NRF_ERROR_INVALID_PARAM);
  }

  return (uint8_t)(threshold/divide_factor);
}

uint32_t drv_acc_interrupt_enable_ia1(uint32_t threshold,uint8_t duration) {
    uint32_t err_code;


    const drv_lis2de12_cfg_t lis2de12_cfg =
        {
            .reg_vals =
                {
                    // AOI1 interrupt generation is routed to INT1 pin.
                    .ctrl_reg3 = BITS_I1_IA1,
                    // letch the output till the value is not read from the INT_SRC register
                    .ctrl_reg5 = BITS_LIR_INT1,
                    // enable the wake-up events on the x,y,z only when there are positive
                    .int1_cfg  =   BITS_YHIE/*| BITS_XHIE | BITS_ZHIE*/ 

                },
            .reg_selects =
                {
                    .ctrl_reg1 = false,
                    .ctrl_reg2 = false,
                    .ctrl_reg3 = true,
                    .ctrl_reg4 = false,
                    .ctrl_reg5 = true,
                    .ctrl_reg6 = false,
                    .temp_cfg_reg = false,
                    .fifo_ctrl_reg = false,
                    .int1_cfg = true,
                    .int2_cfg = false,
                    .click_cfg = false}};

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    // eanble duration and threshold value
    uint8_t threshold_8bits = convert_to_register_value(m_drv_acc.full_scale, threshold);
    err_code =  drv_lis2de12_set_threshold_and_duratio_wakeup(threshold_8bits,duration);
    VERIFY_SUCCESS(err_code);
  
    err_code = drv_lis2de12_cfg_set(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_read_interrupt_ia1(void){

  int err_code;
  uint8_t tmp;

  err_code = drv_lis2de12_open(&m_drv_acc.cfg);
  VERIFY_SUCCESS(err_code);

  err_code = reg_read(INT1_SRC, &tmp);
  VERIFY_SUCCESS(err_code);

  // Release twi bus
  err_code = drv_lis2de12_close();
  VERIFY_SUCCESS(err_code);

  NRF_LOG_INFO("INT1_SRC(0x%x): 0x%x",INT1_SRC, tmp);

  return NRF_SUCCESS;
}


uint32_t drv_acc_disable(void) {
    uint32_t err_code = NRF_SUCCESS;

    m_drv_acc.enabled = false;

    // Disable interrupt pins
    if (m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_disable(m_drv_acc.cfg.pin_int1);
    if (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_disable(m_drv_acc.cfg.pin_int2);

    const drv_lis2de12_cfg_t lis2de12_cfg =
        {
            .reg_vals =
                {
                    .ctrl_reg1 = CTRL_REG1_DEFAULT},
            .reg_selects =
                {
                    .ctrl_reg1 = true,
                    .ctrl_reg2 = false,
                    .ctrl_reg3 = false,
                    .ctrl_reg4 = false,
                    .ctrl_reg5 = false,
                    .ctrl_reg6 = false,
                    .temp_cfg_reg = false,
                    .fifo_ctrl_reg = false,
                    .int1_cfg = false,
                    .int2_cfg = false,
                    .click_cfg = false}};

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_cfg_set(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}


//-----------------------------------------------------------------------------------------------------------------------------------------
// getting the data

uint32_t drv_acc_get(float *p_acc) {
    uint32_t err_code = NRF_SUCCESS;
    int8_t raw_val[3];

    VERIFY_PARAM_NOT_NULL(p_acc);

    // Request twi bus
    drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_acceleration_get(raw_val);
    VERIFY_SUCCESS(err_code);

    float acc_scale = drv_get_scale_factor();  
    

    p_acc[0] = (float)(raw_val[0]) * acc_scale;
    p_acc[1] = (float)(raw_val[1]) * acc_scale;
    p_acc[2] = (float)(raw_val[2]) * acc_scale;

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}

/**@brief ST Implementation of the driver
    code example: https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lis2de12_STdC/examples/lis2de12_read_data_polling.c
    driver implmentation: https://github.com/STMicroelectronics/lis2de12-pid/blob/f66fd6f0c3270b790c42b35f762ac0326a77c832/lis2de12_reg.c
 */


uint32_t drv_acc_get2(float_t *p_acc) {
    uint32_t err_code = NRF_SUCCESS;
    uint16_t raw_val[6];

    VERIFY_PARAM_NOT_NULL(p_acc);

    // Request twi bus
    drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_acceleration_get2(raw_val);
    VERIFY_SUCCESS(err_code);

    /*
    The idea of the implementation on the line 168: 
     https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lis2de12_STdC/examples/lis2de12_read_data_polling.c#L168C7-L168C36
    */
    p_acc[0] = lis2de12_from_fs2_to_mg(raw_val[0]);
    p_acc[1] = lis2de12_from_fs2_to_mg(raw_val[1]);
    p_acc[2] = lis2de12_from_fs2_to_mg(raw_val[2]);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}


/**@brief Zepyhr Implementation of the driver
    https://elixir.bootlin.com/zephyr/latest/source/drivers/sensor/lis2dh/lis2dh.c
 */


#define LIS2DH_BUF_SZ			6
#define sys_le16_to_cpu(x) ((uint16_t) ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8)))

#define SENSOR_G		9806650LL

#define ACCEL_SCALE(sensitivity)			\
	((SENSOR_G * (sensitivity) >> 14) / 100)

static uint32_t lis2dh_reg_val_to_scale[] = {
	ACCEL_SCALE(1600),
	ACCEL_SCALE(3200),
	ACCEL_SCALE(6400),
	ACCEL_SCALE(19200),
};

union lis2dh_sample {
	uint8_t raw[LIS2DH_BUF_SZ];
	struct {
		int16_t xyz[3];
	};
};

uint32_t drv_acc_get3(double *p_acc) {
    uint32_t err_code = NRF_SUCCESS;
    static union lis2dh_sample lis2dh; 
    VERIFY_PARAM_NOT_NULL(p_acc);

    // Request twi bus
    drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    /*
    The idea of the implementation at line 131 and: 
    https://elixir.bootlin.com/zephyr/latest/source/drivers/sensor/lis2dh/lis2dh.c#L131
    */
    err_code = drv_lis2de12_acceleration_get3(lis2dh.raw);
    VERIFY_SUCCESS(err_code);

    for (size_t i = 0; i < (3 * sizeof(int16_t)); i += sizeof(int16_t)) {
        int16_t *sample =
                (int16_t *)&lis2dh.raw[1 + i];

        *sample = sys_le16_to_cpu(*sample);
    }

    /*
    The idea of the implementation at line 124 and: 
    https://elixir.bootlin.com/zephyr/latest/source/drivers/sensor/lis2dh/lis2dh.c#L124
    */

    lis2dh_convert(lis2dh.xyz[0], lis2dh_reg_val_to_scale[0], p_acc);
    lis2dh_convert(lis2dh.xyz[1], lis2dh_reg_val_to_scale[1], p_acc);
    lis2dh_convert(lis2dh.xyz[2], lis2dh_reg_val_to_scale[2], p_acc);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}

float drv_get_scale_factor(void){
  switch (m_drv_acc.full_scale) {

    case BITS_FS_2G:
      return ACC_SCALE_2G;
    case BITS_FS_4G:
      return ACC_SCALE_4G;
    case BITS_FS_8G:
      return ACC_SCALE_8G;
    case BITS_FS_16G:
      return ACC_SCALE_16G;
    defualt:
        // this is an invalid value
        VERIFY_SUCCESS(NRF_ERROR_INVALID_PARAM);
  }
}
