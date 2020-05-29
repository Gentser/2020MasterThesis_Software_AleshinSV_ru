#ifndef _NRFX_MOBSENSPLATFORM_PINS_H_
#define _NRFX_MOBSENSPLATFORM_PINS_H_

#include "nrf_gpio.h"
#include "nrfx_spim.h"
#include "boards.h"


/*  Application pins   */

#ifdef BOARD_PCA10056
// Simple nRF board buildup

#define GPIO_MPU6050_SCL  NRF_GPIO_PIN_MAP(0, 2)
#define GPIO_MPU6050_SDA  NRF_GPIO_PIN_MAP(0,26)
#define GPIO_MPU6050_GND  NRFX_SPIM_PIN_NOT_USED  

#define GPIO_ADXL362_GND  NRFX_SPIM_PIN_NOT_USED
#define GPIO_ADXL362_SCLK NRF_GPIO_PIN_MAP(0, 4)
#define GPIO_ADXL362_MOSI NRF_GPIO_PIN_MAP(0,28)
#define GPIO_ADXL362_MISO NRF_GPIO_PIN_MAP(0,29)
#define GPIO_ADXL362_CS   NRF_GPIO_PIN_MAP(0,30)

#define GPIO_PDM_VDD      NRFX_SPIM_PIN_NOT_USED
#define GPIO_PDM_SEL      NRFX_SPIM_PIN_NOT_USED
#define GPIO_PDM_CLK      NRF_GPIO_PIN_MAP(0, 2)
#define GPIO_PDM_DAT      NRF_GPIO_PIN_MAP(0,26)

#define GPIO_SDBOARD_CS   NRF_GPIO_PIN_MAP(1,11)
#define GPIO_SDBOARD_DI   NRF_GPIO_PIN_MAP(1,12)
#define GPIO_SDBOARD_DO   NRF_GPIO_PIN_MAP(1,13)
#define GPIO_SDBOARD_CLK  NRF_GPIO_PIN_MAP(1,14)
#define GPIO_SDBOARD_GND  NRFX_SPIM_PIN_NOT_USED

#define GPIO_BUTTON1      BUTTON_1
#define GPIO_LED1         LED_2
// endif BOARD_PCA10056 

#elif defined BOARD_PCA10059 
// Small nRF Dongle buildup

#define GPIO_MPU6050_SCL  NRF_GPIO_PIN_MAP(0,29)
#define GPIO_MPU6050_SDA  NRF_GPIO_PIN_MAP(0,31)
#define GPIO_MPU6050_GND  NRFX_SPIM_PIN_NOT_USED

#define GPIO_ADXL362_GND  NRFX_SPIM_PIN_NOT_USED
#define GPIO_ADXL362_SCLK NRF_GPIO_PIN_MAP(0,17)
#define GPIO_ADXL362_MOSI NRF_GPIO_PIN_MAP(0,15)
#define GPIO_ADXL362_MISO NRF_GPIO_PIN_MAP(0,13)
#define GPIO_ADXL362_CS   NRF_GPIO_PIN_MAP(0, 4)

#define GPIO_PDM_VDD      NRFX_SPIM_PIN_NOT_USED
#define GPIO_PDM_SEL      NRFX_SPIM_PIN_NOT_USED
#define GPIO_PDM_CLK      NRF_GPIO_PIN_MAP(1, 0)
#define GPIO_PDM_DAT      NRF_GPIO_PIN_MAP(0,14)

#define GPIO_SDBOARD_CS   NRF_GPIO_PIN_MAP(0,11)
#define GPIO_SDBOARD_DI   NRF_GPIO_PIN_MAP(0,24)
#define GPIO_SDBOARD_DO   NRF_GPIO_PIN_MAP(0,22)
#define GPIO_SDBOARD_CLK  NRF_GPIO_PIN_MAP(0,20)
#define GPIO_SDBOARD_GND  NRFX_SPIM_PIN_NOT_USED

#define GPIO_BUTTON1      NRF_GPIO_PIN_MAP(1,6)
#define GPIO_LED1         NRF_GPIO_PIN_MAP(1,9)

#endif  // BOARD_PCA10059 


/*   Types of errors during app running   */
// Number is describing the period T of blinking
/*  Figure 1: blinking scheme

    F = blinking period
               ______
              |      |       
        ______|      | ... 
        0    T/2     T  
*/ 
enum nrfx_app_error_t
{
    APP_ERR_OK          = 0,      //< no error
    APP_ERR_TWI_INIT    = 2,      //< error during TWI init
    APP_ERR_TWI_RW      = 3,      //< error during MPU command
    APP_ERR_SPI_INIT    = 4,      //< error during SPI init
    APP_ERR_SPI_RW      = 5,      //< error during ADXL command
    APP_ERR_GPIO        = 10,      //< error during GPIO init
    APP_ERR_SDC_INIT    = 11,      //< error during SD Card init
    APP_ERR_SDC_RW      = 12     //< error during SD Card read/write operation
};


/*   Types of errors during sensor initialising   */
typedef struct
{
    int8_t SENS_ERR_SDC;
    int8_t SENS_ERR_PDM;
    int8_t SENS_ERR_ADXL;
    int8_t SENS_ERR_MPU;

    // Number of blinking, describing the error
    const uint8_t SDC_BLINK_N;
    const uint8_t PDM_BLINK_N;
    const uint8_t ADXL_BLINK_N;
    const uint8_t MPU_BLINK_N;
} nrfx_sens_error_t;




#endif // _NRFX_MOBSENSPLATFORM_PINS_H_