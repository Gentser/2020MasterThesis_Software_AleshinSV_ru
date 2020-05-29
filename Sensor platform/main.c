/* TWI sensor include */
#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "app_config.h"


#define APP_NO_ERROR  0  
#define SCALE_VALS    true               ///< Scale data measurements to display them in console

/* File naming convention */
#define FILENAME_LEN        11        ///< e.g. "XXX_123.EXT", because 512 files at the root maximum
#define FILENAME_INDEX_BASE "IDX_"
#define FILENAME_ADXL_BASE  "ADX_"
#define FILENAME_MPU_BASE   "MPU_"
#define FILENAME_TXT_EXT    ".TXT"    ///< used for INDEX file
#define FILENAME_BIN_EXT    ".BIN"    ///< used for MPU and ADXL files


#define ADXL_FIFO_LEN (335)   ///< (3*128) the ADXL FIFO can hold 512 samples maximum (1 sample takes 16 bits; X, Y, and Z samples always come in this order)
#define MPU_FIFO_LEN  (335)   ///< ~2/3 from max samples the MPU FIFO can hold 512 samples maximum (1 sample = 16 bits; {AcclX, AcclY, AcclZ, Temp, GyroX, GyroY, GyroZ} - samples always come in this order)


static uint16_t adxl_fifo[512] = {0};  // adxl fifo
static uint16_t mpu_fifo[512] = {0};   // MPU6050 fifo
static size_t adxl_num = 0;            // number of samples able to read from ADXL
static size_t mpu_num = 0;             // number of samples able to read from MPU6050

/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(GPIO_SDBOARD_DI, GPIO_SDBOARD_DO, GPIO_SDBOARD_CLK, GPIO_SDBOARD_CS)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);


static bool stop_program = false;     ///< an indicator of working system; false = system works, true = system stopped
static bool button_pressed = false;   ///< because NRF_GPIO_PIN_PULLUP is triggered to push and release events

/* IMU Data */
// raw not converted data
static int16_t rawAcclX = 0.0;
static int16_t rawAcclY = 0.0;
static int16_t rawAcclZ = 0.0;
static int16_t rawTcurr = 0.0;
static int16_t rawGyroX = 0.0;
static int16_t rawGyroY = 0.0;
static int16_t rawGyroZ = 0.0;
// converted data in human-readable format
static float acclX = 0.0;
static float acclY = 0.0;
static float acclZ = 0.0;
static float Tcurr = 0.0;
static float gyroX = 0.0;
static float gyroY = 0.0;
static float gyroZ = 0.0;

/* Scale parametres for sensor data */
static float k1_G, k2_G;
static float k1_A, k2_A;

/* Work with FATFS */
static FATFS fs;
static FIL fileMPU;                         ///< MPU sensor data (Temperature, Acceleration, Gyroskop)
static FIL fileADXL;                        ///< ADXL sensor data (Acceleration)
static FIL fileIndex;                       ///< timestamps of ADXL and MPU
static const int SD_CAPACITY = 1919;        ///< SD-Card capacity (in MB)

/* Work with index-file */
static int16_t LAST_MPU_SMPLS_NUMB  = 0;
static int16_t LAST_ADXL_SMPLS_NUMB = 0;
static float MPU_TIMESTAMP  = 0.0;
static float ADXL_TIMESTAMP = 0.0;

/* Application errors*/
static nrfx_sens_error_t sensors_error = {APP_NO_ERROR, APP_NO_ERROR, APP_NO_ERROR, 2, 4, 6};
enum nrfx_app_error_t app_error = APP_ERR_OK;

/* File naming convention */
static int app_filename_last_numb = -1;  ///< last number used for file naming


/* Function declaration */
void buttons_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);       ///< whole buttons handler
static void adxl362_handler(void* supplement);                                      ///< empty ADXL handler
static void clock_handler(nrfx_clock_evt_type_t event);                             ///< empty clock handler


static int find_filename_number(void);            ///< finds the next possible number for file naming convention
static int fatfs_init();                          ///< initialize FS 
static int fatfs_uninit();
static int name_and_open_files(int filename_num); ///< filename_num is the suitable number for naming convention
static void gpio_init(void);                      ///< initialize gpio
int  adxl_init(void);                             ///< initializing ADXL352 interface
void mpu_init(void);                              ///< initializing MPU6050 interface
void uninit_routine(void);                        ///< uninit MPU6050, ADXL362, errorblink, stopwatch and fatfs


void writeBin2file(void *buf, int size, FIL *file);
void write_toIndexFile(float timeStamp, 
              int32_t numbSamples, char dataType);  ///< write string data to index file
void read_data_bin(void);

static void check_init_errors();                            ///< check whether there were errors during sensors initialisation
static void blink_error(uint32_t led, uint8_t blink_numb);  ///< show one concrete error
static void led_turn_on(uint32_t led);                      
static void led_turn_off(uint32_t led);


/**
 * @brief Function for main application entry.
          
          The program consist of 3 parts: init, working loop, uninit.

          Button is used:
            - to stop program (goes from loop to uninit) 
            - and to restart program (goes from uninit to init)

          LED is used to report about working state and errors:
            - occured during the init step (look at app_config.h: nrfx_sens_error_t)
            - runtime errors (look at app_config.h: nrfx_app_error_t) 

          Used pins, LED and Buttons should be defined in app_config.h. 
          Pattern of init errors could be set through nrfx_sens_error_t.XXX_BLINK_N variables
          Frequency of LED blinking of runtime errors could be changed through setting different positive values of nrfx_app_error_t (value = period of blinking)

 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Pre-initialisation
    mpu_init();   // initialise MPU6050 before all of the sensors (it doesn't have real uninit function, just sleep mode)
    gpio_init();

    
label_app_start:    ///< Start and restart of app work go from this point

    while(stop_program)
        __WFE();

    printf("\n#########################################\n");
    printf("Multi sensor platform example started\n");
    nrf_delay_ms(2000);


    // LFCLK config - is needed for RTC
    nrfx_clock_init(clock_handler);
    nrfx_clock_lfclk_start();
    while (!nrfx_clock_lfclk_is_running());

    // Initialize RTC and start error whatching
    nrfx_stopwatch_init();
    nrfx_stopwatch_restart();   // restart the timer
    nrfx_errorblink_init();
    nrfx_errorblink_start();
    

    // Configuration and initialisation
    sensors_error.SENS_ERR_SDC = fatfs_init();
    if (!sensors_error.SENS_ERR_SDC)
    {
        // find last suitable number for naming convention
        app_filename_last_numb =  (app_filename_last_numb < 0) 
                                  ? find_filename_number() : (app_filename_last_numb + 1);
        name_and_open_files(app_filename_last_numb);  // create files for MPU, ADXL and Index data
    }

    sensors_error.SENS_ERR_ADXL = adxl_init();
    nrf_mpu6050_set_active(true);     // wake up MPU sensor


    // If there was no error, make some preparations
    if (sensors_error.SENS_ERR_SDC < 0 || sensors_error.SENS_ERR_ADXL < 0 || sensors_error.SENS_ERR_MPU < 0)
        stop_program = true;
    else
    {     
        // Write MPU6050 scale parameters
        nrf_mpu6050_get_scale_param(&k1_A, &k2_A, &k1_G, &k2_G);
        writeBin2file(&k1_A, sizeof(k1_A), &fileMPU);
        writeBin2file(&k2_A, sizeof(k2_A), &fileMPU);
        writeBin2file(&k1_G, sizeof(k1_G), &fileMPU);
        writeBin2file(&k2_G, sizeof(k2_G), &fileMPU);

        // Write ADXL362 scale parameters
        nrfx_adxl362_get_scale_param(&k1_A, &k2_A);
        writeBin2file(&k1_A, sizeof(k1_A), &fileADXL);
        writeBin2file(&k2_A, sizeof(k2_A), &fileADXL);

        printf("\n*********************************************************************");
        printf("\nMAIN LOOP");
        printf("\n*********************************************************************");
        printf("\n");
    }    
    

    while(!stop_program)
    {
        printf("\n\nTime: %.3fs\n", nrfx_stopwatch_readRTC());


        // - - - - - - - - - - - - 
        /* MPU6050 part */

        int16_t mpu_num = nrf_mpu6050_get_num_samples();
        printf("MPU6050: FIFO count = %d\n", mpu_num);
        if (mpu_num > MPU_FIFO_LEN)
        {
            nrf_mpu6050_get_samples_from_FIFO(mpu_fifo, mpu_num);
            MPU_TIMESTAMP = nrfx_stopwatch_readRTC();

            acclX = nrf_mpu6050_convert_accl(mpu_fifo[0]);
            acclY = nrf_mpu6050_convert_accl(mpu_fifo[1]);
            acclZ = nrf_mpu6050_convert_accl(mpu_fifo[2]);
            Tcurr = nrf_mpu6050_convert_temp(mpu_fifo[3]);
            gyroX = nrf_mpu6050_convert_gyro(mpu_fifo[4]);
            gyroY = nrf_mpu6050_convert_gyro(mpu_fifo[5]);
            gyroZ = nrf_mpu6050_convert_gyro(mpu_fifo[6]);
          

            // Update index file because of new MPU data
            LAST_MPU_SMPLS_NUMB = mpu_num/7;                           // as 1 sample consist of 7 2bytes measurements
            write_toIndexFile(MPU_TIMESTAMP, LAST_MPU_SMPLS_NUMB, 'M'); 
            writeBin2file(mpu_fifo, 2*mpu_num, &fileMPU);

            if(SCALE_VALS)
            {
                printf("Temperature,         grad C: %.2f\n", Tcurr);
                printf("Accel (ax, ay, az),       g: (%6.2f, %6.2f, %6.2f)\n", acclX, acclY, acclZ);
                printf("Gyro  (gx, gy, gz),  grad/s: (%6.2f, %6.2f, %6.2f)\n", gyroX, gyroY, gyroZ);
            }
        } 
        else
        {
            printf("MPU6050:Not enough FIFO elements\n");
        }

        

        // - - - - - - - - - - - - 
        /* ADXL362 part */

        nrfx_adxl362_get_num_samples(&adxl_num);
        ADXL_TIMESTAMP = nrfx_stopwatch_readRTC();
        printf("\nADXL samples ready: %d\n", adxl_num);

        if (adxl_num > ADXL_FIFO_LEN)
        {
            nrfx_adxl362_get_samples(adxl_fifo, adxl_num);

            // Update index file because of new ADXL data
            LAST_ADXL_SMPLS_NUMB = adxl_num/3;                            // as 1 sample consist of 3 2bytes measurements
            write_toIndexFile(ADXL_TIMESTAMP, LAST_ADXL_SMPLS_NUMB, 'A'); 
            writeBin2file(adxl_fifo, 2*adxl_num, &fileADXL);              // as each measurement consist of 16bit value
          
            float adxl_acclX = 0.0f, adxl_acclY = 0.0f, adxl_acclZ = 0.0f;
            adxl_acclX = nrfx_adxl362_convertVal(adxl_fifo[0]);
            adxl_acclY = nrfx_adxl362_convertVal(adxl_fifo[1]);
            adxl_acclZ = nrfx_adxl362_convertVal(adxl_fifo[2]);

            if (SCALE_VALS)
            {
                printf("FETCHED manually\n");
                printf("ADXL accl (ax, ay, az),   g: (%6.2f, %6.2f, %6.2f)\n", adxl_acclX, adxl_acclY, adxl_acclZ);
            }
        }
        

        printf("\n--------------------------------------------------\n");
        nrf_delay_ms(300);
    }



    // Uninit routine
    uninit_routine();  
    printf("\n\nTerminate programm. In case of problems LED will blink\n");

    
    check_init_errors();  // show possible errors
    NVIC_SystemReset();   // reset the device (for being able in case of no SD-Card just put it in and restart system by button push)
    goto label_app_start;


    return 0;
}

/* ########################################################################################## */


void uninit_routine(void)
{
    nrfx_adxl362_stop();
    nrfx_adxl362_uninit();

    nrf_mpu6050_set_active(false);    // toggle sleep mode
    nrf_mpu6050_reset_fifo();


    nrfx_errorblink_stop();
    nrfx_errorblink_uninit();
    nrfx_stopwatch_uninit();

    nrfx_clock_lfclk_stop();
    while (nrfx_clock_lfclk_is_running());
    nrfx_clock_uninit();


    (void) f_close(&fileMPU);
    (void) f_close(&fileIndex);
    (void) f_close(&fileADXL);

    int fs_uninit_res = fatfs_uninit();
    if (sensors_error.SENS_ERR_SDC == 0)
        sensors_error.SENS_ERR_SDC = fs_uninit_res;
}


void mpu_init()
{
    nrf_mpu6050_config_t config=
    {
      .pin_scl =      GPIO_MPU6050_SCL,
      .pin_sda =      GPIO_MPU6050_SDA,
      .pin_gnd =      GPIO_MPU6050_GND,
      .gyro_range =   MPU6050_DR_250_GRAD_S,
      .accl_range =   MPU6050_DR_8G,
      .data_rate =    MPU6050_ODR_50_HZ,
      .self_test =    false
    };

    sensors_error.SENS_ERR_MPU = nrf_mpu6050_init(&config); 
    nrf_mpu6050_set_active(false);      // toggle sleep mode
    nrf_mpu6050_reset_fifo();
}


static int find_filename_number(void)
{
    // FAT filesystem related variables
    DIR dir;
    FILINFO fno;

    // Vars to find the next suitable filename
    char foundFileName[FILENAME_LEN+1] = {0};
    uint16_t currNumb=0, maxNumb=0;

    // List dir and find next filename to use
    if (FR_OK != f_opendir(&dir, "/"))
    {
        printf("find_filename_number()# Directory listing failed\n");
        return -1;
    }
  
    do
    {
        if (FR_OK != f_readdir(&dir, &fno))
        {
            printf("find_filename_number()# Directory read failed\n");
            return -2;
        }
    
        // If file exists and found a file (not a dir)
        if (fno.fname[0] && !(fno.fattrib & AM_DIR))
        {      
            // Does filename fits template (length & starts with "IND_" & ends with ".TXT") 
            if ((strlen(fno.fname) == FILENAME_LEN) && 
                (strncmp(fno.fname, FILENAME_INDEX_BASE, strlen(FILENAME_INDEX_BASE)) == 0) && 
                (strncmp(&fno.fname[7], FILENAME_TXT_EXT, strlen(FILENAME_TXT_EXT)) == 0))
            {
                memcpy(foundFileName, fno.fname, sizeof(fno.fname));                    
                foundFileName[7] = 0;  // put the EOL in the dot place
      
                // Convert after "XXX_" and before '\0' to numb
                currNumb = (uint16_t) atoi((const char*) &foundFileName[4]);  
      
                // Look for the max number
                maxNumb = (currNumb > maxNumb) ? currNumb : maxNumb;
            }
        } 
    }
    while (fno.fname[0]);

    printf("find_filename_number()# proposed number: %d\n", maxNumb+1);

    return maxNumb + 1;
}


static void check_init_errors()
{
    // If there was an init error, endlessly show it
    if (sensors_error.SENS_ERR_SDC < 0 || sensors_error.SENS_ERR_ADXL < 0 || sensors_error.SENS_ERR_MPU < 0)
    {
        nrf_delay_ms(1000);
        while(stop_program)
        {
            // Check all types of init errors
            if (sensors_error.SENS_ERR_SDC != 0)
                blink_error(GPIO_LED1, sensors_error.SDC_BLINK_N);
            if (!stop_program) break;


            if (sensors_error.SENS_ERR_ADXL != 0)
                blink_error(GPIO_LED1, sensors_error.ADXL_BLINK_N);
            if (!stop_program) break;

            if (sensors_error.SENS_ERR_MPU != 0)
                blink_error(GPIO_LED1, sensors_error.MPU_BLINK_N);
            if (!stop_program) break;

            nrf_delay_ms(1000);
        }
    }
    else
    {
        led_turn_off(GPIO_LED1);
        printf("No errors occured\n\n\n");

        while (stop_program)
            __WFE();
    }
}

static void blink_error(uint32_t led, uint8_t blink_numb)
{
    for (int i=0; i < blink_numb; i++)
    {
        led_turn_on(led);
        nrf_delay_ms(100);
        led_turn_off(led);
        nrf_delay_ms(300);
    }
    nrf_delay_ms(1000);
}

static void led_turn_on(uint32_t led)
{
    // LED is put between 3V and pin, so to turn it on pin should be low (sink)
    nrf_gpio_pin_write(led, 0);   
}

                
static void led_turn_off(uint32_t led)
{
    // LED is put between 3V and pin, so to turn it on pin should be high
    nrf_gpio_pin_write(led, 1);   
}

static void clock_handler(nrfx_clock_evt_type_t event)   
{
    // Empty handler - for initialization
}

static void adxl362_handler(void* supplement)
{
  NRF_LOG_INFO("IRQ");
//  busy_wait();
}

// -------------------------------------------------------------------------------------------------------------------
int adxl_init(void)  // initializing ADXL352 interface
{
    uint8_t tmp;
    nrfx_err_t retVal= NRFX_SUCCESS;
    nrfx_adxl362_config_t config= 
    {
      .pin_scl=         GPIO_ADXL362_SCLK,   ///< GPIO used as SPI clock pin (output from Nordic)
      .pin_mosi=        GPIO_ADXL362_MOSI,   ///< GPIO used as SPI MOSI pin (output from Nordic)
      .pin_miso=        GPIO_ADXL362_MISO,   ///< GPIO used as SPI MISO pin (input to Nordic)
      .pin_cs=          GPIO_ADXL362_CS,   ///< GPIO used as SPI CS pin (output from Nordic)
      .pin_irq=         ADXL362_PIN_UNCONNECTED,  // NRF_GPIO_PIN_MAP(0,03),   ///< GPIO used as interrupt request pin (input to Nordic)
      .power_mode=      ADXL362_M_LOW_POWER,      ///< power vs. noise setting
      .range=           ADXL362_DR_8G,            ///< operating range of the sensor
      .data_rate=       ADXL362_ODR_100HZ,        ///< rate of data collection
      .anti_alias=      ADXL362_AA_ODR_QUARTER,   ///< anti-alias setting
      .fifo_watermark=  ADXL362_FIFOWM_85         ///< requested fill level of the FIFO upon interrupt signal
    };

    if ((NRFX_SUCCESS == nrfx_adxl362_init(&config, adxl362_handler)) && (NRFX_SUCCESS == nrfx_adxl362_get_revision(&tmp)))
    {
      printf("ADXL362 (rev. 0x%02X) detected\n", tmp);
      nrfx_adxl362_start();
    }
    else
    {
      printf("ADXL362 detection failed!\n");
      return -1;
    }

    return 0;
}

/* Write string of data to index file */
void write_toIndexFile(float timeStamp, int32_t numbSamples, char dataType)
{
    /*  Writes new string to Index file
        
        time - timestamp
        dataType - {'M', 'A'}, where M=MPU, A=ADXL
    */

    char time[21] = {0};
    char samples[17] = {0};
    char msg[52] = {0};

    if (dataType != 'M' && dataType != 'A')
    {
        printf("write_toIndex_file()# incorrect value of dataType\n");
        return;
    }
    
    snprintf(&msg, 3, "%c,", dataType);
    snprintf(&time, 21, "%.3f,", timeStamp);
    snprintf(&samples, 17, "%d\n", numbSamples); 

    strcat(msg, time);
    strcat(msg, samples);

    writeStr2file(&fileIndex, &msg);
}


void buttons_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == NRF_GPIO_PIN_PULLUP && !button_pressed)
    {
        switch(pin)
        {
            // Just switch between working mode and stop mode
            case GPIO_BUTTON1:
              stop_program = !stop_program;
            break;
        }
    }

    if (action == NRF_GPIO_PIN_PULLUP)
        button_pressed = !button_pressed;
}

// Init gpio
static void gpio_init(void)
{
    app_error = APP_ERR_GPIO;
    APP_ERROR_CHECK(nrf_drv_gpiote_init());

    // Configuration of GPIO pins as I/O
    nrf_gpio_cfg_output(GPIO_LED1);                 // LED as an Output
    nrf_gpio_cfg_input(GPIO_BUTTON1, NRF_GPIO_PIN_PULLUP);  // Button as an Input

    // Configure stop button handler (BUTTON 1)
    nrf_drv_gpiote_in_config_t config_1 = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    config_1.pull = NRF_GPIO_PIN_PULLUP;
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(GPIO_BUTTON1, &config_1, buttons_handler));

    // Enable these buttons
    nrf_drv_gpiote_in_event_enable(GPIO_BUTTON1, true);
    app_error = APP_ERR_OK;
}

/*  Functions for FATFS work  */

static int fatfs_init()
{
    // Configure SD-Card Board GND pin
    if (GPIO_SDBOARD_GND != NRFX_SPIM_PIN_NOT_USED)
    {
        nrf_gpio_cfg_output(GPIO_SDBOARD_GND);
        nrf_gpio_pin_clear(GPIO_SDBOARD_GND);
    }

    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;
    app_error = APP_ERR_SDC_INIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    printf("\nInitializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        printf("\nDisk initialization failed.\n");
        return -1;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    printf("\nCapacity: %d MB", capacity);

    printf("\nMounting volume...\n");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        printf("\nMount failed.\n");
        return -2;
    }

    app_error = APP_ERR_OK;

    return 0;
}


static int fatfs_uninit()
{
    FRESULT ff_result;
    DSTATUS disk_state;
    app_error = APP_ERR_SDC_INIT;

    ff_result = f_mount(NULL, "", 1);   // unmount fs
    if (ff_result)
    {
        printf("fatfs_uninit()# Unmount failed\n");
        return -1;
    }

    disk_state = disk_uninitialize(0);
    if (disk_state == 0)
    {
        printf("fatfs_uninit()# Disk uninitialization failed\n");
        return -2;
    }

    app_error = APP_ERR_OK;
    return 0;
}


static int name_and_open_files(int filename_num)
{
    /* Name the files */
    char filenameMPU[FILENAME_LEN+1]   = {0};
    char filenameADXL[FILENAME_LEN+1]  = {0};
    char filenameINDEX[FILENAME_LEN+1] = {0};

    printf("fatfs_init()# proposed file name number: %d\n", filename_num);

    sprintf(filenameMPU, "%s%03d%s", FILENAME_MPU_BASE, filename_num, FILENAME_BIN_EXT);
    sprintf(filenameADXL, "%s%03d%s", FILENAME_ADXL_BASE, filename_num, FILENAME_BIN_EXT);
    sprintf(filenameINDEX, "%s%03d%s", FILENAME_INDEX_BASE, filename_num, FILENAME_TXT_EXT);

    printf("fatfs_init()# proposed MPU file name: %s\n", filenameMPU);
    printf("fatfs_init()# proposed ADXL file name: %s\n", filenameADXL);
    printf("fatfs_init()# proposed INDEX file name: %s\n", filenameINDEX);


    /* Open files for MPU, ADXL and Index data */
    FRESULT ff_result;

    // Open MPU file
    ff_result = f_open(&fileMPU, filenameMPU, FA_WRITE | FA_READ | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        printf("\nfatfs_init()# Unable to open or create file 1: '%s'\n", filenameMPU);
        return -3;
    }

    // Open ADXL file
    ff_result = f_open(&fileADXL, filenameADXL, FA_WRITE | FA_READ | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        printf("\nfatfs_init()# Unable to open or create file 2: '%s'\n", filenameADXL);
        return -3;
    }

    // Open index-file
    ff_result = f_open(&fileIndex, filenameINDEX, FA_WRITE | FA_READ | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        printf("\nfatfs_init()# Unable to open or create file 3: '%s'\n", filenameINDEX);
        return -3;
    }
}



void writeStr2file(FIL *fd, char *msg)
{
    app_error = APP_ERR_SDC_RW;

    uint32_t bytes_written;
    FRESULT ff_result = f_write(fd, msg, strlen(msg), (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        printf("writeStr2file()# Write failed\n");
    }

    app_error = APP_ERR_OK;
}


void writeBin2file(void *buf, int size, FIL *file)
{
    app_error = APP_ERR_SDC_RW;

    uint32_t bytes_written;
    FRESULT ff_result = f_write(file, buf, size, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        printf("writeBin2file()# Write failed\n");
    }

    app_error = APP_ERR_OK;
}



void read_data_bin()
{
    /*  
        Format of data reading:
        <scaledVals>[<k1_A><k2_A><k1_G><k2_G>]
        ...
        <acclX><acclY><acclZ><temperature><gyroX><gyroY><gyroZ>
        ...

        bool scaledVals - are the written data in scaled or raw format
        float k1_A, k2_A, k1_G, k2_G - scale parametres if (scaledVals == false)
    */

    int step = 0;
    int tmpBytesRead;
    int bytesRead = 0;
    bool scaledVals = false;  // is the data in scaled or raw format
    
    // Move to the file beginning
    f_lseek(&fileMPU, 0);
    FRESULT ff_result;


    // Read, are the data in scaled or raw format
    ff_result = f_read(&fileMPU, &scaledVals, sizeof(scaledVals), &tmpBytesRead);
    if( ff_result != FR_OK )
        printf("read_data_bin()# read is failed\n");
    else
        bytesRead += tmpBytesRead;

    // HERE WE WORK JUST WITH RAW DATA
    scaledVals = false;

    // Read scale parametres 
    if(!scaledVals)
    {
        ff_result = f_read(&fileMPU, &k1_A, sizeof(k1_A), &tmpBytesRead);
        if( ff_result != FR_OK )
            printf("read_data_bin()# read is failed\n");
        else
            bytesRead += tmpBytesRead;
    
        ff_result = f_read(&fileMPU, &k2_A, sizeof(k2_A), &tmpBytesRead);
        if( ff_result != FR_OK )
            printf("read_data_bin()# read is failed\n");
        else
            bytesRead += tmpBytesRead;

        ff_result = f_read(&fileMPU, &k1_G, sizeof(k1_G), &tmpBytesRead);
        if( ff_result != FR_OK )
            printf("read_data_bin()# read is failed\n");
        else
            bytesRead += tmpBytesRead;
    
        ff_result = f_read(&fileMPU, &k2_G, sizeof(k2_G), &tmpBytesRead);
        if( ff_result != FR_OK )
            printf("read_data_bin()# read is failed\n");
        else
            bytesRead += tmpBytesRead;

        printf("read_data_bin()# Scale parametres were read\n");
    }
    
    // Main loop of data reading
    while (!f_eof(&fileMPU))
    {

        uint8_t dataUnscaled[14] = {0};
        float dataScaled[7] = {0.0f};
        ff_result = (!scaledVals) ? f_read(&fileMPU, &dataUnscaled[0], 14, &tmpBytesRead) 
                                  : f_read(&fileMPU, &dataScaled[0], 7*sizeof(float), &tmpBytesRead);
        if( ff_result != FR_OK )
            printf("read_data_bin()# read is failed\n");
        else
            bytesRead += tmpBytesRead;

        // Need to scale?
        if (!scaledVals)
        {
            // Read raw data
            rawAcclX = (int16_t) ((dataUnscaled[0] << 8)  | dataUnscaled[1]);
            rawAcclY = (int16_t) ((dataUnscaled[2] << 8)  | dataUnscaled[3]);
            rawAcclZ = (int16_t) ((dataUnscaled[4] << 8)  | dataUnscaled[5]);
            rawTcurr = (int16_t) ((dataUnscaled[6] << 8)  | dataUnscaled[7]);
            rawGyroX = (int16_t) ((dataUnscaled[8] << 8)  | dataUnscaled[9]);
            rawGyroY = (int16_t) ((dataUnscaled[10] << 8) | dataUnscaled[11]);
            rawGyroZ = (int16_t) ((dataUnscaled[12] << 8) | dataUnscaled[13]);

            // Scale temperature
            Tcurr = rawTcurr/340.0 + 36.53;

            // Scale acceleration
            acclX = (rawAcclX - k2_A)/k1_A;
            acclY = (rawAcclY - k2_A)/k1_A;
            acclZ = (rawAcclZ - k2_A)/k1_A;
    
            // Scale gyro
            gyroX = (rawGyroX - k2_G)/k1_G;
            gyroY = (rawGyroY - k2_G)/k1_G;
            gyroZ = (rawGyroZ - k2_G)/k1_G;
        }
        else
        {
            acclX = dataScaled[0];
            acclY = dataScaled[1];
            acclZ = dataScaled[2];
            Tcurr = dataScaled[3];
            gyroX = dataScaled[4];
            gyroY = dataScaled[5];
            gyroZ = dataScaled[6];
        }

        // Print result
        printf("read_data_bin()# data length got: %d\n", bytesRead);
        printf("read_data_bin()# sample %d\n", step);
        printf("read_data_bin()# Temperature, grad C: %.2f\n", Tcurr);
        printf("read_data_bin()# Acceleration,     g: (%.2f, %.2f, %.2f)\n", acclX, acclY, acclZ);
        printf("read_data_bin()# Gyroskop,    grad/s: (%.2f, %.2f, %.2f)\n\n", gyroX, gyroY, gyroZ);
        
        step++;
        nrf_delay_ms(15);
    }
}