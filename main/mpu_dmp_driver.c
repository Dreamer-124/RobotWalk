/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      motion_driver_test.c
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// #include "USB_eMPL/descriptors.h"

// #include "USB_API/USB_Common/device.h"
// #include "USB_API/USB_Common/types.h"
// #include "USB_API/USB_Common/usb.h"

// #include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
// #include "F5xx_F6xx_Core_Lib/HAL_PMM.h"
// #include "F5xx_F6xx_Core_Lib/HAL_FLASH.h"

// #include "USB_API/USB_CDC_API/UsbCdc.h"
// #include "usbConstructs.h"

// #include "msp430.h"
// #include "msp430_clock.h"
// #include "msp430_i2c.h"
// #include "msp430_interrupt.h" 

#include "esp_system.h"
#include "mpu_dmp_driver.h"
#include "empl_driver.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100) // 设置MPU6050的采样率

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

// enum packet_type_e {
//     PACKET_TYPE_ACCEL,
//     PACKET_TYPE_GYRO,
//     PACKET_TYPE_QUAT,
//     PACKET_TYPE_TAP,
//     PACKET_TYPE_ANDROID_ORIENT,
//     PACKET_TYPE_PEDO,
//     PACKET_TYPE_MISC
// };

/* Send data to the Python client application.
 * Data is formatted as follows:
 * packet[0]    = $
 * packet[1]    = packet type (see packet_type_e)
 * packet[2+]   = data
 */
// void send_packet(char packet_type, void *data)
// {
// #define MAX_BUF_LENGTH  (18)
//     char buf[MAX_BUF_LENGTH], length;

//     memset(buf, 0, MAX_BUF_LENGTH);
//     buf[0] = '$';
//     buf[1] = packet_type;

//     if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO) {
//         short *sdata = (short*)data;
//         buf[2] = (char)(sdata[0] >> 8);
//         buf[3] = (char)sdata[0];
//         buf[4] = (char)(sdata[1] >> 8);
//         buf[5] = (char)sdata[1];
//         buf[6] = (char)(sdata[2] >> 8);
//         buf[7] = (char)sdata[2];
//         length = 8;
//     } else if (packet_type == PACKET_TYPE_QUAT) {
//         long *ldata = (long*)data;
//         buf[2] = (char)(ldata[0] >> 24);
//         buf[3] = (char)(ldata[0] >> 16);
//         buf[4] = (char)(ldata[0] >> 8);
//         buf[5] = (char)ldata[0];
//         buf[6] = (char)(ldata[1] >> 24);
//         buf[7] = (char)(ldata[1] >> 16);
//         buf[8] = (char)(ldata[1] >> 8);
//         buf[9] = (char)ldata[1];
//         buf[10] = (char)(ldata[2] >> 24);
//         buf[11] = (char)(ldata[2] >> 16);
//         buf[12] = (char)(ldata[2] >> 8);
//         buf[13] = (char)ldata[2];
//         buf[14] = (char)(ldata[3] >> 24);
//         buf[15] = (char)(ldata[3] >> 16);
//         buf[16] = (char)(ldata[3] >> 8);
//         buf[17] = (char)ldata[3];
//         length = 18;
//     } else if (packet_type == PACKET_TYPE_TAP) {
//         buf[2] = ((char*)data)[0];
//         buf[3] = ((char*)data)[1];
//         length = 4;
//     } else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
//         buf[2] = ((char*)data)[0];
//         length = 3;
//     } else if (packet_type == PACKET_TYPE_PEDO) {
//         long *ldata = (long*)data;
//         buf[2] = (char)(ldata[0] >> 24);
//         buf[3] = (char)(ldata[0] >> 16);
//         buf[4] = (char)(ldata[0] >> 8);
//         buf[5] = (char)ldata[0];
//         buf[6] = (char)(ldata[1] >> 24);
//         buf[7] = (char)(ldata[1] >> 16);
//         buf[8] = (char)(ldata[1] >> 8);
//         buf[9] = (char)ldata[1];
//         length = 10;
//     } else if (packet_type == PACKET_TYPE_MISC) {
//         buf[2] = ((char*)data)[0];
//         buf[3] = ((char*)data)[1];
//         buf[4] = ((char*)data)[2];
//         buf[5] = ((char*)data)[3];
//         length = 6;
//     }
//     cdcSendDataWaitTilDone((BYTE*)buf, length, CDC0_INTFNUM, 100);
// }

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}


// 重启系统  
void system_reset(void)
{
    esp_restart();
}


static inline void run_self_test(void)
{
    int result;
    // char test_packet[4] = {0};
    long gyro[3], accel[3];
    unsigned char i = 0;

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x3) { //六轴不是九轴
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
    }
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

uint8_t mpu_init_i2c(void)
{
    esp_err_t esp_err;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU_I2C_SDA,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MPU_I2C_SCL,         // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 200000,  // select frequency specific to your project
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err = i2c_param_config(0, &conf);
    printf("i2c_param_config: %d \n", esp_err);

    esp_err = i2c_driver_install(0, I2C_MODE_MASTER, 0, 0, 0);
    printf("i2c_driver_install: %d \n", esp_err);

    return esp_err;
}


// 初始化mpu6050
uint8_t mpu_dmp_init(void)
{
    int result;  

    if (mpu_init_i2c() != 0)
        return 1;

    result = mpu_init();
    printf("mpu_init: %d\n", result);

    if (result){
        // msp430_reset();
        system_reset();
        return 1;
    };
    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
    {
        return 2;
    }

    /* Push both gyro and accel data into the FIFO. */
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
    {
        return 3;
    }
    if (mpu_set_sample_rate(DEFAULT_MPU_HZ))
    {
        return 4;
    }

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;

    if (dmp_load_motion_driver_firmware())
    {
        return 5;
    }

    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
    {
        return 6;
    }

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;

    if (dmp_enable_feature(hal.dmp_features))
    {
        return 7;
    }

    if (dmp_set_fifo_rate(DEFAULT_MPU_HZ))
    {
        return 8;
    }

    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    return 0;
}

unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4]; 


#define q30 1073741824.0f
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float pitch, roll, yaw;

// 获取数据，给外部调用 
uint8_t dmp_get_data(void)
{
    if (hal.new_gyro && hal.dmp_on) {
        if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
        {
            return 1;
        }
        if (!more)
            hal.new_gyro = 0;
        
        if (sensors & INV_WXYZ_QUAT)
        {
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;

            pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
            roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)*57.3;
            yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 -q3 * q3)*57.3;
        }
    }
    return 0;
}
