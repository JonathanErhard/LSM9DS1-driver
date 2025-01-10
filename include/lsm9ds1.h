#pragma once
#include "rodos.h"
#include "hal/hal_i2c.h"

namespace IMU
{
    class LMS9DS1 : protected RODOS::HAL_I2C
    {
        uint32_t speed;

    public:
        /**
         * @brief default calibration
         * use this <https://github.com/JonathanErhard/LSM9DS1-calibration>
         * repository to get your calibration values :)
         */

        int16_t GYR_CALIB_VALS[3] = {93, 27, 30};                               // offset
        int16_t ACC_CALIB_VALS[3] = {-244, 0, 0};                               // offset
        int16_t MAG_BOUNDRIES[3][2] = {{-545, 523}, {-767, 761}, {-1044, 449}}; //{x[min,max], y[min,max], z[min,max]}

        /**
         * @brief databuffers for HAL_I2C.writeRead
         */
        uint8_t DATA_L[1];
        uint8_t DATA_H[1];

        /**
         * @brief register adresses
         * lsm9ds1.pdf p.50,56,66
         */

        // GYRO
        uint8_t GYR_X_L[1] = {0x18};
        uint8_t GYR_X_H[1] = {0x19};
        uint8_t GYR_Y_L[1] = {0x1A};
        uint8_t GYR_Y_H[1] = {0x1B};
        uint8_t GYR_Z_L[1] = {0x1C};
        uint8_t GYR_Z_H[1] = {0x1D};

        // ACCELLEROMETER
        uint8_t ACC_X_L[1] = {0x28};
        uint8_t ACC_X_H[1] = {0x29};
        uint8_t ACC_Y_L[1] = {0x2A};
        uint8_t ACC_Y_H[1] = {0x2B};
        uint8_t ACC_Z_L[1] = {0x2C};
        uint8_t ACC_Z_H[1] = {0x2D};

        // MAGNETOMETER
        uint8_t MAG_X_L[1] = {0x28};
        uint8_t MAG_X_H[1] = {0x29};
        uint8_t MAG_Y_L[1] = {0x2A};
        uint8_t MAG_Y_H[1] = {0x2B};
        uint8_t MAG_Z_L[1] = {0x2C};
        uint8_t MAG_Z_H[1] = {0x2D};

        /**
         * @brief Data Buffers
         *
         */

        // raw (ignoring scaling/calibration)
        int16_t GYR_RAW_VALS[3] = {0, 0, 0};
        int16_t ACC_RAW_VALS[3] = {0, 0, 0};
        int16_t MAG_RAW_VALS[3] = {0, 0, 0};

        // adjusted
        float GYR_ADJ_VALS[3];
        float ACC_ADJ_VALS[3];
        float MAG_ADJ_VALS[3];

        LMS9DS1(RODOS::I2C_IDX idx, int16_t GYR_CALIB_VALS[3], int16_t ACC_CALIB_VALS[3], int16_t MAG_BOUNDRIES[3][2]);

        LMS9DS1(RODOS::I2C_IDX idx);
        // constructor for default values we used in the floatsat to avoid copy pasting the calib values 24/7
        LMS9DS1();

        int32_t init(uint32_t speed);

        // raw for calibration, adj for normal use
        void read_raw();
        void read_adj();

        // copies values
        void cpy_raw(float *ACC_RAW_VALS, float *GYR_RAW_VALS, float *MAG_RAW_VALS);
        void cpy_adj(float *ACC_ADJ_VALS, float *GYR_ADJ_VALS, float *MAG_ADJ_VALS);

        // mostly debug prints
        void print_real();
        void print_raw();
        void print_raw_bits();
    };
}