#include "../include/lsm9ds1.h" // I would have to modify the cmakelist and I cant be bothered to

namespace IMU
{
#define AccGyrADDR 0x6B // Register to activate acc and gyro (lsm9ds1.pdf p.42)
#define MagADDR 0x1E    // Register to activate mag (lsm9ds1.pdf p.42)

    /**
     * @brief constructors
     */

    LMS9DS1::LMS9DS1(RODOS::I2C_IDX idx, int16_t GYR_CALIB_VALS[3], int16_t ACC_CALIB_VALS[3], int16_t MAG_BOUNDRIES[3][2]) : RODOS::HAL_I2C(idx)
    {
        RODOS::memcpy(LMS9DS1::ACC_CALIB_VALS, ACC_CALIB_VALS, 3 * sizeof(int16_t));
        RODOS::memcpy(LMS9DS1::GYR_CALIB_VALS, GYR_CALIB_VALS, 3 * sizeof(int16_t));
        RODOS::memcpy(LMS9DS1::MAG_BOUNDRIES, MAG_BOUNDRIES, 3 * 2 * sizeof(int16_t));
    }

    LMS9DS1::LMS9DS1(RODOS::I2C_IDX idx) : RODOS::HAL_I2C(idx) {} // uses default calib values

    LMS9DS1::LMS9DS1() : RODOS::HAL_I2C(RODOS::I2C_IDX::I2C_IDX2) {} // PB10 & PB11

    int32_t LMS9DS1::init(uint32_t speed)
    {
        int32_t ret = HAL_I2C::init(speed);

        // init all the registered required for I2C communications
        uint8_t INIT_REG_ACC[2] = {0x20, 0b10000011};
        LMS9DS1::write(AccGyrADDR, INIT_REG_ACC, 2);

        uint8_t INIT_REG_GYR[2] = {0x10, 0b10011000};
        LMS9DS1::write(AccGyrADDR, INIT_REG_GYR, 2);

        uint8_t INIT_REG_MAG_1[2] = {0x20, 0b00011100};
        LMS9DS1::write(MagADDR, INIT_REG_MAG_1, 2);

        uint8_t INIT_REG_MAG_2[2] = {0x21, 0b01100000};
        LMS9DS1::write(MagADDR, INIT_REG_MAG_2, 2);

        uint8_t INIT_REG_MAG_3[2] = {0x22, 0b00000000};
        LMS9DS1::write(MagADDR, INIT_REG_MAG_3, 2);

        return ret;
    }

    void LMS9DS1::read_raw()
    {
        auto i2cerror = [&]()
        {
            this->reset();
            AT(NOW() + 5 * MILLISECONDS);
            this->init(this->speed);
            RODOS::PRINTF("reset IMU");
        };

        //  accellerometer
        if (LMS9DS1::writeRead(AccGyrADDR, ACC_X_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(AccGyrADDR, ACC_X_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::ACC_RAW_VALS[0] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        if (LMS9DS1::writeRead(AccGyrADDR, ACC_Y_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(AccGyrADDR, ACC_Y_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::ACC_RAW_VALS[1] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        if (LMS9DS1::writeRead(AccGyrADDR, ACC_Z_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(AccGyrADDR, ACC_Z_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::ACC_RAW_VALS[2] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        // gyroscope
        if (LMS9DS1::writeRead(AccGyrADDR, GYR_X_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(AccGyrADDR, GYR_X_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::GYR_RAW_VALS[0] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        if (LMS9DS1::writeRead(AccGyrADDR, GYR_Y_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(AccGyrADDR, GYR_Y_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::GYR_RAW_VALS[1] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        if (LMS9DS1::writeRead(AccGyrADDR, GYR_Z_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(AccGyrADDR, GYR_Z_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::GYR_RAW_VALS[2] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        // magnetometer
        if (LMS9DS1::writeRead(MagADDR, MAG_X_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(MagADDR, MAG_X_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::MAG_RAW_VALS[0] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        if (LMS9DS1::writeRead(MagADDR, MAG_Y_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(MagADDR, MAG_Y_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::MAG_RAW_VALS[1] = DATA_L[0] + int16_t((DATA_H[0] << 8));

        if (LMS9DS1::writeRead(MagADDR, MAG_Z_L, 1, DATA_L, 1) <= 0)
            i2cerror();
        if (LMS9DS1::writeRead(MagADDR, MAG_Z_H, 1, DATA_H, 1) <= 0)
            i2cerror();
        LMS9DS1::MAG_RAW_VALS[2] = DATA_L[0] + int16_t((DATA_H[0] << 8));
    }

    void LMS9DS1::read_adj()
    {
        LMS9DS1::read_raw();

        // calculate acceleration
        for (int j = 0; j < 3; j++)
            LMS9DS1::ACC_ADJ_VALS[j] = (0.061f / 1000) * (LMS9DS1::ACC_RAW_VALS[j] - LMS9DS1::ACC_CALIB_VALS[j]);

        // calculate angular velocity
        for (int j = 0; j < 3; j++)
            LMS9DS1::GYR_ADJ_VALS[j] = (70.0f / 1000) * (LMS9DS1::GYR_RAW_VALS[j] - LMS9DS1::GYR_CALIB_VALS[j]);

        // calculate magnetic vector
        for (int j = 0; j < 3; j++)
            LMS9DS1::MAG_ADJ_VALS[j] = (((float)(LMS9DS1::MAG_RAW_VALS[j] - LMS9DS1::MAG_BOUNDRIES[j][0])) / (LMS9DS1::MAG_BOUNDRIES[j][1] - LMS9DS1::MAG_BOUNDRIES[j][0])) * 2 - 1; // normalize raw data to be within [0-1] (unlesse boundry is breached)
    }

    void LMS9DS1::cpy_raw(float *ACC_RAW_VALS, float *GYR_RAW_VALS, float *MAG_RAW_VALS)
    {
        RODOS::memcpy(ACC_RAW_VALS, LMS9DS1::ACC_RAW_VALS, sizeof(float) * 3);
        RODOS::memcpy(GYR_RAW_VALS, LMS9DS1::GYR_RAW_VALS, sizeof(float) * 3);
        RODOS::memcpy(MAG_RAW_VALS, LMS9DS1::MAG_RAW_VALS, sizeof(float) * 3);
    }

    void LMS9DS1::cpy_adj(float *ACC_ADJ_VALS, float *GYR_ADJ_VALS, float *MAG_ADJ_VALS)
    {
        RODOS::memcpy(ACC_ADJ_VALS, LMS9DS1::ACC_ADJ_VALS, sizeof(float) * 3);
        RODOS::memcpy(GYR_ADJ_VALS, LMS9DS1::GYR_ADJ_VALS, sizeof(float) * 3);
        RODOS::memcpy(MAG_ADJ_VALS, LMS9DS1::MAG_ADJ_VALS, sizeof(float) * 3);
    }

    /**
     * @brief print methods
     * mostly for debugging -.-
     */
    void LMS9DS1::print_real()
    {
        PRINTF("#########-ADJ-###########\nacc: %f, %f, %f\ngyr: %f, %f, %f\nmag: %f, %f, %f\n\n",
               (double)LMS9DS1::ACC_ADJ_VALS[0], (double)LMS9DS1::ACC_ADJ_VALS[1], (double)LMS9DS1::ACC_ADJ_VALS[2],
               (double)LMS9DS1::GYR_ADJ_VALS[0], (double)LMS9DS1::GYR_ADJ_VALS[1], (double)LMS9DS1::GYR_ADJ_VALS[2],
               (double)LMS9DS1::MAG_ADJ_VALS[0], (double)LMS9DS1::MAG_ADJ_VALS[1], (double)LMS9DS1::MAG_ADJ_VALS[2]);
    }

    // horrendous code, sorry
    template <typename T>
    void print_bits(T num)
    {
        int size = sizeof(T);
        for (int i = 0; i++ < 8 * size; num <<= 1)
            PRINTF("%d", !!(num & 0x01 << 8 * size));
    }

    void LMS9DS1::print_raw_bits()
    {
        PRINTF("##########-RAW_BITS-###########\n");
        PRINTF("\nacc:");
        print_bits(LMS9DS1::ACC_RAW_VALS[0]);
        PRINTF(", ");
        print_bits(LMS9DS1::ACC_RAW_VALS[1]);
        PRINTF(", ");
        print_bits(LMS9DS1::ACC_RAW_VALS[2]);
        PRINTF("\ngyr:");
        print_bits(LMS9DS1::GYR_RAW_VALS[0]);
        PRINTF(", ");
        print_bits(LMS9DS1::GYR_RAW_VALS[1]);
        PRINTF(", ");
        print_bits(LMS9DS1::GYR_RAW_VALS[2]);
        PRINTF("\nmag:");
        print_bits(LMS9DS1::MAG_RAW_VALS[0]);
        PRINTF(", ");
        print_bits(LMS9DS1::MAG_RAW_VALS[1]);
        PRINTF(", ");
        print_bits(LMS9DS1::MAG_RAW_VALS[2]);
        PRINTF("\n");
    }

    void LMS9DS1::print_raw()
    {
        PRINTF("##########-RAW-###########\nacc: %d, %d, %d\ngyr: %d, %d, %d\nmag: %d, %d, %d\n\n",
               LMS9DS1::ACC_RAW_VALS[0], LMS9DS1::ACC_RAW_VALS[1], LMS9DS1::ACC_RAW_VALS[2],
               LMS9DS1::GYR_RAW_VALS[0], LMS9DS1::GYR_RAW_VALS[1], LMS9DS1::GYR_RAW_VALS[2],
               LMS9DS1::MAG_RAW_VALS[0], LMS9DS1::MAG_RAW_VALS[1], LMS9DS1::MAG_RAW_VALS[2]);
    }
    /**/
}