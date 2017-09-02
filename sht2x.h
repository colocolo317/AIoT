#include "mbed.h"

#ifndef SHT2x_H
#define SHT2x_H

const int POLYNOMIAL = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001
const int ACK = 1;
const int NoACK = 0;

// sensor command
enum etSHT2xCommand {
    TRIG_T_MEASUREMENT_HM    = 0xE3, // 0b11100011 command trig. temp meas. hold master
    TRIG_RH_MEASUREMENT_HM   = 0xE5, // command trig. humidity meas. hold master
    TRIG_T_MEASUREMENT_POLL  = 0xF3, // command trig. temp meas. no hold master
    TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
    USER_REG_W               = 0xE6, // command writing user register
    USER_REG_R               = 0xE7, // command reading user register
    SOFT_RESET               = 0xFE  // command soft reset
};

enum etSHT2xResolution {
    SHT2x_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
    SHT2x_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
    SHT2x_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
    SHT2x_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
    SHT2x_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
};

enum etSHT2xEob {
    SHT2x_EOB_ON             = 0x40, // end of battery
    SHT2x_EOB_MASK           = 0x40, // Mask for EOB bit(6) in user reg.
};

enum etSHT2xHeater {
    SHT2x_HEATER_ON          = 0x04, // heater on
    SHT2x_HEATER_OFF         = 0x00, // heater off
    SHT2x_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
};
// measurement signal selection

enum etSHT2xMeasureType {
    HUMIDITY,
    TEMP
};

enum etI2cHeader {
    I2C_ADR_W                = 128,   // sensor I2C address + write bit
    I2C_ADR_R                = 129    // sensor I2C address + read bit
};

// Error codes
enum etError {
    ACK_ERROR                = 0x01,
    TIME_OUT_ERROR           = 0x02,
    CHECKSUM_ERROR           = 0x04,
    UNIT_ERROR               = 0x08
};

class SHT2x{
public:
    SHT2x (PinName p_sda, PinName p_scl);

    int checkCrc(int data[], int nbrOfBytes, int checksum);
    int readUserRegister(int *pRegisterValue);
    int writeUserRegister(int *pRegisterValue);
    int measurePoll(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand);
    int measureHM(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand);
    int softReset();
    float calcRH(int u16sRH);
    float calcTemperatureC(int u16sT);
    int getSerialNumber(int u8SerialNumber[]);
    float getDewpoint(float h, float t);    

protected:
    I2C i2c;
    int i2cWrite(int data);

private:
    PinName _sda;
    PinName _scl;

};
#endif