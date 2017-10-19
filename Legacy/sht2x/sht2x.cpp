#include "sht2x.h"
#include "mbed.h"

SHT2x::SHT2x (PinName p_sda, PinName p_scl) : i2c(p_sda, p_scl){
    _sda = p_sda;
    _scl = p_scl;
}

//==============================================================================//
int SHT2x::checkCrc(int data[], int nbrOfBytes, int checksum)
//==============================================================================//
{
    // printf("This is checkCrc\r\n");
    uint8_t crc = 0;
    int byteCtr;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr) {
        crc ^= (data[byteCtr]);
        for (int bit = 8; bit > 0; --bit) {
            if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else crc = (crc << 1);
        }
    }

    if (crc != checksum) return CHECKSUM_ERROR;
    else return 0;
}

//===========================================================================//
int SHT2x::i2cWrite(int data)
//===========================================================================//
{
    if (i2c.write(data) < 1) return ACK_ERROR;
    else return 0;
}

//===========================================================================//
int SHT2x::readUserRegister(int *pRegisterValue)
//===========================================================================//
{
    // printf("This is readUserRegister\r\n");
    int checksum;   //variable for checksum byte
    int error=0;    //variable for error code

    i2c.start();
    error |= i2cWrite(I2C_ADR_W);

    error |= i2cWrite(USER_REG_R);

    i2c.start();
    error |= i2cWrite(I2C_ADR_R);
    *pRegisterValue = i2c.read(ACK);
    
    checksum=i2c.read(NoACK);

    error |= checkCrc (pRegisterValue,1,checksum);

    i2c.stop();
    return error;
}
//===========================================================================//
int SHT2x::writeUserRegister(int *pRegisterValue)
//===========================================================================//
{
    // printf("This is writeUserRegister\r\n");
    int error=0;   //variable for error code
    i2c.start();

    error |= i2cWrite(I2C_ADR_W);
    error |= i2cWrite(USER_REG_W);
    error |= i2cWrite(*pRegisterValue);
    i2c.stop();

    return error;
}
//===========================================================================//
int SHT2x::measureHM(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand)
//===========================================================================//
{
    // printf("This is measureHM\r\n");
    int checksum = 0;   //checksum
    int data[2] = {0,0};    //data array for checksum verification
    int error=0;    //error variable

    //-- write I2C sensor address and command --
    i2c.start();

    error |= i2cWrite(I2C_ADR_W); // I2C Adr
    switch (eSHT2xMeasureType) {
        case HUMIDITY:
            error |= i2cWrite(TRIG_RH_MEASUREMENT_HM);
            break;
        case TEMP:
            error |= i2cWrite(TRIG_T_MEASUREMENT_HM);
            break;
        default:
            break;
    }

    //-- wait until hold master is released --
    i2c.start();

    __disable_irq();     // Disable Interrupts

    error |= i2cWrite(I2C_ADR_R);

    // {
    //     // SCL=HIGH;                     // set SCL I/O port as input
    //     // i2c.sclAsInput();
    //     DigitalIn scl_in(_scl);

    //     for (i=0; i<1000; i++) {      // wait until master hold is released or
    //         wait_ms(1);    // a timeout (~1s) is reached
    //         if (scl_in == 1) break;
    //     }

    //     //-- check for timeout --
    //     if (scl_in == 0) error |= TIME_OUT_ERROR;
    //     //-- read two data bytes and one checksum byte --

    //     // i2c.sclNormal();
    // }
    wait_ms(300);

    data[0] = i2c.read(ACK);
    data[1] = i2c.read(ACK);

    data[0] &= 255;
    data[1] &= 255;

    *pMeasurand = data[0] << 8;
    *pMeasurand |= data[1];

    checksum = i2c.read(NoACK);

    __enable_irq();     // Enable Interrupts

    //-- verify checksum --
    error |= checkCrc (data, 2, checksum);

    i2c.stop();
    return error;
}
//===========================================================================//
int SHT2x::measurePoll(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand)
//===========================================================================//
{
    // printf("This is measurePoll\r\n");
    int  checksum;   //checksum
    int  data[2];    //data array for checksum verification
    int  error = 0;    //error variable
    int i = 0;        //counting variableSample Code SHT21

    //-- write I2C sensor address and command --
    i2c.start();

    error |= i2cWrite(I2C_ADR_W);
    switch (eSHT2xMeasureType) {
        case HUMIDITY:
            error |= i2cWrite(TRIG_RH_MEASUREMENT_POLL);
            break;
        case TEMP:
            error |= i2cWrite(TRIG_T_MEASUREMENT_POLL);
            break;
        default:
            break;

    }
    //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
    do {
        i2c.start();
        wait_ms(10);
        if (i++ >= 20) break;
    } while (i2c.write(I2C_ADR_R) == 0);

    if (i >= 20) error |= TIME_OUT_ERROR;

    //-- read two data bytes and one checksum byte --
    data[0] = i2c.read(ACK);
    data[1] = i2c.read(ACK);

    *pMeasurand = data[0] << 8;
    *pMeasurand |= data[1];

    checksum = i2c.read(NoACK);

    //-- verify checksum --
    error |= checkCrc (data,2,checksum);
    i2c.stop();
    return error;
}
//===========================================================================//
int SHT2x::softReset()
//===========================================================================//
{
    // printf("This is softReset\r\n");
    int  error=0;           //error variable
    i2c.start();
    error |= i2cWrite(I2C_ADR_W); // I2C Adr
    error |= i2cWrite(SOFT_RESET);                            // Command
    i2c.stop();
    wait_ms(15);
    return error;
}
//==============================================================================//
float SHT2x::calcRH(int u16sRH)
//==============================================================================//
{
    // printf("This is calcRH\r\n");
    float humidityRH;              // variable for result
    u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --
    humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    return humidityRH;
}
//==============================================================================//
float SHT2x::calcTemperatureC(int u16sT)
//==============================================================================//
{
    // printf("This is calcTemperatureC\r\n");
    float temperatureC;            // variable for result
    u16sT &= ~0x0003;           // clear bits [1..0] (status bits)
    //-- calculate temperature [in degrees C] --
    temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
    return temperatureC;
}

//==============================================================================//
float SHT2x::getDewpoint(float h, float t)
//==============================================================================//
{
    float logEx, dew_point;
    logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
    dew_point = (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);
    return dew_point;
}

//==============================================================================//
int SHT2x::getSerialNumber(int u8SerialNumber[])
//==============================================================================//
{
    int  error=0;                          //error variable
    //Read from memory location 1
    i2c.start();
    error |= i2cWrite(I2C_ADR_W);
    error |= i2cWrite(0xFA);         //Command for readout on-chip memory
    error |= i2cWrite(0x0F);         //on-chip memory address
    i2c.start();
    error |= i2cWrite(I2C_ADR_R);    //I2C address
    u8SerialNumber[5] = i2c.read(ACK); //Read SNB_3
    i2c.read(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
    u8SerialNumber[4] = i2c.read(ACK); //Read SNB_2
    i2c.read(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
    u8SerialNumber[3] = i2c.read(ACK); //Read SNB_1Sample Code SHT21
    i2c.read(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
    u8SerialNumber[2] = i2c.read(ACK); //Read SNB_0
    i2c.read(NoACK);                  //Read CRC SNB_0 (CRC is not analyzed)
    i2c.stop();
    //Read from memory location 2
    i2c.start();
    error |= i2cWrite(I2C_ADR_W);    //I2C address
    error |= i2cWrite(0xFC);         //Command for readout on-chip memory
    error |= i2cWrite(0xC9);         //on-chip memory address
    i2c.start();
    error |= i2cWrite(I2C_ADR_R);    //I2C address
    u8SerialNumber[1] = i2c.read(ACK); //Read SNC_1
    u8SerialNumber[0] = i2c.read(ACK); //Read SNC_0
    i2c.read(ACK);
    u8SerialNumber[7] = i2c.read(ACK); //Read SNA_1
    u8SerialNumber[6] = i2c.read(ACK); //Read SNA_0
    i2c.read(NoACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
    i2c.stop();
    return error;
}