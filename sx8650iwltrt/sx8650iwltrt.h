/*
 * Copyright (c) 2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CATIE_SIXTRON_SX8650IWLTRT_H_
#define CATIE_SIXTRON_SX8650IWLTRT_H_

#include "mbed.h"
#include "lvgl.h"
#include "ili9163c.h"


namespace sixtron {

struct COORDINATES {
        uint16_t x = 2;
        uint16_t y = 2;
};

struct COEFFICIENT {
        uint16_t ax = 0;
        uint16_t x_off = 0;
        uint16_t ay = 0;
        uint16_t y_off = 0;
};

struct POINT {
        uint16_t x = 0;
        uint16_t y = 0;
};

enum I2CAddress {
        Address1                = (0x91),       // slave address with pin A0 connected to VDD
        Address2                = (0x90),       // slave address with pin A0 connected to ground
};

enum ChannelAddress : uint8_t {
        /** DATA ADDRESS DEFINITIONS */
        CH_X                    = (0x00),       // X Coordinates Channel 
        CH_Y                    = (0x01),       // Y Coordinates Channel 
        CH_Z1                   = (0x02),       // First channel for pressure measurement
        CH_Z2                   = (0x03),       // Second channele for pressure measurement
        CH_AUX                  = (0x04),       // Auxiliary channel
        CH_SEQ                  = (0x07),       // Channel sequentially selected from i2CRegChanMsk multiple channels are sampled. This requires programming the POWDLY field in register RegCTRL0.

};

enum Mode : uint8_t {
        /* sx8650 MODE ADDRESS DEFINITIONS */
        ManAuto                 = (0XB0),       // Enter Manual or automatic mode
        PenDet                 = (0XC0),       // Enter pen detect mode
        PenTrg                  = (0XE0),       // Enter pen trigger mode
};

enum RegisterAddress : uint8_t {
        /* sx8650 REGISTER ADDRESS DEFINITIONS */
        I2CRegCtrl0             = (0x00),       // I2C register control 0: Rate and Powdly
        I2CRegCtrl1             = (0x01),       // I2C register control 1: AuxAQC Condirq Rpdnt and Filt
        I2CRegCtrl2             = (0x02),       // I2C register control 2: Setdly
        I2CRegChanMsk           = (0x04),       // I2C register channel mask: is used when channel Seq is select    
        I2CRegStat              = (0x05),       // I2C register status : The host status reading allows the host to read the status of the SX8650
        I2CRegSoftReset         = (0x1F),       // I2C register soft reset
};

enum Rate : uint8_t {
        /* sx8650 RATE ADDRESS DEFINITIONS in coordinates per second */
        /* If RATE = 0 --> Manual mode if RATE > 0 then Automatic mode */
        RATE_TIMER_DISABLED     = (0x00),       //Timer disabled Manual mode
        RATE_10_cps             = (0x10),
        RATE_20_cps             = (0x20),
        RATE_40_cps             = (0x30),
        RATE_60_cps             = (0x40),
        RATE_80_cps             = (0x50),
        RATE_100_cps            = (0x60),
        RATE_200_cps            = (0x70),
        RATE_300_cps            = (0x80),
        RATE_400_cps            = (0x90),
        RATE_500_cps            = (0xA0),
        RATE_1K_cps             = (0xB0),
        RATE_2K_cps             = (0xC0),
        RATE_3K_cps             = (0xD0),
        RATE_4K_cps             = (0xE0),
        RATE_5K_cps             = (0xF0),
};

enum Time : uint8_t {
        /* sx8650 power delay: The channel will be biased for a time of POWDLY before each channel conversion for CTRL0 register */
        POWDLY_IMMEDIATE        = (0x00),       // Immediate 0,5 us
        POWDLY_1_1US            = (0x01),
        POWDLY_2_2US            = (0x02),
        POWDLY_4_4US            = (0x03),
        POWDLY_8_9US            = (0x04),
        POWDLY_17_8US           = (0x05),
        POWDLY_35_5US           = (0x06),
        POWDLY_71US             = (0x07),
        POWDLY_140US            = (0x08),
        POWDLY_280US            = (0x09),
        POWDLY_570US            = (0x0A),
        POWDLY_1_1MS            = (0x0B),
        POWDLY_2_3MS            = (0x0C),
        POWDLY_4_6MS            = (0x0D),
        POWDLY_9MS              = (0x0E),
        POWDLY_18MS             = (0x0F),
};
enum RegCtrl1Address : uint8_t {
        /* sx8650 bits for RegCtrl1 */
        CONDIRQ                 = (0x20),       // enable conditional interrupts
        FILT_NONE               = (0x00),       // no averaging 
        FILT_3SA                = (0x01),       // 3 sample averaging 
        FILT_5SA                = (0x02),       // 5 sample averaging 
        FILT_7SA                = (0x03),       // 7 samples, sort, then average of 3 middle samples
};

enum PenResistor : uint8_t {
        /* sx8650 bits for RegCtrl1 : select the pen detect resistor */
        RPDNT_100_KOHM          = (0x00),       
        RPDNT_200_KOHM          = (0x04),       
        RPDNT_50_KOHM           = (0x08),       
        RPDNT_25_KOHM           = (0x0C),            
};
enum RegChanMskAddress : uint8_t {
        /* sx8650 bits for register , I2CRegChanMsk */
        CONV_X                  = (0x80),       // 0: no sample 1: sample, report X channel
        CONV_Y                  = (0x40),       // 0: no sample 1: sample, report Y channel
        CONV_Z1                 = (0x20),       // 0: no sample 1: sample, report Z1 channel
        CONV_Z2                 = (0x10),       // 0: no sample 1: sample, report Z2 channel
        CONV_AUX                = (0x08),       // 0: no sample 1: sample, report AUX channel
};

class SX8650IWLTRT
{
public:
    
     volatile struct COORDINATES coordinates;
     volatile struct COEFFICIENT coefficient;

       /*! Constructor
     *
     *  \param sda I2C data line pin
     *  \param scl I2C clock line pin
     *  \param mosi SPI data input from master pin
     *  \param miso SPI data output from slave pin
     *  \param sck SPI serial clock line pin
     */
    SX8650IWLTRT(PinName i2c_sda, PinName i2c_scl,PinName spi_mosi,PinName spi_miso ,PinName spi_sck,I2CAddress i2cAddress = I2CAddress::Address2 );

    /*! SX8650IWLTRT software reset
     */
    void soft_reset();
   
     /*! Set the SX8650IWLTRT mode
     *
     * \param mode Mode to be applied
     */
    void set_mode(Mode mode);

    /** Attach a callback to interrupt
     *
     * Callback is executed in interrupt modes (\p InterruptReadOutMode and \p
     * WakeUpOperationMode).
     *
     * \param function callback to execute on interrupt
     */
    void attach(Callback<void()> function);

    void read_channel();

    /*! Set the SX8650IWLTRT RegCtrl1 condirq config
     *
     * \param value RegCtrl1Address Address to be applied
     */
    void set_condirq(RegCtrl1Address value);

    /*! Get the SX8650IWLTRT RegCtrl1 condirq config
     *
     * \return condirq
     */
    RegCtrl1Address condirq();  

     /*! Set the SX8650IWLTRT RegCtrl0 rate config
     *
     * \param value Rate Address to be applied
     */
    void set_rate(Rate value);

     /*! Get the SX8650IWLTRT RegCtrl0 rate config
     *
     * \return rate Rate 
     */
    Rate rate();

     /*! Get the SX8650IWLTRT RegCtrl0 rate config
     *
     * \return convirq status
     */
    uint8_t convirq();

    /*! Get the SX8650IWLTRT RegCtrl0 rate config
     *
     * \return penirq status
     */
    uint8_t penirq();

    void get_touch();

    void no_touch();

    void calibrate(Callback<void(int,int)> func);

    void set_calibration(uint8_t a ,uint8_t b ,uint8_t c ,uint8_t d);    

private:

    /*! Set register value
     *
     * \param registerAddress register address     
     * \param value to store write value
     * \param NbWriteValue Number of register to write to be applied
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_set_register(RegisterAddress registerAddress,char value);

    /*! Get register value
     *
     * \param registerAddress register address     
     * \param value to store write value
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_register(RegisterAddress registerAddress,char *value);
    
    /*! Set command register value
     *
     *\param writecommand register address 
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_write_command(char writecommand);
    
    /*! Get channel value
     *
     * \param channel_x channel address
     * \param channel_y to store write value
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_channel();

    /*! Select the SX8650IWLTRT channel
     *
     * \param value Channel Address to be applied
     */
    void select_channel(ChannelAddress value);

    /*! Convert the SX8650IWLTRT channel
     *
     * \param value Channel Address to be applied
     */
    void convert_channel(ChannelAddress value);

    /*! Set the SX8650IWLTRT RegCtrl0 time of powdly config
     *
     * \param value Powdly Address to be applied
     */
    void set_powdly(Time value);

     /*! Get the SX8650IWLTRT RegCtrl0 time of powdly config
     *
     * \returns powdly
     * 
     */
    Time powdly();

    /*! Set the SX8650IWLTRT RegCtrl1 auxaqc config
     *
     * \param value RegCtrl1Address Address to be applied
     */
    void set_auxaqc(RegCtrl1Address value);

    /*! Get the SX8650IWLTRT RegCtrl1 auxaqc config
     *
     * \returns auxaqc
     */
    RegCtrl1Address auxaqc();

     /*! Set the SX8650IWLTRT RegCtrl1 pen resistor config
     *
     * \param value PenResistor Address to be applied
     */
    void set_pen_resistor(PenResistor value);

     /*! Get the SX8650IWLTRT RegCtrl1 pen resistor config
     *
     * \return resistor 
     */
    PenResistor pen_resistor();

    
     /*! Set the SX8650IWLTRT RegCtrl1 filt config
     *
     * \param value RegCtrl1Address Address to be applied
     */
    void set_filt(RegCtrl1Address value);

     /*! Get the SX8650IWLTRT RegCtrl1 filt config
     *
     * \return filt
     */
    RegCtrl1Address filt();

    /*! Set the SX8650IWLTRT RegCtrl1 time of sedly config
     *
     * \param value Powdly Address to be applied
     */
    void set_sedly(Time value);

     /*! Get the SX8650IWLTRT RegCtrl1 time of sedly config
     *
     * \return sedly
     */
    Time sedly();

    /*! Set the SX8650IWLTRT RegCtrlMsk config
     *
     * \param value RegChanMskAddress Address to be applied
     */
    void set_reg_chan_msk(RegChanMskAddress value);

    /*! Get the SX8650IWLTRT RegChanMsk config
     *
     * \return mask
     */
    RegChanMskAddress reg_chan_msk();

    I2C _i2c;
    SPI _spi;
    I2CAddress _i2cAddress;
    Callback<void()> _user_callback;
    EventQueue _event_queue;
    Thread _thread;
    unsigned short pp_tx,pp_ty;
    InterruptIn _nirq;
    bool touch;
};


} // namespace sixtron


#endif // CATIE_SIXTRON_SX8650IWLTRT_H_

