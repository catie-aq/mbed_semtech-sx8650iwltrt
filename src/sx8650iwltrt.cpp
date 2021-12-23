/*
 * Copyright (c) 2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sx8650iwltrt.h"

/* sx8650 commands */
#define SX8650_SELECT_CH        0x80    // Select Channel Register : Select and bias a channel
#define SX8650_CONVERT_CH       0x90    // Convert Channel Register : Select and bias a channel Wait for the programmed settling time (POWDLY) Start conversion
#define SX8650_RESET            0xDE    // value to reset software

/* bits for I2C_REG_STAT */
#define STATUS_CONVIRQ          0x80	// I2C_REG_STAT: end of conversion flag
#define STATUS_PENIRQ           0x40	// I2C_REG_STAT: pen detected



namespace sixtron {

    SX8650IWLTRT::SX8650IWLTRT(PinName i2c_sda, PinName i2c_sdl ,I2CAddress i2cAddress):
    _i2c(i2c_sda,i2c_sdl),
    _i2cAddress(i2cAddress),
    _user_callback(nullptr),
    _event_queue(),
    _thread()

    {
            
        
        _thread.start(callback(&_event_queue, &EventQueue::dispatch_forever));
    }

/* PUBLIC */

     void SX8650IWLTRT::soft_reset(){
        i2c_set_register(RegisterAddress::I2CRegSoftReset,SX8650_RESET);
    }

    void SX8650IWLTRT::set_mode(Mode mode){
        i2c_write_command(static_cast<char>(mode));
    }

    Mode SX8650IWLTRT::mode(){
        return Mode::ManAuto;
    }

    void SX8650IWLTRT::attach(Callback<void()> function)
    {
        _user_callback = function;
    }

    
/* PRIVATE */

    void SX8650IWLTRT::select_channel(ChannelAddress value){
        char tmp = static_cast<char>(SX8650_SELECT_CH) + static_cast<char>(value);
        i2c_write_command(tmp);
    }

    void SX8650IWLTRT::convert_channel(ChannelAddress value){
        char tmp = static_cast<char>(SX8650_CONVERT_CH) + static_cast<char>(value);
        i2c_write_command(tmp);
    }

    int SX8650IWLTRT::i2c_set_register(RegisterAddress registerAddress,char value ){
        char data[2];
        data[0] = static_cast<char>(registerAddress);
        data[1] = value;
        if (_i2c.write(static_cast<int>(_i2cAddress), data, 2) != 0) {
            return -1;
        } 
        return 0;
    }

    int SX8650IWLTRT::i2c_read_register(RegisterAddress registerAddress, char *value){
        char data = static_cast<char>(0x40|registerAddress);
        if (_i2c.write(static_cast<int>(_i2cAddress), &data, 1, true) != 0) {
            return -1;
        }
        if (_i2c.read(static_cast<int>(_i2cAddress), value, 1) != 0) {
            return -2;
        }
        return 0;
    }

    int SX8650IWLTRT::i2c_write_command(char writecommand){
        char data[1];
        data[0] = writecommand;

        if (_i2c.write(static_cast<int>(_i2cAddress), data, 1) != 0) {
            return -1;
        }
        return 0;
    }

    int SX8650IWLTRT::i2c_read_channel(ChannelAddress chan,char *value){
        static char data[2];
        data[0] = static_cast<char>(chan);
        if (_i2c.write(static_cast<int>(_i2cAddress), data, 1, true) != 0) {
            return -1;
        }
        if (_i2c.read(static_cast<int>(_i2cAddress), data, 2) != 0) {
            return -2;
        }
        *value = (data[1] << 8) | (0xFF & data[0]);
        return 0;
    }
   
    void SX8650IWLTRT::set_powdly(Time value){
        i2c_set_register(RegisterAddress::I2CRegCtrl0,static_cast<char>(value));
    }
  
    Time SX8650IWLTRT::powdly(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl0,&data);
        return static_cast<Time>(data);
    }

    void SX8650IWLTRT::set_rate(Rate value){
        i2c_set_register(RegisterAddress::I2CRegCtrl0,static_cast<char>(value));
    }

    Rate SX8650IWLTRT::rate(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl0,&data);
        return static_cast<Rate>(data);
    }

    void SX8650IWLTRT::set_auxaqc(RegCtrl1Address value){
        i2c_set_register(RegisterAddress::I2CRegCtrl1,static_cast<char>(value));
    }

    RegCtrl1Address SX8650IWLTRT::auxaqc(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl1,&data);
        return static_cast<RegCtrl1Address>(data);
    }

    void SX8650IWLTRT::set_condirq(RegCtrl1Address value){
        i2c_set_register(RegisterAddress::I2CRegCtrl1,static_cast<char>(value));
    }

    RegCtrl1Address SX8650IWLTRT::condirq(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl1,&data);
        return static_cast<RegCtrl1Address>(data);
    }

    void SX8650IWLTRT::set_pen_resistor(PenResistor value){
        i2c_set_register(RegisterAddress::I2CRegCtrl1,static_cast<char>(value));
    }

    PenResistor SX8650IWLTRT::pen_resistor(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl1,&data);
        return static_cast<PenResistor>(data);
    }

    void SX8650IWLTRT::set_filt(RegCtrl1Address value){
        i2c_set_register(RegisterAddress::I2CRegCtrl1,static_cast<char>(value));
    }

    RegCtrl1Address SX8650IWLTRT::filt(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl1,&data);
        return static_cast<RegCtrl1Address>(data);
    }

    void SX8650IWLTRT::set_sedly(Time value){
        i2c_set_register(RegisterAddress::I2CRegCtrl2,static_cast<char>(value));
    }
  
    Time SX8650IWLTRT::sedly(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl2,&data);
        return static_cast<Time>(data);
    }

    void SX8650IWLTRT::set_reg_chan_msk(RegChanMskAddress value){
        i2c_set_register(RegisterAddress::I2CRegChanMsk,static_cast<char>(value));
    }

    RegChanMskAddress SX8650IWLTRT::reg_chan_msk(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegChanMsk,&data);
        return static_cast<RegChanMskAddress>(data);
    }
    
} // namespace sixtron

