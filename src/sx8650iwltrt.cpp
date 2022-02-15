/*
 * Copyright (c) 2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sx8650iwltrt.h"
#include "lvgl.h"
// #include "touch_tft.h"

/* sx8650 commands */
#define SX8650_SELECT_CH        0x80    // Select Channel Register : Select and bias a channel
#define SX8650_CONVERT_CH       0x90    // Convert Channel Register : Select and bias a channel Wait for the programmed settling time (POWDLY) Start conversion
#define SX8650_RESET            0xDE    // value to reset software

/* bits for I2C_REG_STAT */
#define STATUS_CONVIRQ          0x80	// I2C_REG_STAT: end of conversion flag
#define STATUS_PENIRQ           0x40	// I2C_REG_STAT: pen detected

/* Screen size */
#define SCREEN_HEIGHT   160 
#define SCREEN_WIDTH    128


namespace sixtron {

    

    SX8650IWLTRT::SX8650IWLTRT(PinName i2c_sda, PinName i2c_sdl ,
    PinName spi_mosi,PinName spi_miso ,PinName spi_sck ,I2CAddress i2cAddress):
    _i2c(i2c_sda,i2c_sdl),
    _spi(spi_mosi,spi_miso,spi_sck),
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


    void SX8650IWLTRT::attach(Callback<void()> function)
    {
        _user_callback = function;
    }    

    void SX8650IWLTRT::read_channel(){
        i2c_read_channel();
    }

    void SX8650IWLTRT::set_rate(Rate value){
        i2c_set_register(RegisterAddress::I2CRegCtrl0,static_cast<char>(value));
    }

    Rate SX8650IWLTRT::rate(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl0,&data);
        return static_cast<Rate>(data);
    }

    void SX8650IWLTRT::set_condirq(RegCtrl1Address value){
        i2c_set_register(RegisterAddress::I2CRegCtrl1,static_cast<char>(value));
    }

    RegCtrl1Address SX8650IWLTRT::condirq(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegCtrl1,&data);
        return static_cast<RegCtrl1Address>(data);
    }

    uint8_t SX8650IWLTRT::convirq(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegStat,&data);
        return static_cast<uint8_t>(data>>7);
    }

    uint8_t SX8650IWLTRT::penirq(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegStat,&data);
        return static_cast<uint8_t>(data>>6);
    }

    
    void SX8650IWLTRT::calibrate(Callback<uint8_t()> func){
        bool calibration_check(false);
        uint16_t pointcheck[6] = {10,10,64,80,118,150};
        // draw_cross(10,10);
        while(!calibration_check)
        {
            for(int i = 0;i<5;i=i+2)
            {
                if((coordinates.x-8 > pointcheck[i] || pointcheck[i] > coordinates.x+8) 
                && (coordinates.y-11 > pointcheck[i+1] || pointcheck[i+1] > coordinates.y+11))
                {
                    printf("P%d validated\n\n Touch the next cross\n\n",i);
                }
            }
        }
        printf("Calibration done!\n\n");
    }

    void SX8650IWLTRT::draw_cross(uint8_t x, uint8_t y)
    {
        static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(SCREEN_WIDTH, SCREEN_HEIGHT)];
        lv_obj_t *canvas = lv_canvas_create(lv_scr_act());
        lv_canvas_set_buffer(canvas, cbuf, SCREEN_WIDTH, SCREEN_HEIGHT, LV_IMG_CF_TRUE_COLOR);

        uint8_t w = 20;
        uint8_t h = 20;
        const lv_point_t vertical[] = {{x, y - h / 2}, {x, y + h / 2}};
        const lv_point_t vertical[] = {{x, y - h / 2}, {x, y + h / 2}};
        lv_draw_line_dsc_t line;
        lv_draw_line_dsc_init(&line);
        line.color = LV_COLOR_MAKE(255, 255, 255);
        line.width = 1;

        lv_canvas_draw_line(canvas, vertical, 2, &line);

        const lv_point_t horizontal[] = {{x - w / 2, y}, {x + w / 2, y}};

        lv_canvas_draw_line(canvas, horizontal, 2, &line);
    }

    void SX8650IWLTRT::set_calibration(uint8_t a ,uint8_t b ,uint8_t c){
       
    }


/* PRIVATE */

    void SX8650IWLTRT::select_channel(ChannelAddress value){
        char tmp = static_cast<char>(SX8650_SELECT_CH | value);
        i2c_write_command(tmp);
    }

    void SX8650IWLTRT::convert_channel(ChannelAddress value){
        char tmp = static_cast<char>(SX8650_CONVERT_CH | value);
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
        char data = static_cast<char>(writecommand);

        if (_i2c.write(static_cast<int>(_i2cAddress), &data, 1) != 0) {
            return -1;
        }
        return 0;
    }

    int SX8650IWLTRT::i2c_read_channel(){
        char data[4];
        if (_i2c.read(static_cast<int>(_i2cAddress), data, 4) != 0) {
            return -2;
        }
        coordinates.x = ((data[0] & 0x0F)<<8 | data[1])*128/(4095-250);
        coordinates.y = ((data[2] & 0x0F)<<8 | data[3])*160/(4095-250);
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

    void SX8650IWLTRT::set_auxaqc(RegCtrl1Address value){
        i2c_set_register(RegisterAddress::I2CRegCtrl1,static_cast<char>(value));
    }

    RegCtrl1Address SX8650IWLTRT::auxaqc(){
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

