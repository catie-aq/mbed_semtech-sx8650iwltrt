/*
 * Copyright (c) 2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sx8650iwltrt.h"
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

/* Event Flag for calibrate */
#define CROSS_1_CHECK (1UL << 0)
#define CROSS_2_CHECK (1UL << 1)
#define CROSS_3_CHECK (1UL << 2)

#define EVENT_FLAGS (CROSS_1_CHECK | CROSS_2_CHECK | CROSS_3_CHECK)


namespace sixtron {

    SX8650IWLTRT::SX8650IWLTRT(PinName i2c_sda, PinName i2c_sdl ,I2CAddress i2cAddress):
    _i2c(i2c_sda,i2c_sdl),
    _i2cAddress(i2cAddress),
    _user_callback(nullptr),
    _event_queue(),
    _event_flags(),
    _thread(),
    _nirq(DIO5),
    _led1(LED1)

    {
        check_calibrate = false;
        touch = false;
        _thread.start(callback(&_event_queue, &EventQueue::dispatch_forever));        
        _nirq.fall(_event_queue.event(this,&SX8650IWLTRT::nirq_fall));
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
        return static_cast<uint8_t>(data>>7)&&(0x01);
    }

    uint8_t SX8650IWLTRT::penirq(){
        char data;
        i2c_read_register(RegisterAddress::I2CRegStat,&data);
        return static_cast<uint8_t>(data>>6)&&(0x01);
    }

    void SX8650IWLTRT::nirq_fall(){
        read_channel();
        // printf("-----------------\n\n");
        // printf("X : %u | Y : %u \n\n",coordinates.x,coordinates.y); 
        _led1 = !_led1;   
    } 

    void SX8650IWLTRT::read_channel(){
        i2c_read_channel();
        coord.x = coefficient.ax*coordinates.x + coefficient.bx*coordinates.x + coefficient.x_off;
        coord.y = coefficient.ay*coordinates.x + coefficient.by*coordinates.x + coefficient.y_off;
        
    }

    void SX8650IWLTRT::calibrate_check()
    {
        printf("Waiting for any flag from 0x%08lx.\r\n", CROSS_1_CHECK & CROSS_2_CHECK);
        uint32_t flags_read = 0;
        while (true) {
            flags_read = _event_flags.wait_all(EVENT_FLAGS);
            printf("Got: 0x%08lx\r\n", flags_read);
            if(flags_read != 0){
                check_calibrate = true;
            }
        }
    }

    void SX8650IWLTRT::calibrate(Callback<void(int,int)> func){
        
        float x0 = 0 , y0 = 0 , x1 = 0 , y1 = 0 , x2 = 0 , y2 = 0 , k = 0;
        float xd0 , xd1 , xd2 , yd0 , yd1 , yd2;
        uint16_t pointcheck[6] = {10, 10, SCREEN_WIDTH - 10, SCREEN_HEIGHT/2, SCREEN_WIDTH - 30, SCREEN_HEIGHT -10};
        
        xd0 = (SCREEN_WIDTH - pointcheck[0]);
        xd1 = (SCREEN_WIDTH - pointcheck[2]);
        xd2 = (SCREEN_WIDTH - pointcheck[4]);
        yd0 = (SCREEN_HEIGHT - pointcheck[1]);
        yd1 = (SCREEN_HEIGHT - pointcheck[3]);
        yd2 = (SCREEN_HEIGHT - pointcheck[5]);

        set_filt(FILT_5SA);

        /* Draw point on edge of the screen */
        func(pointcheck[0],pointcheck[1]);
        func(pointcheck[2],pointcheck[3]);
        func(pointcheck[4],pointcheck[5]);


        // _nirq.fall(callback(this,&SX8650IWLTRT::get_touch));
        // _nirq.rise(callback(this,&SX8650IWLTRT::no_touch));

        /* Check if calibrate not already done */

        printf("First touch the upper left cross \n\n");
        printf("Then touch the middle right cross \n\n");
        printf("Finaly touch the bottom right cross \n\n");

        /* Wait for touch */
        // while(!touch){
        //     ThisThread::sleep_for(100ms);
        // }
        
        /* Touch detected */
        // set_mode(PenTrg);

        // i2c_read_channel();
        printf("Upper left cross \n\n");
        printf("X : %u | Y : %u \n\n",coordinates.x,coordinates.y);
            
        x0 = coordinates.x;
        y0 = coordinates.y;
        
        printf("-----------------\n\n");
        if (x0 != 0 && y0 != 0) 
        {
            _event_flags.set(CROSS_1_CHECK);
        }
        _event_flags.wait_any(CROSS_1_CHECK , osWaitForever, true);
        
        /* Wait for touch */
        // while(!touch){
        //     ThisThread::sleep_for(100ms);
        // }
        
        /* Touch detected */
        // set_mode(PenTrg);
            
        // i2c_read_channel();
        printf("Middle right cross \n\n");
        printf("X : %u | Y : %u \n\n",coordinates.x,coordinates.y);

        x1 = coordinates.x;
        y1 = coordinates.y;

        printf("-----------------\n\n");
        if (x1 != 0 && y1 != 0) 
        {
            _event_flags.set(CROSS_2_CHECK);
        }
        _event_flags.wait_any(CROSS_2_CHECK , osWaitForever, true);


        /* Wait for touch */
        // while(!touch){
        //     ThisThread::sleep_for(100ms);
        // }
        
        /* Touch detected */
        // set_mode(PenTrg);
            
        // i2c_read_channel();
        printf("Bottom right cross \n\n");
        printf("X : %u | Y : %u \n\n",coordinates.x,coordinates.y);

        x2 = coordinates.x;
        y2 = coordinates.y;

        printf("-----------------\n\n");
        if (x2 != 0 && y2 != 0) 
        {
            _event_flags.set(CROSS_3_CHECK);
        }
        _event_flags.wait_any(CROSS_3_CHECK , osWaitForever, true);

        _event_flags.wait_any(EVENT_FLAGS , osWaitForever, true);

        k = (x0-x2)*(y1-y2)-(x1-x2)*(y0-y2);
        coefficient.ax = ((xd0-xd2)*(y1-y2)-(xd1-xd2)*(y0-y2))/k;
        coefficient.bx = ((x0-x2)*(xd1-xd2)-(xd0-xd2)*(x1-x2))/k;
        coefficient.x_off = (y0*(x2*xd1-x1*xd2)+y1*(x0*xd2-x2*xd0)+y2*(x1*xd0-x0*xd1))/k;
        coefficient.ay = ((yd0-yd2)*(y1-y2)-(yd1-yd2)*(y0-y2))/k;
        coefficient.by = ((x0-x2)*(yd1-yd2)-(yd0-yd2)*(x1-x2))/k;
        coefficient.y_off = (y0*(x2*yd1-x1*yd2)+y1*(x0*yd2-x2*yd0)+y2*(x1*yd0-x0*yd1))/k;
    
       
        printf("Calibration done!\n\n");
        read_channel();
        printf("AX %f | BX %f | XoFF %f || AY %f | BY %f | Yoff %f \n\n",coefficient.ax,coefficient.bx,coefficient.x_off,coefficient.ay,coefficient.by,coefficient.y_off);
    }

    void SX8650IWLTRT::set_calibration(){
        if(coefficient.ax != 2.00 && coefficient.bx != 2.00 
        && coefficient.ax != 2.00 && coefficient.by != 2.00 
        && coefficient.x_off != 2.00 && coefficient.y_off != 200 )
        {
            printf("Calibration already done!\n\n");
            check_calibrate = true;
        }
       
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
        coordinates.x = ((data[0] & 0x0F)<<8 | data[1])/*128/(4095-220)*/;
        coordinates.y = ((data[2] & 0x0F)<<8 | data[3])/*160/(4095-180)*/;
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

    void SX8650IWLTRT::get_touch(){
        touch = true;
    }

    void SX8650IWLTRT::no_touch(){
        touch = false;
    }
    
} // namespace sixtron

