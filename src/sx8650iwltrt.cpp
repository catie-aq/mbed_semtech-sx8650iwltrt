/*
 * Copyright (c) 2021, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sx8650iwltrt.h"

/* sx8650 commands */
#define SX8650_SELECT_CH 0x80 // Select Channel Register : Select and bias a channel
#define SX8650_CONVERT_CH                                                                          \
    0x90 // Convert Channel Register : Select and bias a channel Wait for the programmed settling
         // time (POWDLY) Start conversion
#define SX8650_RESET 0xDE // value to reset software

/* bits for I2C_REG_STAT */
#define STATUS_CONVIRQ 0x80 // I2C_REG_STAT: end of conversion flag
#define STATUS_PENIRQ 0x40 // I2C_REG_STAT: pen detected

/* Event Flag for calibrate */
#define TOUCH_DETECTED (1UL << 0)

namespace sixtron {

SX8650IWLTRT::SX8650IWLTRT(
        PinName i2c_sda, PinName i2c_sdl, EventQueue *_event_queue, I2CAddress i2cAddress):
        _i2c(i2c_sda, i2c_sdl),
        _i2cAddress(i2cAddress),
        _user_callback_coordinates(nullptr),
        _user_callback_pressures(nullptr),
        _event_flags(),
        _nirq(DIO5)
{
    _status_calibration = CalibrationMode::Deactivated;
    _nirq.fall(_event_queue->event(this, &SX8650IWLTRT::get_touch));
}

/* PUBLIC */

void SX8650IWLTRT::soft_reset()
{
    i2c_set_register(RegisterAddress::I2CRegSoftReset, SX8650_RESET);
}

void SX8650IWLTRT::set_mode(Mode mode)
{
    i2c_write_command(static_cast<char>(mode));
}

void SX8650IWLTRT::attach_coordinates_measurement(Callback<void(uint16_t, uint16_t)> function)
{
    _user_callback_coordinates = function;
}

void SX8650IWLTRT::attach_pressures_measurement(Callback<void(uint16_t, uint16_t)> function)
{
    _user_callback_pressures = function;
}

void SX8650IWLTRT::enable_coordinates_measurement()
{
    set_reg_chan_msk(RegChanMskAddress(RegChanMskAddress::CONV_X | RegChanMskAddress::CONV_Y));
    _status_msk = _status_msk + uint8_t(RegChanMskAddress::CONV_X | RegChanMskAddress::CONV_Y);
}

void SX8650IWLTRT::enable_pressures_measurement()
{
    set_reg_chan_msk(RegChanMskAddress(RegChanMskAddress::CONV_Z1 | RegChanMskAddress::CONV_Z2));
    _status_msk = _status_msk + uint8_t(RegChanMskAddress::CONV_Z1 | RegChanMskAddress::CONV_Z2);
}

void SX8650IWLTRT::set_rate(Rate value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegCtrl0, &tmp);
    i2c_set_register(RegisterAddress::I2CRegCtrl0, static_cast<char>(value) | tmp);
}

Rate SX8650IWLTRT::rate()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegCtrl0, &data);
    return static_cast<Rate>(data >> 4);
}

void SX8650IWLTRT::set_condirq(RegCtrl1Address value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &tmp);
    i2c_set_register(RegisterAddress::I2CRegCtrl1, static_cast<char>(value) | (tmp & 0xDF));
}

RegCtrl1Address SX8650IWLTRT::condirq()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &data);
    char tmp = (data >> 5) & 0x01;
    return static_cast<RegCtrl1Address>(tmp);
}

uint8_t SX8650IWLTRT::convirq()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegStat, &data);
    return static_cast<uint8_t>(data >> 7) & (0x01);
}

uint8_t SX8650IWLTRT::penirq()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegStat, &data);
    return static_cast<uint8_t>(data >> 6) & (0x01);
}

void SX8650IWLTRT::set_height(uint16_t height)
{
    _height = height;
}

uint16_t SX8650IWLTRT::height()
{
    return _height;
}

void SX8650IWLTRT::set_width(uint16_t width)
{
    _width = width;
}

uint16_t SX8650IWLTRT::width()
{
    return _width;
}

void SX8650IWLTRT::calibrate(Callback<void(int, int)> func)
{

    double xd0, xd1, xd2, yd0, yd1, yd2;
    uint16_t pointcheck[6] = { 10,
        10,
        uint16_t(_width - 20),
        uint16_t(_height / 2),
        uint16_t(_width - 30),
        uint16_t(_height - 10) };
    _status_calibration = CalibrationMode::Activated;
    xd0 = pointcheck[0];
    xd1 = pointcheck[2];
    xd2 = pointcheck[4];
    yd0 = pointcheck[1];
    yd1 = pointcheck[3];
    yd2 = pointcheck[5];

    set_filt(FILT_5SA);

    /* Draw points on the screen */

    func(pointcheck[0], pointcheck[1]);

    /* Wait for touch */

    _event_flags.wait_any(TOUCH_DETECTED);

    /* Touch detected */

    _x0 = _raw_coordinates.x;
    _y0 = _raw_coordinates.y;

    /* Draw points on the screen */

    func(pointcheck[2], pointcheck[3]);

    while (abs(_x0 - _raw_coordinates.x) < 500 && (abs(_y0 - _raw_coordinates.y) < 500)) {
        _event_flags.wait_any(TOUCH_DETECTED);
    }

    /* Touch detected */

    _x1 = _raw_coordinates.x;
    _y1 = _raw_coordinates.y;

    /* Draw points on the screen */

    func(pointcheck[4], pointcheck[5]);

    while (abs((_x1 - _raw_coordinates.x) < 500) && (abs(_y1 - _raw_coordinates.y) < 500)) {
        _event_flags.wait_any(TOUCH_DETECTED);
    }

    /* Touch detected */

    _x2 = _raw_coordinates.x;
    _y2 = _raw_coordinates.y;

    /* Calculate coefficient */

    _k = (_x0 - _x2) * (_y1 - _y2) - (_x1 - _x2) * (_y0 - _y2);
    _coefficient.ax = ((xd0 - xd2) * (_y1 - _y2) - (xd1 - xd2) * (_y0 - _y2)) / _k;
    _coefficient.bx = ((_x0 - _x2) * (xd1 - xd2) - (xd0 - xd2) * (_x1 - _x2)) / _k;
    _coefficient.x_off = (_y0 * (_x2 * xd1 - _x1 * xd2) + _y1 * (_x0 * xd2 - _x2 * xd0)
                                 + _y2 * (_x1 * xd0 - _x0 * xd1))
            / _k;
    _coefficient.ay = ((yd0 - yd2) * (_y1 - _y2) - (yd1 - yd2) * (_y0 - _y2)) / _k;
    _coefficient.by = ((_x0 - _x2) * (yd1 - yd2) - (yd0 - yd2) * (_x1 - _x2)) / _k;
    _coefficient.y_off = (_y0 * (_x2 * yd1 - _x1 * yd2) + _y1 * (_x0 * yd2 - _x2 * yd0)
                                 + _y2 * (_x1 * yd0 - _x0 * yd1))
            / _k;

    _status_calibration = CalibrationMode::Deactivated;
}

void SX8650IWLTRT::set_calibration(double ax, double bx, double x_off, double ay, double by, double y_off)
{

    _coefficient.ax = ax;
    _coefficient.bx = bx;
    _coefficient.x_off = x_off;
    _coefficient.ay = ay;
    _coefficient.by = by;
    _coefficient.y_off = y_off;
}

/* PRIVATE */

void SX8650IWLTRT::select_channel(ChannelAddress value)
{
    char tmp = static_cast<char>(SX8650_SELECT_CH | value);
    i2c_write_command(tmp);
}

void SX8650IWLTRT::convert_channel(ChannelAddress value)
{
    char tmp = static_cast<char>(SX8650_CONVERT_CH | value);
    i2c_write_command(tmp);
}

int SX8650IWLTRT::i2c_set_register(RegisterAddress registerAddress, char value)
{
    char data[2];
    data[0] = static_cast<char>(registerAddress);
    data[1] = value;
    if (_i2c.write(static_cast<int>(_i2cAddress), data, 2) != 0) {
        return -1;
    }
    return 0;
}

int SX8650IWLTRT::i2c_read_register(RegisterAddress registerAddress, char *value)
{
    char data = static_cast<char>(0x40 | registerAddress);
    if (_i2c.write(static_cast<int>(_i2cAddress), &data, 1, true) != 0) {
        return -1;
    }
    if (_i2c.read(static_cast<int>(_i2cAddress), value, 1) != 0) {
        return -2;
    }
    return 0;
}

int SX8650IWLTRT::i2c_write_command(char writecommand)
{
    char data = static_cast<char>(writecommand);

    if (_i2c.write(static_cast<int>(_i2cAddress), &data, 1) != 0) {
        return -1;
    }
    return 0;
}

int SX8650IWLTRT::i2c_read_channel()
{
    switch (_status_msk) {
        case uint8_t(RegChanMskAddress::CONV_X | RegChanMskAddress::CONV_Y):
            char data[4];
            if (_i2c.read(static_cast<int>(_i2cAddress), data, 4) != 0) {
                return -2;
            }
            _raw_coordinates.x = ((data[0] & 0x0F) << 8 | data[1]);
            _raw_coordinates.y = ((data[2] & 0x0F) << 8 | data[3]);
            return 0;
            break;
        case uint8_t(RegChanMskAddress::CONV_Z1 | RegChanMskAddress::CONV_Z2):
            char data1[8];
            if (_i2c.read(static_cast<int>(_i2cAddress), data1, 8) != 0) {
                return -2;
            }
            _pressures.z1 = ((data1[4] & 0x0F) << 8 | data1[5]);
            _pressures.z2 = ((data1[6] & 0x0F) << 8 | data1[7]);
            return 0;
            break;
        case uint8_t(RegChanMskAddress::CONV_X | RegChanMskAddress::CONV_Y
                | RegChanMskAddress::CONV_Z1 | RegChanMskAddress::CONV_Z2):
            char data2[8];
            if (_i2c.read(static_cast<int>(_i2cAddress), data2, 8) != 0) {
                return -2;
            }
            _raw_coordinates.x = ((data2[0] & 0x0F) << 8 | data2[1]);
            _raw_coordinates.y = ((data2[2] & 0x0F) << 8 | data2[3]);
            _pressures.z1 = ((data2[4] & 0x0F) << 8 | data2[5]);
            _pressures.z2 = ((data2[6] & 0x0F) << 8 | data2[7]);
            return 0;
    }
    return 0;
}

void SX8650IWLTRT::set_powdly(Time value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegCtrl0, &tmp);
    i2c_set_register(RegisterAddress::I2CRegCtrl0, static_cast<char>(value) | tmp);
}

Time SX8650IWLTRT::powdly()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegCtrl0, &data);
    char tmp = data & 0x0F;
    return static_cast<Time>(tmp);
}

void SX8650IWLTRT::set_auxaqc(AuxAqc value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &tmp);
    i2c_set_register(RegisterAddress::I2CRegCtrl1, static_cast<char>(value) | (tmp & 0x3F));
}

AuxAqc SX8650IWLTRT::auxaqc()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &data);
    char tmp = (data >> 6) & 0x03;
    return static_cast<AuxAqc>(tmp);
}

void SX8650IWLTRT::set_pen_resistor(PenResistor value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &tmp);
    i2c_set_register(RegisterAddress::I2CRegCtrl1, static_cast<char>(value) | tmp);
}

PenResistor SX8650IWLTRT::pen_resistor()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &data);
    char tmp = (data >> 2) & 0x03;
    return static_cast<PenResistor>(tmp);
}

void SX8650IWLTRT::set_filt(RegCtrl1Address value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &tmp);
    i2c_set_register(RegisterAddress::I2CRegCtrl1, static_cast<char>(value) | tmp);
}

RegCtrl1Address SX8650IWLTRT::filt()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegCtrl1, &data);
    char tmp = data & 0x03;
    return static_cast<RegCtrl1Address>(tmp);
}

void SX8650IWLTRT::set_setdly(Time value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegCtrl2, &tmp);
    i2c_set_register(RegisterAddress::I2CRegCtrl2, static_cast<char>(value) | tmp);
}

Time SX8650IWLTRT::setdly()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegCtrl2, &data);
    char tmp = data & 0x0F;
    return static_cast<Time>(tmp);
}

void SX8650IWLTRT::set_reg_chan_msk(RegChanMskAddress value)
{
    char tmp;
    i2c_read_register(RegisterAddress::I2CRegChanMsk, &tmp);
    i2c_set_register(RegisterAddress::I2CRegChanMsk, static_cast<char>(value) | tmp);
}

RegChanMskAddress SX8650IWLTRT::reg_chan_msk()
{
    char data;
    i2c_read_register(RegisterAddress::I2CRegChanMsk, &data);
    return static_cast<RegChanMskAddress>(data);
}

void SX8650IWLTRT::get_touch()
{
    i2c_read_channel();
    if (_status_calibration == CalibrationMode::Activated) {
        _event_flags.set(TOUCH_DETECTED);
    }
    switch (_status_msk) {
        case uint8_t(RegChanMskAddress::CONV_X | RegChanMskAddress::CONV_Y):
            if (_user_callback_coordinates) {
                if (_status_calibration == CalibrationMode::Deactivated) {
                    _coordinates.x = int(_coefficient.ax * _raw_coordinates.x
                            + _coefficient.bx * _raw_coordinates.y + _coefficient.x_off);
                    _coordinates.y = int(_coefficient.ay * _raw_coordinates.x
                            + _coefficient.by * _raw_coordinates.y + _coefficient.y_off);

                    _user_callback_coordinates(_coordinates.x, _coordinates.y);
                }
            }
            break;
        case uint8_t(RegChanMskAddress::CONV_Z1 | RegChanMskAddress::CONV_Z2):
            if (_user_callback_pressures) {
                _user_callback_pressures(_pressures.z1, _pressures.z2);
            }
            break;
        case uint8_t(RegChanMskAddress::CONV_X | RegChanMskAddress::CONV_Y
                | RegChanMskAddress::CONV_Z1 | RegChanMskAddress::CONV_Z2):
            if (_user_callback_pressures) {
                _user_callback_pressures(_pressures.z1, _pressures.z2);
            }
            if (_user_callback_coordinates) {
                if (_status_calibration == CalibrationMode::Deactivated) {
                    _coordinates.x = int(_coefficient.ax * _raw_coordinates.x
                            + _coefficient.bx * _raw_coordinates.y + _coefficient.x_off);
                    _coordinates.y = int(_coefficient.ay * _raw_coordinates.x
                            + _coefficient.by * _raw_coordinates.y + _coefficient.y_off);

                    _user_callback_coordinates(_coordinates.x, _coordinates.y);
                } else {
                    _event_flags.set(TOUCH_DETECTED);
                }
            }
    }
}

} // namespace sixtron
