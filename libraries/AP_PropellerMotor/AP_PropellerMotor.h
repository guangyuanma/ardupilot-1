/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>

class AP_PropellerMotor {
public:
    AP_PropellerMotor();

    /* Do not allow copies */
    AP_PropellerMotor(const AP_PropellerMotor &other) = delete;
    AP_PropellerMotor &operator=(const AP_PropellerMotor&) = delete;

    // init - perform required initialisation
    void init(const AP_SerialManager& serial_manager);

    //设置速度命令
    bool setSpeed_Cmd(uint8_t channel, uint16_t value);

    //设置方向命令
    bool setDirection_Cmd(uint8_t channel, uint8_t value);

    //设置启停命令
    bool setON_OFF_Cmd(uint8_t channel, uint8_t value);


    //推进器速度
    uint8_t m_cmdSpeed[7];

    //推进器启停
    uint8_t m_cmdOn_OFF[5];

    //推进器方向
    uint8_t m_cmdDirection[6];

    typedef union
    {
    	uint8_t B[2];
    	uint16_t all;
    	int16_t sall;
    } WORD_Un;


private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver

    uint8_t _step;




};
