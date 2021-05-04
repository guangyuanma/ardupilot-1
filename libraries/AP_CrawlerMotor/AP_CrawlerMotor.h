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


class AP_CrawlerMotor {
public:
    AP_CrawlerMotor();

    /* Do not allow copies */
    AP_CrawlerMotor(const AP_CrawlerMotor &other) = delete;
    AP_CrawlerMotor &operator=(const AP_CrawlerMotor&) = delete;

    // init - perform required initialisation
    void init(const AP_SerialManager& serial_manager);

    bool update(void);


    //履带电机发送命令
    uint8_t m_cmd[8];
    typedef union
    {
    	uint8_t B[2];
    	uint16_t all;
    	int16_t sall;
    } WORD_Un;

    int16_t m_crawlerspeed_motor1;
    int16_t m_crawlerspeed_motor2;
private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver

    uint8_t _step;




};
