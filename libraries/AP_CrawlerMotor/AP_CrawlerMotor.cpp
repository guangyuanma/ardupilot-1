/*

   Inspired by work done here https://github.com/PX4/Firmware/tree/master/src/drivers/frsky_telemetry from Stefan Rado <px4@sradonia.net>

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

/* 
   FRSKY Telemetry library
*/
#define AP_SERIALMANAGER_YI_STAR_BAND 9600
#define AP_SERIALMANAGER_YI_STAR_BUFSIZE_RX 64
#define AP_SERIALMANAGER_YI_STAR_BUFSIZE_TX 64
#include "AP_CrawlerMotor.h"



extern const AP_HAL::HAL& hal;

AP_CrawlerMotor::AP_CrawlerMotor(void)
{
	_port = NULL;
	_step = 0;
    memset(&m_cmd, 0, 8);
    m_crawlerspeed_motor1 = 300;
    m_crawlerspeed_motor2 = 300;

}

/*
 * init - perform required initialisation
 */

void AP_CrawlerMotor::init(const AP_SerialManager& serial_manager)

{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CrawlerMotor, 0)))
    {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_YI_STAR_BAND, AP_SERIALMANAGER_YI_STAR_BUFSIZE_RX, AP_SERIALMANAGER_YI_STAR_BUFSIZE_TX);
    }

}


bool AP_CrawlerMotor::update(void)
{
	if(_port == NULL)
		return false;



	if (_port->txspace() < 8)
	{
		return false;
	}
	m_cmd[0] = 0x55;
	m_cmd[1]  = 0xaa;
	m_cmd[2]  = 0x05;
	WORD_Un speed1;


	// 先高后低字节
	speed1.sall = m_crawlerspeed_motor1;
	m_cmd[3]  = speed1.B[1];
	m_cmd[4]  = speed1.B[0];

	WORD_Un speed2;
	speed2.sall = m_crawlerspeed_motor2;
	m_cmd[5]  = speed2.B[1];
	m_cmd[6]  = speed2.B[0];

	uint8_t cksSum = 0;
	for(int i = 0; i <7; i++)
	{
		cksSum ^= m_cmd[i];
	}
	m_cmd[7] = cksSum;

	_port->write(m_cmd, sizeof(m_cmd));

	return true;

}







