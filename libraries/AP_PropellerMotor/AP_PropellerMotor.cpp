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
#define AP_SERIALMANAGER_YI_STAR_BAND 115200
#define AP_SERIALMANAGER_YI_STAR_BUFSIZE_RX 32
#define AP_SERIALMANAGER_YI_STAR_BUFSIZE_TX 32
#include "AP_PropellerMotor.h"



extern const AP_HAL::HAL& hal;

AP_PropellerMotor::AP_PropellerMotor(void)
{
	_port = NULL;
	_step = 0;
    memset(&m_cmdSpeed, 0, sizeof(m_cmdSpeed));
    memset(&m_cmdOn_OFF, 0, sizeof(m_cmdOn_OFF));
    memset(&m_cmdDirection, 0, sizeof(m_cmdDirection));


}

/*
 * init - perform required initialisation
 */

void AP_PropellerMotor::init(const AP_SerialManager& serial_manager)

{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_PropellerMotor, 0)))
    {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_YI_STAR_BAND, AP_SERIALMANAGER_YI_STAR_BUFSIZE_RX, AP_SERIALMANAGER_YI_STAR_BUFSIZE_TX);
    }

}


bool AP_PropellerMotor::setSpeed_Cmd(uint8_t channel, uint16_t value)
{

	if(_port == NULL)
		return false;

	if (_port->txspace() < sizeof(m_cmdSpeed))
	{
		return false;
	}


	m_cmdSpeed[0] = channel;        //CAN ID0 低位
	m_cmdSpeed[1]  = 0;             //CAN ID1 高位
	m_cmdSpeed[2]  = 0x01;          //功能码01
	m_cmdSpeed[3]  = 0x0C;          //寄存器地址低
	m_cmdSpeed[4]  = 0;             //寄存器地址高

	WORD_Un speed1;                //速度值
	speed1.sall = value;
	// 先高后低字节
	m_cmdSpeed[5]  = speed1.B[1];
	m_cmdSpeed[6]  = speed1.B[0];

	_port->write(m_cmdSpeed, sizeof(m_cmdSpeed));

	 //gcs().send_text(MAV_SEVERITY_INFO,"3241324243");
	return true;

}



bool AP_PropellerMotor::setDirection_Cmd(uint8_t channel, uint8_t value)
{

	if(_port == NULL)
		return false;

	if (_port->txspace() < sizeof(m_cmdSpeed))
	{
		return false;
	}

	m_cmdDirection[0] = channel;        //CAN ID0 低位
	m_cmdDirection[1]  = 0;             //CAN ID1 高位
	m_cmdDirection[2]  = 0x01;          //功能码01
	m_cmdDirection[3]  = 0x34;          //寄存器地址低
	m_cmdDirection[4]  = 0x10;             //寄存器地址高
	m_cmdDirection[5]  = value;

	_port->write(m_cmdSpeed, sizeof(m_cmdSpeed));

	// gcs().send_text(MAV_SEVERITY_INFO,"3241324243");
	return true;

}

// 0 启动 1停止
bool AP_PropellerMotor::setON_OFF_Cmd(uint8_t channel, uint8_t value)
{

	if(_port == NULL)
		return false;

	if (_port->txspace() < sizeof(m_cmdOn_OFF))
	{
		return false;
	}

	m_cmdOn_OFF[0] = channel;        //CAN ID0 低位
	m_cmdOn_OFF[1]  = 0;             //CAN ID1 高位
	m_cmdOn_OFF[2]  = 0x01;          //功能码01
	m_cmdOn_OFF[3]  = value;          //寄存器地址低 00 启动 01 停机
	m_cmdOn_OFF[4]  = 0x30;             //寄存器地址高


	_port->write(m_cmdOn_OFF, sizeof(m_cmdOn_OFF));

	// gcs().send_text(MAV_SEVERITY_INFO,"3241324243");
	return true;

}




