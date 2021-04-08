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
#define AP_SERIALMANAGER_YI_STAR_BUFSIZE_RX 64
#define AP_SERIALMANAGER_YI_STAR_BUFSIZE_TX 64
#include "AP_Yi_Star.h"



extern const AP_HAL::HAL& hal;

AP_Yi_Star::AP_Yi_Star(void)
{
	_port = NULL;
	_step = 0;

}

/*
 * init - perform required initialisation
 */

void AP_Yi_Star::init(const AP_SerialManager& serial_manager)

{
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Yi_Star, 0)))
    {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_YI_STAR_BAND, AP_SERIALMANAGER_YI_STAR_BUFSIZE_RX, AP_SERIALMANAGER_YI_STAR_BUFSIZE_TX);
    }

}


bool AP_Yi_Star::update(void)
{
	if(_port == NULL)
		return false;

	int16_t num = _port->available();
	uint8_t data;
	//uint8_t checksum = 0;

	for(int16_t i = 0; i<num; i++)
	{
		data = _port->read();
		switch(_step)
		{
		  	case 0:
		  		if(data == 0xA5)
		  			_step = 1;
		  		break;

		  	case 1:
		  		if(data == 0x5A)
		  			_step = 2;
		  		else
		  			_step = 0;
		  		break;


		  	case 2:
		  		{
		  			_cx_temp = data;
		  		 	_step = 3;
		  		}

		  		break;

		  	case 3:
		  		{
		  			_cy_temp = data;
		  		 	_step = 4;
		  		}

		  		break;

		  	case 4:
		  		{



		  		 //	if(checksum == data)
		  		 	{   _step = 0;
		  		 	    //checksum = _cx_temp + _cy_temp;

		  		 		cx = _cx_temp;
		  		 		cy = _cy_temp;
		  		 		last_frame_ms = AP_HAL::millis();
		  		 		return true;
		  		 	}


		  		}

		  		break;

		  	default:
		  		_step = 0;
		 }


	}
	return true;

}







