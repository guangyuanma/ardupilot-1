
/* 
 ÂÄ´øµç»ú library
*/
#define AP_SERIALMANAGER_CRAWLER_MOTOR_BAND 9600
#define AP_SERIALMANAGER_CRAWLER_MOTOR_BUFSIZE_RX 32
#define AP_SERIALMANAGER_CRAWLER_MOTOR_BUFSIZE_TX 32
#include "AP_CrawlerMotor.h"



extern const AP_HAL::HAL& hal;

AP_CrawlerMotor::AP_CrawlerMotor(void)
{
	_port = NULL;
	_step = 0;

	memset(m_cmd,0,sizeof(m_cmd));
	m_crawlerspeed_motor1 = 0;
	m_crawlerspeed_motor2 = 0;
	memset(&_recvdata, 0, sizeof(_recvdata));

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
        _port->begin(AP_SERIALMANAGER_CRAWLER_MOTOR_BAND, AP_SERIALMANAGER_CRAWLER_MOTOR_BUFSIZE_RX, AP_SERIALMANAGER_CRAWLER_MOTOR_BUFSIZE_TX);
    }

}


bool AP_CrawlerMotor::sendData(void)
{
	// const AP_AHRS &_ahrs = AP::ahrs();

	// this is the constant for hub data frame
	if (_port->txspace() < 8)
	{
		return false;
	}

	m_cmd[0] = 0x55;
	m_cmd[1]  = 0xaa;
	m_cmd[2]  = 0x05;
	WORD_Un speed1;

	speed1.sall = m_crawlerspeed_motor1;
	m_cmd[3]  = speed1.B[0];
	m_cmd[4]  = speed1.B[1];

	WORD_Un speed2;
	speed2.sall = m_crawlerspeed_motor2;
	m_cmd[5]  = speed2.B[0];
	m_cmd[6]  = speed2.B[1];

	uint8_t cksSum = 0;
	for(int i = 0; i <7; i++)
	{
		cksSum ^= m_cmd[i];
	}
	m_cmd[7] = cksSum;

	_port->write(m_cmd, sizeof(m_cmd));

	return true;

}



bool AP_CrawlerMotor::receiveData(void)
{

	if(_port == NULL)
		return false;

	int16_t num = _port->available();
	uint8_t data;

	for(int16_t i = 0; i<num; i++)
	{
		data = _port->read();
		switch(_step)
		{
		  	case 0:
		  	{
		  		if(data == 0x55)
		  			_step = 1;
		  		else
		  			_step = 0;
		  		break;
		  	}

		  	case 1:
		  	{
		  		if(data == 0xaa)
		  			_step = 2;
		  		else
		  			_step = 0;
		  		break;
		  	}


		  	case 2:
		  	{
		  		if(data == 0x05)
		  			_step = 3;
		  		else
		  			_step = 0;
		  		break;
		  	}

		  	case 3:
		  	{
		  		_recvdata.V_motor1.B[0] = data;
		  		 _step = 4;
		  		break;
		  	}
		  	case 4:
		  	{
		  		_recvdata.V_motor1.B[1] = data;
		  		 _step = 5;
		  		break;
		  	}
		  	case 5:
		  	{
		  		_recvdata.V_motor2.B[0] = data;
		  		 _step = 6;
		  		break;
		  	}
		  	case 6:
		  	{
		  		_recvdata.V_motor2.B[1] = data;
		  		 _step = 0;
		  		break;
		  	}





		  	default:
		  		_step = 0;
		 }


	}

	return true;

}






