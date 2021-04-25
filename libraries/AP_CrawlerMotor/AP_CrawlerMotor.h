/*
 �Ĵ���� library
*/
#pragma once
#pragma pack(1)

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

    bool sendData(void);

    bool receiveData(void);

/*
    struct
    {
        uint8_t frame1;    //֡ͷ0x55
        uint8_t frame2;    //֡ͷ0xaa
        uint8_t cmd;       //���߿�������0x05
        int16_t V_motor1;  //���1 �ٶ�
        int16_t V_motor2;  //���2 �ٶ�
        uint8_t crc;       //���У��

    } _cmddata;*/

    //�Ĵ������������
    uint8_t m_cmd[8];

    typedef union
    {
    	uint8_t B[2];
    	uint16_t all;
    	int16_t sall;
    } WORD_Un;

    struct
    {
        uint8_t frame1;    //֡ͷ0x55
        uint8_t frame2;    //֡ͷ0xaa
        uint8_t cmd;       //���߿�������0x05
        WORD_Un V_motor1;  //���1 �ٶ�
        WORD_Un V_motor2;  //���2 �ٶ�
        int16_t Current_motor1;  //���1 ���� ϵ��0.01
        int16_t Current_motor2;  //���2 ���� ϵ��0.01
        int16_t Voltage;  //��ѹ
        uint8_t temp_motor1;    //���1�¶�
        uint8_t temp_motor2;    //���2�¶�
        uint8_t error;    //���ϴ���
        uint8_t crc;       //���У��
    } _recvdata;



    int16_t m_crawlerspeed_motor1;
    int16_t m_crawlerspeed_motor2;

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver

    uint8_t _step;



};
