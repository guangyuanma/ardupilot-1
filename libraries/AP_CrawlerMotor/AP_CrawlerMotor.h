/*
 履带电机 library
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
        uint8_t frame1;    //帧头0x55
        uint8_t frame2;    //帧头0xaa
        uint8_t cmd;       //行走控制命令0x05
        int16_t V_motor1;  //电机1 速度
        int16_t V_motor2;  //电机2 速度
        uint8_t crc;       //异或校验

    } _cmddata;*/

    //履带电机发送命令
    uint8_t m_cmd[8];

    typedef union
    {
    	uint8_t B[2];
    	uint16_t all;
    	int16_t sall;
    } WORD_Un;

    struct
    {
        uint8_t frame1;    //帧头0x55
        uint8_t frame2;    //帧头0xaa
        uint8_t cmd;       //行走控制命令0x05
        WORD_Un V_motor1;  //电机1 速度
        WORD_Un V_motor2;  //电机2 速度
        int16_t Current_motor1;  //电机1 电流 系数0.01
        int16_t Current_motor2;  //电机2 电流 系数0.01
        int16_t Voltage;  //电压
        uint8_t temp_motor1;    //电机1温度
        uint8_t temp_motor2;    //电机2温度
        uint8_t error;    //故障代码
        uint8_t crc;       //异或校验
    } _recvdata;



    int16_t m_crawlerspeed_motor1;
    int16_t m_crawlerspeed_motor2;

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver

    uint8_t _step;



};
