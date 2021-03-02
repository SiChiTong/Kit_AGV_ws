//
// Created by Hiep  on 5/12/2020.
//
#pragma once
#include <ros/ros.h>
#include "serial.h"
#include "setting.h"

typedef struct _rs485 rs485_t;

class rs485: public Serial
{

private:  
    virtual int _rs485_connect(rs485_t *ctx);
    virtual void new_port(const char* devfile, unsigned int baud, parity_t parity, 
                                        data_bits_t data_bit,stop_bits_t stop_bit);
protected:
    rs485_t ctx;
    uint8_t size_pkg;
public:
    virtual ssize_t sendMsgs(uint8_t *to_send, uint length);
    virtual ssize_t receiveMsgs(uint8_t *buffer) const;
    virtual void close_port();
    rs485(const char* devfile, unsigned int baud, parity_t parity, 
                                        data_bits_t data_bit,stop_bits_t stop_bit);
    virtual ~rs485();
};
