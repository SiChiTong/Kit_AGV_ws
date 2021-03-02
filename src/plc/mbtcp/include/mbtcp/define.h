#pragma once
#include <iostream>

typedef struct{
	std::string ip_address;
	int port;
	int id;
}info;

typedef enum MODBUS_EXIT_CODES_ENUM
{
  EXIT_OK = 0,
  EXIT_ERROR = 1,
  EXIT_CONFIG_ERROR
} MODBUS_EXIT_CODES;
