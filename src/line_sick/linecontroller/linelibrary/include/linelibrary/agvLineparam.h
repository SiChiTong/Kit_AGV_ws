#ifndef AGVLINEPARAM_H
#define AGVLINEPARAM_H
#pragma once 

#include <unistd.h>
#include <libraryparam/agvparam.h>

#define LCP_NUM_BIT             0b00000111
#define MAKER_NUM_BIT 	        0b11111000 >> 3
#define LINE_IS_GOOD_BIT        0b00000001
#define TRACK_LEVEL_BIT         0b00001110 >> 1
#define SENSOR_FLIPPED_BIT      0b00010000 >> 4
#define POLARITY_BIT 	        0b00100000 >> 5
#define READING_CODE_BIT      	0b01000000 >> 6
#define EVENT_FLAG_BIT          0b10000000 >> 7


/* Parameter agv concept */
struct agvLineParam{
	agvParam agv;					/*	folow package libraryparam/agvparam.h	*/
    double Lm; 						/*  The distance from the center of the wheel axis to the center line  */
    double V;  						/*  Forward velocity (ie meters per second)*/
    float K;						/*  */
	double PresentRunning;     		/*  Percent speed normal % */
    double PresentSlow;             /*  Percent speed slow %  */
	float error_max;
};

/* SICK line MLSE/ PD0/LCP */
struct Lcp_info{
	uint8_t lcp_nummber;
	uint8_t marker;
};  

/* SICK line MLSE/ PD0/STATUS */								
struct Status_info{
	bool line_good;
	bool sensor_fipped;
	bool polarity;
	bool reading_code;
	uint8_t track_level;
}; 

/* SICK line MLSE */
struct Mlse_info{
	uint8_t status;
	uint8_t lcp;
	uint8_t error_register;	
	double position[3];
};  

struct agvLineinfo{
	Mlse_info mlse;
	Status_info status;
	Lcp_info lcp;
};

#endif