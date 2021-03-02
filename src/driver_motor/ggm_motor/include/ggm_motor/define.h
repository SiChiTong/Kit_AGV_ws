#pragma once
 
#define GGM_MOTOR_NAMESPACE_BEGIN namespace GGM {
#define GGM_MOTOR_NAMESPACE_END }

#define HEADER_RMID_SEND 0xB7
#define HEADER_TMID_SEND 0XAC

#define MAX_BUFFER_LENGHT 2084
#define PACKAGE_MSG_MAIN_DATA 22
#define MAX_ID_LENGHT 253
#define TIME_PUBLISH_TO_SEND 7500 //us 


/* Frame of messgae package*/
typedef struct _Header
{
   uint8_t RMID = HEADER_RMID_SEND;
   uint8_t TMID;
}header_t;

typedef struct _ID_Number
{
   uint8_t adr[MAX_ID_LENGHT];
   uint8_t size;
}id_number_t;

typedef struct _Parameter_ID
{
   /* data */
   uint8_t PID;
}param_id_t;

typedef struct Data_number
{
   uint8_t NUM;
}dt_num_t;

/***********************************************************************************************************/

typedef struct _filter
{
   uint8_t _buffer[MAX_BUFFER_LENGHT];
   uint8_t result[PACKAGE_MSG_MAIN_DATA];
   unsigned long r_index;
   unsigned long w_index;
   unsigned long c_flag;
}filter_t;

enum PID_COMMAND
{
   CMD_TQ_OFF = 2,               // Tq-off, motor free state
   CMD_BRAKE  = 4,               // Erectric brake
   CMD_MAIN_BC_ON = 5,           // PID_MAIN_DATA broadcasting ON
   CMD_MAIN_BC_OFF = 6,          // broadcasing OFF
   CMD_ALARM_RESET = 8,          //  Reset alarm
   CMD_POSI_RESET = 10,          // Position reset(set position to zero)
   CMD_MONITOR_BC_ON = 11,       // PID_MONITOR broadcasting ON
   CMD_MONITOR_BC_OFF = 12,      // Broadcasting off
   CMD_IO_MONITOR_ON = 13,       // PID_IO_MONITOR BC ON
   CMD_IO_MONITOR_OFF = 14,      // PID_IO_MONITOR BC OFF
   CMD_FAN_ON = 15,              // Fan ON(motor cooling fan)
   CMD_FAN_OFF = 16,             // Fan OFF
   CMD_CLUTCH_ON = 17,           // Mechanical brake(clutch) ON
   CMD_CLUTCH_OFF = 18,          // Mechanical breka OFF
   CMD_TAR_VEL_OFF = 20,         // Erase target vel, set by PID_TAR_VEL
   CMD_SLOW_START_OFF = 21,      // Erase target slow/start value
   CMD_SLOW_DOWN_OFF = 22,       // Erase target slow/down vaule
   CMD_CAN_RESEND_ON = 23,       // Send CAN data to RS485 serial port.
   CMD_CAN_RESEND_OFF = 24,      // Turn off resending of CAN data
   CMD_MAX_TQ_OFF = 25,          // Erase target limit load(max. current)
   CMD_ENC_OFF = 26,             // Cancel the use of encoder sensor.
   CMD_LOW_SPEED_LIMIT_OF = 27,  // Cancel the set of low speed limit
   F_HIGH = 28,                  // Cancel the set of high speed limit.
   CMD_SPEED_LI_OFF =29,         // Cancel the set of low/high speed limits
   CMD_CURVE_OFF = 31,           // Cancel set of curve fitting func.
   CMD_STEP_OFF = 32,            // Cancel step input mode
   CMD_UICOM_OFF = 44,           // I/O control(ctrl 11pin cnt) available
   CMD_UICOM_ON = 45,            // I/O control disable(when comm. is used)
   CMD_MAX_RP_OFF = 46,          // Cancel max. speed set by DIP SW
   CMD_HALL_TY_OFF = 47,         // Cancel set of motor hall type
   CMD_LOW_POT_OFF = 48,         // Cancel set of low limit of POT input
   CMD_HIGH_POT_OFF = 49,        // Cancel set of high limit of POT input
   CMD_MAIN_BC_ON2 = 50,         // PID_MAIN_DATA, BC ON for 2nd motor
   CMD_MAIN_BC_OFF2 = 51,        // PID_MAIN_DATA, BC OFF for 2nd motor
   CMD_MONIT_BC_ON2 = 52,        // PID_MONITOR, BC ON for 2nd motor
   CMD_MONIT_BC_OFF2 = 53,       // PID_MONITOR, BC OFF for 2nd motor
   CMD_IO_MONIT_BC_ON2 = 54,     // PID_IO_MONITOR, BC ON for 2nd motor
   CMD_IO_MONIT_BC_OFF2 = 55     // PID_IO_MONITOR, BC OFF for 2nd motor
};

enum _BIT
{
   _OFF = 0x00u,
   _ON = 0x01u 
};

enum _BAUD_OF_MD200
{
   _9600bps = 1,
   _19200bps = 2,
   _38400bps = 3,
   _57600bps = 4,
   _115200bps = 5
};

typedef enum EXIT_CODES_ENUM
{
  EXIT_OK = 0,
  EXIT_ERROR = 1,
  EXIT_CONFIG_ERROR = -1
} EXIT_CODES;
