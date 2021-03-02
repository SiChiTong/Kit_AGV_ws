#pragma once

#define OFF 0
#define ON 1
#define MAX_MSG_LENGTH 1024
#define HSAT 1.5

typedef enum parity {
	PARITY_NONE,
	PARITY_EVEN,
	PARITY_ODD
} parity_t;

typedef enum stop_bits {
	STOPBIT_1,
	STOPBIT_2
} stop_bits_t;

typedef enum data_bits {
    DATABIT_5,
    DATABIT_6,
	DATABIT_7,
    DATABIT_8
} data_bits_t;

struct _rs485 {
    /* Socket or file descriptor */
    int s;
    /* Device: "/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*" on Mac OS X. */
    const char* _port;
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    unsigned int _baud;
    /* Data bit */
    data_bits_t _data_bit;
    /* Stop bit */
    stop_bits_t _stop_bit;
    /* Parity:*/
    parity_t _parity;
    /* Save old termios settings */
    struct termios _old_tios;
};



