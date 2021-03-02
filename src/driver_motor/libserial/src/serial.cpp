#include "serial.h"

speed_t Serial::setBaudRate(unsigned int b){
    speed_t speed;
    switch (b) {
    case 110:
        speed = B110;
        break;
    case 300:
        speed = B300;
        break;
    case 600:
        speed = B600;
        break;
    case 1200:
        speed = B1200;
        break;
    case 2400:
        speed = B2400;
        break;
    case 4800:
        speed = B4800;
        break;
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 8400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 15200:
        speed = B115200;
        break;
    case 230400:
        speed = B230400;
        break;
    case 460800:
        speed = B460800;
        break;
    case 500000:
        speed = B500000;
        break;
    case 576000:
        speed = B576000;
        break;
    case 921600:
        speed = B921600;
        break;
    case 1000000:
        speed = B1000000;
        break;
   case 1152000:
        speed = B1152000;
        break;
    case 1500000:
        speed = B1500000;
        break;
    case 2500000:
        speed = B2500000;
        break;
    case 3000000:
        speed = B3000000;
        break;
    case 3500000:
        speed = B3500000;
        break;
    case 4000000:
        speed = B4000000;
        break;
    default:
        speed = B9600;
        break;
    }
    return speed;
}

void Serial::setParity(termios *tios, parity p){  
    /* PARENB       Enable parity bit
    PARODD       Use odd parity instead of even */
    switch (p)
    {
        /* None */
        case PARITY_NONE:
            tios->c_cflag &= ~PARENB;
            break;
        /* Even */
        case PARITY_EVEN:
            tios->c_cflag |= PARENB;
            tios->c_cflag &= ~PARODD;
            break;
         /* Odd */
        case PARITY_ODD:
            tios->c_cflag |= PARENB;
            tios->c_cflag |= PARODD;
            break;
        /* Default None */   
        default:
            tios->c_cflag &= ~PARENB;
            break;
    }  
}

void Serial::setStopBits(termios *tios, stop_bits s){
    /* Stop bit (1 or 2) */
    switch (s)
    {
        case STOPBIT_1:
            tios->c_cflag &= ~CSTOPB;
            break;
            
        case STOPBIT_2:
            tios->c_cflag |= CSTOPB;
            break;
            
        default:
            tios->c_cflag &= ~CSTOPB;
            break;
    }

}

void Serial::setDataBits(termios *tios, data_bits d){
    /* C_CFLAG      Control options
    CLOCAL       Local line - do not change "owner" of port
    CREAD        Enable receiver
    */
    tios->c_cflag |= (CREAD | CLOCAL);
    /* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

    /* Set data bits (5, 6, 7, 8 bits)
       CSIZE        Bit mask for data bits
    */
    tios->c_cflag &= ~CSIZE;
    switch (d)
    {
        case DATABIT_8:
            tios->c_cflag |= CS8;
            break;
            
        case DATABIT_7:
            tios->c_cflag |= CS7;
            break;
            
        case DATABIT_6:
            tios->c_cflag |= CS6;
            break;
            
        case DATABIT_5:
            tios->c_cflag |= CS5;
            break;
            
        default:
            tios->c_cflag |= CS8;
            break;
    }
}

