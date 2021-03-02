#include <ros/ros.h>
#include "libserial/rs485.h"
 
int main(int argc, char *argv[])
{
    rs485 *md200 = new rs485("/dev/ttyUSB0",115200,PARITY_NONE,DATABIT_8,STOPBIT_1);
    delete(md200);
    return 0;
}