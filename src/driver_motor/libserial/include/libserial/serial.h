//
// Created by Hiep  on 4/12/2020.
//
#pragma once 
// #include <bits/stdc++.h>
#include <iostream>  /* std::cout */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <cstdint>	/* uin8_t */
#include <stdlib.h>  
#include "setting.h"

class Serial{
public:
   virtual speed_t setBaudRate(unsigned int b);
   virtual void setParity(termios *tios, parity p);
   virtual void setStopBits(termios *tios, stop_bits s);
   virtual void setDataBits(termios *tios, data_bits d);
};
