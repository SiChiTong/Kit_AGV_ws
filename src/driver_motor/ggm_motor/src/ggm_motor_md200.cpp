#include "ggm_motor/ggm_motor_md200.h"
         

GGM::Driver_md200::~Driver_md200(){}
/**
 * Object
 * @param devfile   Path name of dev  
 * @param baud      Baudrate 
 * @param parity    Parity
 * @param data_bit  Data bit
 * @param stop_bit  Stop bit 
 */
GGM::Driver_md200::Driver_md200(const char *devfile, unsigned int baud, parity_t parity, data_bits_t data_bit,stop_bits_t stop_bit)
    :  rs485(devfile,baud, parity, data_bit, stop_bit)  
{
    if(!this->check_baudrate(baud))
    {
        ROS_ERROR("DRIVER not support baudrate %u bps", baud);
        this->close_port();
        exit(-1);
    }
    else
    {
        this->err = false;
        this->err_no = 0;
        ROS_INFO("Connected device %s with %u bps", devfile,baud);
    }
} 

/**
 * Check baudrate has supported for device
 * @param baud      Package Array size byte
 * @param return    true/false 
 */
bool GGM::Driver_md200::check_baudrate(unsigned int baud)
{
    bool status = false;
    unsigned int data_defaut[] = {9600, 19200, 38400, 57600, 115200};
    for(uint8_t i = 0; i <  (uint8_t)sizeof(data_defaut)/sizeof(unsigned int); i++)
    {
        if(baud == data_defaut[i])
        {
            status = true;
            break;
        } 
    }
    return status;
}

/**
 * Checksum
 * @param length    Package Array size byte
 * @param to_send   Package Array
 * @param return    byte 
 */
uint8_t GGM::Driver_md200::GetcheckSum(uint length, uint8_t *to_send)
{
   uint8_t byTmp = 0;
   for(short i = 0; i < length; i++ ) byTmp += *(to_send + i);
   return (~byTmp +1);

} //GetcheckSum

/**
 * Set connection is bad
 */
void GGM::Driver_md200::set_bad_con()
{
    this->err = true;
    ROS_ERROR("BAD CONNECTION");
}

/**
 * Modbus Request Builder
 * @param to_send   Message Buffer input
 * @param hd        Header 
 * @param add       ID Number 
 * @param pid       Parameter ID
 * @param num       Data number
 */
void GGM::Driver_md200::buidRequest(uint8_t *to_send,header_t hd ,
                                    uint8_t address, param_id_t pid, dt_num_t num) const
{
    to_send[0] = hd.RMID;
    to_send[1] = hd.TMID;
    to_send[2] = address;
    to_send[3] = pid.PID;
    to_send[4] = num.NUM;
} // buidRequest 

/**
 * Write Request Builder and Sender
 * @param address   Reference Address
 * @param lenght    Amount of data to be Written
 * @param value     Data to Be Written
 */
int GGM::Driver_md200::md200Write(header_t hd ,uint8_t address, param_id_t pid,
                                         dt_num_t num, uint8_t *value)
{
    int status = 0;
    uint lenght = num.NUM + 5;
    uint8_t to_send[lenght+1];
    this->buidRequest(to_send,hd,address,pid,num);
    for(uint8_t i = 0; i < num.NUM; i++) to_send[lenght -num.NUM +i] = value[i];
    to_send[lenght] = this->GetcheckSum(lenght,to_send);
    status = this->sendMsgs(to_send,lenght+1);
    return status;
}

/**
 * Read Request Builder and Sender
 * @param address   Reference Address
 * @param lenght    Amount of Data to Read
 * @param return    EXIT_CODES Type
 */
int GGM::Driver_md200::md200Read(uint8_t *read_buf, ssize_t &num)
{
    ssize_t k =  this->receiveMsgs(read_buf);
    num = k;
    if(k < 0) 
    {
        this->set_bad_con();
        return EXIT_CONFIG_ERROR;
    }
    else{
        return EXIT_OK;
    }
    
}

/**
 * Default setting
 * @param address   ID  
 *                  Data : 0x55(CHECK)
 *                  183, 172, ID, 3, 1, 0x55, CHK
 */
void GGM::Driver_md200::pid_defaut_set(uint8_t address)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 3;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = 0x55u; //Data : 0x55(CHECK)
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Stop naturally
 * @param address   ID  
 *                  Stop motor naturally, data don’t care(x).
 *                  183, 172, ID, 5, 1, x, CHK
 */
void GGM::Driver_md200::pid_tq_off(uint8_t address)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 5;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = 0X01u; //data don’t care(x).
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Erectric brake
 * @param address   ID  
 *                  Stop motor urgently(electric braking mode)
 *                  183, 172, ID, 6, 1, x, CHK
 */
void GGM::Driver_md200::pid_break(uint8_t address)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 6;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = 0x01u; //data don’t care(x).
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Pid command
 * @param address   ID  
 * @param cmd       Contents on CMD number  (forllow page 6-CommunicationProtocolOnRS485_V44)
 *                  183, 172, ID, 10, 1, CMD, CHK
 */
void GGM::Driver_md200::pid_command(uint8_t address, PID_COMMAND cmd)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 10;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = cmd;
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Reset alarm
 * @param address   ID  
 *                  Data don’t care(x)
 *                  183, 172, ID, 12, 1, x, CHK
 */
void GGM::Driver_md200::pid_alarm_reset(uint8_t address)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 12;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = 0X01u; //data don’t care(x).
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Reset position, Motor position to zero
 * @param address   ID  
 *                  Data don’t care(x)
 *                  183, 172, ID, 13, 1, x, CHK
 */
void GGM::Driver_md200::pid_posi_reset(uint8_t address)
{
    header_t hd;        hd.TMID = 172;
    param_id_t pid;     pid.PID = 13;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = 0X01u; //data don’t care(x).
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Request broadcasting of PID_MAIN_DATA
 * @param address   ID  
 * @param data      DATA  1 : PID 193 broadcasting on OR 0 : broasdcasting off
 *                  183, 172, ID, 14, 1, DATA, CHK
 */
void GGM::Driver_md200::pid_main_bc_state(uint8_t address, _BIT data)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 14;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = data; 
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Request BC on/off of PID_MONITOR
 * @param address   ID  
 * @param data      DATA  1 : PID 196 broadcasting on OR 0 : broasdcasting off
 *                  183, 172, ID, 15, 1, DATA, CHK
 */
void GGM::Driver_md200::pid_monitor_bc_state(uint8_t address, _BIT data)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 15;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    value[num.NUM - 1] = data; 
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Save preset position
 * @param address   ID  
 * @param data      DATA  Preset number(address, 1~20)
 *                        Set current position to the preset address
 *                  183, 172, ID, 31, 1, DATA, CHK
 */
void GGM::Driver_md200::pid_preset_save(uint8_t address, uint8_t data)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 31;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    if(data > 0 && data <=20)
    {   
        value[num.NUM - 1] = data; 
        this->md200Write(hd,address,pid,num,value);
    }
    else
    {
        ROS_ERROR("Command 'PID_PRESET_SAVE' element 'data' number must in ranges (1~20). Prefer page 10 CommunicationProtocolOnRS485_V44");
        
    }  
}

/**
 * Go to the recalled preset position
 * @param address   ID  
 * @param data      DATA  Preset number(address, 1~20)
 *                        Recall the saved preset data and move to thatposition.(position control)
 *                  183, 172, ID, 32, 1, DATA, CHK
 */
void GGM::Driver_md200::pid_preset_recall(uint8_t address, uint8_t data)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 32;
    dt_num_t num;       num.NUM = 1;
    uint8_t value[num.NUM];
    if(data > 0 && data <=20)
    {   
        value[num.NUM - 1] = data; 
        this->md200Write(hd,address,pid,num,value);
    }
    else
    {
        ROS_ERROR("Command 'PID_PRESET_RECALL' element 'data' number must in ranges (1~20). Prefer page 10 CommunicationProtocolOnRS485_V44");
    }
    
}

/**
 * Velocity command
 * @param address   ID 
 * @param speed     Speed(rpm) = (D1 | D2<<8) 
 *                  Speed>0, CCW direction
 *                  Speed<0, CW direction
 *                  183, 172, ID, 130, 2, D1, D2, CHK
 */
void GGM::Driver_md200::pid_vel_cmd(uint8_t address, int16_t speed)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 130;
    dt_num_t num;       num.NUM = 2;
    uint8_t value[num.NUM];
    value[num.NUM - 2] = speed & 0x00FFu;
    value[num.NUM - 1] = speed >> 8u;
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Velocity command used more than 25,000rpm
 * @param address   ID  
 * @param high_speed    Speed(rpm) = (D1 | D2<<8) x 10 
 *                      Speed>0, CCW direction
 *                      Speed<0, CW direction
 *                      183, 172, ID, 131, 2, D1, D2, CHK
 */
void GGM::Driver_md200::pid_vel_cmd2(uint8_t address, int16_t high_speed)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 131;
    dt_num_t num;       num.NUM = 2;
    uint8_t value[num.NUM];
    value[num.NUM - 2] = high_speed*10 & 0x00FFu;
    value[num.NUM - 1] = high_speed*10 >> 8u;
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Velocity command used more than 25,000rpm
 * @param address   ID  
 * @param data      D1, D2: Open-loop velocity 
 *                  Range : -1023~1023
 *                  183, 172, ID, 134, 2, D1, D2, CHK
 */
void GGM::Driver_md200::pid_open_vel_cmd(uint8_t address, int16_t data)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 134;
    dt_num_t num;       num.NUM = 2;
    uint8_t value[num.NUM];
    if(data >= -1023 && data <= 1023)
    {
        value[num.NUM - 2] = data & 0x00FFu;
        value[num.NUM - 1] = data >> 8u;
        this->md200Write(hd,address,pid,num,value);
    }
    else
    {
        ROS_ERROR("Command 'PID_PRESET_RECALL' element 'data' number must in ranges (-1023 ~ 1023). Prefer page 10 CommunicationProtocolOnRS485_V44");
    }
}

/**
 * ID setting
 * @param id_setting    ID : 1~253 : setting ID
 *                      Write command(0xaa)
 *                      183, 172, 254, 133, 2, 0xaa, ID, CHK
 */
void GGM::Driver_md200::pid_id_setting(uint8_t id_setting)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 133;
    dt_num_t num;       num.NUM = 2;
    uint8_t value[num.NUM];
    if(id_setting > 0 && id_setting <=253)
    {
        value[num.NUM - 2] = 0xAAu;
        value[num.NUM - 1] = id_setting;
        this->md200Write(hd,254,pid,num,value);
    }
    else
    {
        ROS_ERROR("Command 'PID_ID' element 'id_setting' number must in ranges (1 ~ 253). Prefer page 12 CommunicationProtocolOnRS485_V44");
    }
    
}

/**
 * Set the baudrate, BAUD
 * @param address           ID 
 * @param baudrate_setting  Baudrate setting(RS485)
 *                          Write command(0xaa)
 *                          183, 172, ID, 135, 2, 0xaa, BAUD, CHK
 */
void GGM::Driver_md200::pid_baudrate_setting(uint8_t address, _BAUD_OF_MD200 baudrate_setting)
{
    header_t hd;        hd.TMID = 172; 
    param_id_t pid;     pid.PID = 133;
    dt_num_t num;       num.NUM = 2;
    uint8_t value[num.NUM];
    value[num.NUM - 2] = 0xAAu;
    value[num.NUM - 1] = (uint8_t)baudrate_setting;
    this->md200Write(hd,address,pid,num,value);
}

/**
 * Set slave id 
 * @param ids       Pointer save array slave id  
 * @param lenght    Lenght of ids array (sizeof(ids)/sizeof(uint8_t))
 */
void GGM::Driver_md200::set_slave_id(uint8_t *ids, uint8_t lenght)
{
   uint8_t n = lenght; 
   for(uint8_t i = 0; i < n; i++)
   {
       for(int j = 0; j < n; j++)
       {
           if(ids[j] == ids[i] && j != i)
           {
               ids[j] = ids[j+1];
               n--;
           }
       }
   }
   if(n != lenght)  ROS_WARN("There are %d ID input to be same!!", lenght - n);
   this->slave_id.size = n;
   for(uint8_t k = 0; k < n; k++) this->slave_id.adr[k] = ids[k];
   ROS_INFO("The slave ID get OK");
}

/**
 * Set slave id 
 * @param speed     Speed list input 
 */
void GGM::Driver_md200::publish_speed(int16_t *speed)
{
    if(this->slave_id.size > 0)
    {
        for(uint8_t i = 0; i < this->slave_id.size; i++)
        {
            //std::cout << std::dec << (int)speed[i] << " ";
            if(abs(speed[i]) > 0)
                this->pid_vel_cmd(this->slave_id.adr[i],speed[i]);
            else 
                //this->pid_break(this->slave_id.adr[i]);
                this->pid_tq_off(this->slave_id.adr[i]);
        }
        //std::cout << std::endl;
    }else
    {
        ros::Duration(3).sleep(); // sleep for 3 second
        ROS_ERROR("There are not slave ID of devices GGM Motor");
    }
    
}

/**
 * PID_MAIN_DATA broadcasting ON
 * @param CMD_MAIN_BC_ON 5
 */
void GGM::Driver_md200::main_BC_ON()
{
    if(this->slave_id.size > 0)
    {
        for(uint8_t i = 0; i < this->slave_id.size; i++)
        {
            this->pid_command(this->slave_id.adr[i],CMD_MAIN_BC_ON);
            usleep(30000);
        }
        this->theadEnable = true;
        if(this->ThreadOne.joinable() == false)
        {
            this->ThreadOne = boost::thread(this->cmdMainBCdoStuff,this,&this->rate);
        }
    }else
    {
        ros::Duration(3).sleep(); // sleep for 3 second
        ROS_ERROR("There are not slave ID of devices GGM Motor");
    }
}

/**
 * PID_MAIN_DATA broadcasting OFF
 * @param CMD_MAIN_BC_OFF 6
 */
void GGM::Driver_md200::main_BC_OFF()
{
    if(this->slave_id.size > 0)
    {
        for(uint8_t i = 0; i < this->slave_id.size; i++)
        {
            this->pid_command(this->slave_id.adr[i],CMD_MAIN_BC_OFF);
            usleep(30000);
        }
        this->theadEnable = false;
        if(this->ThreadOne.joinable() == true)
            this->ThreadOne.join();
    }else
    {
        ros::Duration(3).sleep(); // sleep for 3 second
        ROS_ERROR("There are not slave ID of devices GGM Motor");
    }
}

/**
 * Read data 
 * @param payload       This 
 * @param publish_rate  Loop rate
 */
void GGM::Driver_md200::cmdMainBCdoStuff(void *payload, uint8_t *publish_rate)
{
    GGM::Driver_md200 *as = (GGM::Driver_md200 *)payload;
    ros::Rate loop_rate(*publish_rate);
    ROS_INFO("TheadOne to read msg devices motor is ready !!");
    while(ros::ok() && as->theadEnable)
    {
        ssize_t num;
        uint8_t msg[256];
        as->md200Read(msg,num);
        
        as->cs.w_index += num;
        if(as->cs.w_index >= MAX_BUFFER_LENGHT) {
            uint8_t l_part = MAX_BUFFER_LENGHT - (as->cs.w_index - num);
            for(uint8_t i = 0; i < l_part; i++) as->cs._buffer[as->cs.w_index - num + i] = msg[i]; 
            as->cs.w_index = num - l_part;
            for(uint8_t i = l_part; i < num; i++) as->cs._buffer[as->cs.w_index - num + l_part + i] = msg[i]; 
            as->cs.c_flag++; 
        } 
        else {
            for(uint8_t i = 0; i < num; i++) as->cs._buffer[as->cs.w_index - num + i] = msg[i];
        }
        
        if(as->cs.w_index - as->cs.r_index > PACKAGE_MSG_MAIN_DATA || as->cs.c_flag > 0)
        {
            if(as->cs._buffer[as->cs.r_index] == 172 && as->cs._buffer[as->cs.r_index + 1] == 183)
            {    
                int num_data = as->cs._buffer[as->cs.r_index + 4];
                if(as->cs.w_index - as->cs.r_index >= 6 + num_data)
                {
                    as->cs.r_index += 6 + num_data;

                    if(as->cs.r_index >= MAX_BUFFER_LENGHT) 
                    {
                        as->cs.c_flag--;
                        uint8_t l_part = MAX_BUFFER_LENGHT - (as->cs.r_index - PACKAGE_MSG_MAIN_DATA);
                        for(uint8_t i = 0; i < l_part; i++) 
                            as->cs.result[i] = as->cs._buffer[as->cs.r_index - PACKAGE_MSG_MAIN_DATA + i];
                        as->cs.r_index = PACKAGE_MSG_MAIN_DATA - l_part;
                        for(uint8_t i = l_part; i < PACKAGE_MSG_MAIN_DATA; i++) 
                            as->cs.result[i] = as->cs._buffer[as->cs.r_index - PACKAGE_MSG_MAIN_DATA + l_part + i ];
                    }   
                    else 
                    {
                        for(uint8_t i = 0; i < PACKAGE_MSG_MAIN_DATA; i++) 
                            as->cs.result[i] = as->cs._buffer[as->cs.r_index - PACKAGE_MSG_MAIN_DATA + i];
                    }
                    std::cout << "num_data " << std::dec << num_data << " ";
                    // std::cout << "w_index " << std::dec << (int)(as->cs.w_index) << " r_index = " << std::dec << (int)(as->cs.r_index) << " ";
                    // std::cout << " c_flags = " <<  std::dec << (int)(as->cs.c_flag); 
                    //std::cout << " result = "; 
                    // for(int i = 0; i < 22; i++) 
                    // {
                    //     std::cout  << std::hex << (int)as->cs.result[i] << " ";
                    // }
                    std::cout << std:: endl;
                }
            }
            else 
            {
                std::cout << " buffer = "; 
                for(int i = as->cs.r_index; i < as->cs.r_index + 22; i++) 
                {
                    std::cout  << std::hex << (int)as->cs._buffer[i] << " ";
                }
                std::cout << std:: endl;
                as->cs.r_index ++;   
            }
            if(as->cs.c_flag > 1)
            {
                ROS_WARN("ERROR DATA");
                exit(-1);
            }
        }
        std::cout << "w_index = " << std::dec << (int)(as->cs.w_index) << " r_index = " << std::dec << (int)(as->cs.r_index) << " ";
        std::cout << " c_flags = " <<  std::dec << (int)(as->cs.c_flag); 
        // std::cout << " buffer = "; 
        // for(int i = 0; i < 300; i++) 
        // {
        //     std::cout  << std::hex << (int)as->cs._buffer[i] << " ";
        // }
        std::cout << std:: endl;

        loop_rate.sleep();
        ros::spinOnce();
    }

}
