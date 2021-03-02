#pragma once
#include <ros/ros.h>
#include "libserial/rs485.h"
#include "ggm_motor/define.h"
#include <boost/thread/thread.hpp>

GGM_MOTOR_NAMESPACE_BEGIN 
class Driver_md200: public rs485
{
private:
   id_number_t slave_id;
   uint8_t rate = 100;
   boost::thread ThreadOne,ThreadTwo;
   bool theadEnable;
   filter_t cs; 

   virtual uint8_t GetcheckSum(uint length, uint8_t *to_send);
   virtual void buidRequest(uint8_t *to_send,header_t hd ,uint8_t add, param_id_t pid, dt_num_t num) const;
   virtual int md200Read(uint8_t *read_buf, ssize_t &num);
   virtual int md200Write(header_t hd ,uint8_t address, param_id_t pid, dt_num_t num, uint8_t *value);
   virtual bool check_baudrate(unsigned int baud);
   virtual void set_bad_con();
   /* COMMAND */
   virtual void pid_defaut_set(uint8_t address);
   virtual void pid_tq_off(uint8_t address); 
   virtual void pid_break(uint8_t address);
   virtual void pid_command(uint8_t address, PID_COMMAND cmd);
   virtual void pid_alarm_reset(uint8_t address);
   virtual void pid_posi_reset(uint8_t address);
   virtual void pid_main_bc_state(uint8_t address, _BIT data);
   virtual void pid_monitor_bc_state(uint8_t address, _BIT data);
   virtual void pid_preset_save(uint8_t address, uint8_t data);
   virtual void pid_preset_recall(uint8_t address, uint8_t data);
   virtual void pid_vel_cmd(uint8_t address, int16_t speed);
   virtual void pid_vel_cmd2(uint8_t address, int16_t high_speed);
   virtual void pid_open_vel_cmd(uint8_t address, int16_t data);
   /*Write*/
   virtual void pid_id_setting(uint8_t id_setting);
   virtual void pid_baudrate_setting(uint8_t address, _BAUD_OF_MD200 baudrate_setting);
   /* READ */
   static void cmdMainBCdoStuff(void *payload, uint8_t *publish_rate);
public:
   bool err{};
   int err_no{};
  
   virtual void set_slave_id(uint8_t *ids, uint8_t lenght);
   virtual void publish_speed(int16_t *speed);
   virtual void main_BC_ON();
   virtual void main_BC_OFF();

   /*************************************************************************************/
   Driver_md200(const char *devfile, unsigned int baud, parity_t parity, data_bits_t data_bit,stop_bits_t stop_bit);
   virtual ~Driver_md200();  

};
GGM_MOTOR_NAMESPACE_END

