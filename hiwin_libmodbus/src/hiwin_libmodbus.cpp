#include <iostream>
#include "hiwin_libmodbus/hiwin_libmodbus.hpp"

// namespace hiwin_libmodbus
// {

HiwinLibmodbus::HiwinLibmodbus()
{
}

HiwinLibmodbus::~HiwinLibmodbus()
{
}

void HiwinLibmodbus::Holding_Registers_init(){
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  std::cout<<"999999999999999999999999999"<<std::endl;
  wrt_ = modbus_write_register(ctx_, 201, 6);
  wrt_ = modbus_write_register(ctx_, 200, 0);
}

int HiwinLibmodbus::libModbus_Connect(const std::string& ip_address){
  return libModbus_Connect(ip_address.c_str());
}

int HiwinLibmodbus::libModbus_Connect(const wchar_t * ip_address){
  std::wstring ip_adr(ip_address);
  std::cout<<std::string(ip_adr.begin(), ip_adr.end())<<std::endl;
  return libModbus_Connect(std::string(ip_adr.begin(), ip_adr.end()).c_str());
}

int HiwinLibmodbus::libModbus_Connect(const char *ip_address){
  /***** set IP and Port *****/
  ctx_ = modbus_new_tcp(ip_address, MODBUS_SERVER_PORT);
 
  /* Debug mode */
  modbus_set_debug(ctx_, TRUE);

  /* set timeout */
  ret_ = modbus_set_response_timeout(ctx_, 1, 0);

  if(ctx_ == NULL)
  {
    fprintf(stderr, "Unable to allocate libmodbus context\n");
  }

  /********* TCP/IP Connect *********/
  if (modbus_connect(ctx_) == -1) {
    fprintf(stderr, "Connexion failed: %s\n", modbus_strerror(errno));
    modbus_free(ctx_);
    return 0;
  }
  return 1;
}

void HiwinLibmodbus::Modbus_Close(){ 
  modbus_free(ctx_);
  modbus_close(ctx_);
}

int HiwinLibmodbus::Arm_State_REGISTERS(){
  uint16_t regs[MAX_READ_REGISTERS] = {0};
  ret_ = modbus_read_input_registers(ctx_, ROBOT_MOVE_STATE, MOVE_STATE_LEN, regs);
  return ret_;
}

int HiwinLibmodbus::Read_REGISTERS(int addr){
  uint16_t regs[MAX_READ_REGISTERS] = {0};
  ret_ = modbus_read_input_registers(ctx_, addr, MOVE_STATE_LEN, regs);
  return ret_;
}

/************* Discret_e Input *************/
  /*********************************
        S0
  value:0 ~ 255 -> S0[1] ~ [256]
    0 or 1    -> R 
  ----------------------------------
        DI
  value:300 ~ 555 -> DI[1] ~ [256]
    0 or 1      -> R
  **********************************/

  /************* Coil *************/
  /*********************************
        DI
  value:0 ~ 255  -> DI[1] ~ [256]
    0 or 65280 -> R/W 
  ----------------------------------
        DO
  value:300 ~ 555 -> DO[1] ~ [256]
    0 or 65280  -> R/W 
  **********************************/
void HiwinLibmodbus::DO(int DO_Num, int x){
  wrt_ = modbus_write_bit(ctx_, DO_Num, x);
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  wrt_ = modbus_write_register(ctx_, 200, 1);
}

/*************  Holding Register  *************/
void HiwinLibmodbus::HOME(){  
  int state = 0;
  int arm_state = 0;
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t home_run[2] = {4, 1};
  uint16_t home_stop[2] = {4, 0};
  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 2, home_run);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}
// void HiwinLibmodbus::PTP(int type, int vel, int acc, int TOOL, int BASE, std::vector<double> Angle){
  
//   return PTP(type, vel, acc, TOOL, BASE, Angle);
// }
void HiwinLibmodbus::PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle){
  // double Angle[6] = {joint1, joint2, joint3, joint4, joint5, joint6};
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  double A_L[6] = {0};
  double A_H[6] = {0};
  double num = 0;

  for (int i = 0; i < 6; i++)
  {
    num = Angle[i] - (int)Angle[i];
    if (Angle[i] >= 0)
    {
      // angle > 0 ex. 90
      A_L[i] = ((int)Angle[i]*1000)%65536;
      A_L[i] = A_L[i] + (num*1000);
      if (A_L[i] > 32767)
      {
        A_L[i] = (((int)Angle[i]*1000)%65536)-65536;
        A_L[i] = A_L[i] + (num*1000);
      }
      A_H[i] = (Angle[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      A_L[i] = (((int)Angle[i]*1000)%65536)+65536;
      A_L[i] = A_L[i] + (num*1000);
      A_H[i] = ((Angle[i]*1000)/65536)-1;
    }
  }
  
  uint16_t table[26] = {0, type, A_L[0], A_H[0], A_L[1], A_H[1], A_L[2], A_H[2], A_L[3], A_H[3], A_L[4], A_H[4], A_L[5], A_H[5],
              vel, acc, TOOL, BASE, 0, 0, 0, 0, 0, 0, 0, 0};

  /*********************** test ***********************/
  // uint16_t table[26] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                       vel, acc, TOOL, BASE, 0, 0, 0, 0, 0, 0, 0, 0};
  /*********************** test ***********************/

  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 26, table);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}

// void HiwinLibmodbus::LIN(int type, int vel, int acc, int TOOL, int BASE, std::vector<double> XYZ){
  
//   return LIN(type, vel, acc, TOOL, BASE, XYZ);
// }
void HiwinLibmodbus::LIN(int type, int vel, int acc, int TOOL, int BASE, double *XYZ){
  // double Angle[6] = {x, y, z, a, b, c};
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  double A_L[6] = {0};
  double A_H[6] = {0};
  double num = 0;

  for (int i = 0; i < 6; i++)
  {
    num = XYZ[i] - (int)XYZ[i];
    if (XYZ[i] >= 0)
    {
      // angle > 0 ex. 90
      A_L[i] = ((int)XYZ[i]*1000)%65536;
      A_L[i] = A_L[i] + (num*1000);
      if (A_L[i] > 32767)
      {
        A_L[i] = (((int)XYZ[i]*1000)%65536)-65536;
        A_L[i] = A_L[i] + (num*1000);
      }
      A_H[i] = (XYZ[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      A_L[i] = (((int)XYZ[i]*1000)%65536)+65536;
      A_L[i] = A_L[i] + (num*1000);
      A_H[i] = ((XYZ[i]*1000)/65536)-1;
    }
  }

  uint16_t table[26] = {1, type, A_L[0], A_H[0], A_L[1], A_H[1], A_L[2], A_H[2], A_L[3], A_H[3], A_L[4], A_H[4], A_L[5], A_H[5],
              vel, acc, TOOL, BASE, 0, 0, 0, 0, 0, 0, 0, 0};

  /*********************** test ***********************/
  // uint16_t table[26] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                       vel, acc, TOOL, BASE, 0, 0, 0, 0, 0, 0, 0, 0};
  /*********************** test ***********************/

  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 26, table);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}

void HiwinLibmodbus::MOTOR_EXCITE(){
  std::cout<<"================================="<<std::endl;
  wrt_ = modbus_write_register(ctx_, 200, 0);
}

// void HiwinLibmodbus::CIRC(int vel, int acc, int TOOL, int BASE, std::vector<double> CIRC_s, std::vector<double> CIRC_end){
//   return CIRC(vel, acc, TOOL, BASE, CIRC_s, CIRC_end);
// }
void HiwinLibmodbus::CIRC(int vel, int acc, int TOOL, int BASE, double *CIRC_s, double *CIRC_end){
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  double start_L[6] = {0};
  double start_H[6] = {0};
  double end_L[6] = {0};
  double end_H[6] = {0};
  double num1 = 0;
  double num2 = 0;

  for (int i = 0; i < 6; i++)
  {
    num1 = CIRC_s[i] - (int)CIRC_s[i];
    num2 = CIRC_end[i] - (int)CIRC_end[i];
    if (CIRC_s[i] >= 0)
    {
      // angle > 0 ex. 90
      start_L[i] = ((int)CIRC_s[i]*1000)%65536;
      start_L[i] = start_L[i] + (num1*1000);
      if (start_L[i] > 32767 || end_L[i] > 32767)
      {
        start_L[i] = (((int)CIRC_s[i]*1000)%65536)-65536;
        start_L[i] = start_L[i] + (num1*1000);
      }
      start_H[i] = (CIRC_s[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      start_L[i] = (((int)CIRC_s[i]*1000)%65536)+65536;
      start_L[i] = start_L[i] + (num1*1000);
      start_H[i] = ((CIRC_s[i]*1000)/65536)-1;
    }
    if (CIRC_end[i]>= 0)
    {
      // angle > 0 ex. 90
      end_L[i] = ((int)CIRC_end[i]*1000)%65536;
      end_L[i] = end_L[i] + (num2*1000);
      if (end_L[i] > 32767)
      {
        end_L[i] = (((int)CIRC_end[i]*1000)%65536)-65536;
        end_L[i] = end_L[i] + (num2*1000);
      }
      end_H[i] = (CIRC_end[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      end_L[i] = (((int)CIRC_end[i]*1000)%65536)+65536;
      end_L[i] = end_L[i] + (num2*1000);
      end_H[i] = ((CIRC_end[i]*1000)/65536)-1;
    }
  }

  uint16_t table[37] = {2, 
              start_L[0], start_H[0], start_L[1], start_H[1], start_L[2], start_H[2], start_L[3], start_H[3], start_L[4], start_H[4], start_L[5], start_H[5],
              end_L[0], end_H[0], end_L[1], end_H[1], end_L[2], end_H[2], end_L[3], end_H[3], end_L[4], end_H[4], end_L[5], end_H[5],
              vel, acc, TOOL, BASE, 0, 0, 0, 0, 0, 0, 0, 0};
  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 37, table);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}

/*********************
  value   joint 
  0~5 -> A1~A6 
  value  Cartesian
  6~11 -> XYZABC 
*********************/
void HiwinLibmodbus::JOG(int joint,int dir){
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t table[3] = {3, joint, dir};
  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 3, table);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}
// }  // namespace hiwin_libmodbus
