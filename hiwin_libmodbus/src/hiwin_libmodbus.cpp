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
  wrt_ = modbus_write_register(ctx_, 201, 6);
  wrt_ = modbus_write_register(ctx_, 200, 0);
}

void HiwinLibmodbus::MOTOR_EXCITE(){
    //motor excite again, makesure it works
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


void HiwinLibmodbus::Arm_State_REGISTERS(int &arm_state){
  uint16_t regs[MAX_READ_REGISTERS] = {0};
  ret_ = modbus_read_input_registers(ctx_, ROBOT_MOVE_STATE, MOVE_STATE_LEN, regs);
  arm_state = static_cast<int>(regs[0]); // convert uint_val to int_val
}


void HiwinLibmodbus::Read_REGISTERS(int addr, int &state){
  uint16_t regs[MAX_READ_REGISTERS] = {0};
  ret_ = modbus_read_input_registers(ctx_, addr, MOVE_STATE_LEN, regs);
  state = static_cast<int>(regs[0]); // convert uint_val to int_val
}

void HiwinLibmodbus::Read_DI(int addr, int &state){
  uint8_t bits[MODBUS_MAX_READ_BITS] = {0};
  ret_ = modbus_read_input_bits(ctx_, addr, 32, bits);
  // if (ret_ < 0) {
  //      fprintf(stderr, "%s\n", modbus_strerror(errno));
  // } else {
  //     printf("BITS COILS:\n");
  //     for (i=0; i < ret_; i++) {
  //         printf("[%d]=%d\n", i, bits[i]);
  //     }
  // }
  state = static_cast<int>(bits[0]); // convert uint_val to int_val
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
void HiwinLibmodbus::DO(int DO_Num, int active){
  wrt_ = modbus_write_bit(ctx_, DO_Num, active);
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  wrt_ = modbus_write_register(ctx_, 200, 1);
}

/*************  Holding Register  *************/
void HiwinLibmodbus::HOME(){  
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t home_run[2] = {4, 1};
  // uint16_t home_stop[2] = {4, 0};
  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 2, home_run);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}


void HiwinLibmodbus::getArmJoints(std::vector<double> &Joints){
    Joints.clear();
    uint16_t regs[MAX_READ_REGISTERS] = {0};
    modbus_read_input_registers(ctx_, 300, MOVE_STATE_LEN, regs);
    A1_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 301, MOVE_STATE_LEN, regs);
    A1_High = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 302, MOVE_STATE_LEN, regs);
    A2_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 303, MOVE_STATE_LEN, regs);
    A2_High = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 304, MOVE_STATE_LEN, regs);
    A3_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 305, MOVE_STATE_LEN, regs);
    A3_High = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 306, MOVE_STATE_LEN, regs);
    A4_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 307, MOVE_STATE_LEN, regs);
    A4_High = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 308, MOVE_STATE_LEN, regs);
    A5_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 309, MOVE_STATE_LEN, regs);
    A5_High = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 310, MOVE_STATE_LEN, regs);
    A6_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 311, MOVE_STATE_LEN, regs);
    A6_High = static_cast<int>(regs[0]);

    if (A1_Low >= 0)
      joint1 = (65536*A1_High + A1_Low)*0.001;
    else
      joint1 = (65536*(A1_High+1) + A1_Low)*0.001;

    if (A2_Low >= 0)
      joint2 = (65536*A2_High + A2_Low)*0.001;
    else
      joint2 = (65536*(A2_High+1) + A2_Low)*0.001;

    if (A3_Low >= 0)
      joint3 = (65536*A3_High + A3_Low)*0.001;
    else
      joint3 = (65536*(A3_High+1) + A3_Low)*0.001;

    if (A4_Low >= 0)
      joint4 = (65536*A4_High + A4_Low)*0.001;
    else
      joint4 = (65536*(A4_High+1) + A4_Low)*0.001;

    if (A5_Low >= 0)
      joint5 = (65536*A5_High + A5_Low)*0.001;
    else
      joint5 = (65536*(A5_High+1) + A5_Low)*0.001;

    if (A6_Low >= 0)
      joint6 = (65536*A6_High + A6_Low)*0.001;
    else
      joint6 = (65536*(A6_High+1) + A6_Low)*0.001;

    Joints.push_back(joint1);
    Joints.push_back(joint2);
    Joints.push_back(joint3);
    Joints.push_back(joint4);
    Joints.push_back(joint5);
    Joints.push_back(joint6);

}

void HiwinLibmodbus::getArmPose(std::vector<double> &Pose){
    Pose.clear();
    uint16_t regs[MAX_READ_REGISTERS] = {0};
    modbus_read_input_registers(ctx_, 400, MOVE_STATE_LEN, regs);
    X_Low   = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 401, MOVE_STATE_LEN, regs);
    X_High  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 402, MOVE_STATE_LEN, regs);
    Y_Low   = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 403, MOVE_STATE_LEN, regs);
    Y_High  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 404, MOVE_STATE_LEN, regs);
    Z_Low   = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 405, MOVE_STATE_LEN, regs);
    Z_High  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 406, MOVE_STATE_LEN, regs);
    RX_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 407, MOVE_STATE_LEN, regs);
    RX_High = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 408, MOVE_STATE_LEN, regs);
    RY_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 409, MOVE_STATE_LEN, regs);
    RY_High = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 410, MOVE_STATE_LEN, regs);
    RZ_Low  = static_cast<int>(regs[0]);
    modbus_read_input_registers(ctx_, 411, MOVE_STATE_LEN, regs);
    RZ_High = static_cast<int>(regs[0]);


    if (X_Low >= 0)
      posex = (65536*X_High + X_Low)*0.001;
    else
      posex = (65536*(X_High+1) + X_Low)*0.001;

    if (Y_Low >= 0)
      posey = (65536*Y_High + Y_Low)*0.001;
    else
      posey = (65536*(Y_High+1) + Y_Low)*0.001;

    if (Z_Low >= 0)
      posez = (65536*Z_High + Z_Low)*0.001;
    else
      posez = (65536*(Z_High+1) + Z_Low)*0.001;

    if (RX_Low >= 0)
      rx = (65536*RX_High + RX_Low)*0.001;
    else
      rx = (65536*(RX_High+1) + RX_Low)*0.001;

    if (RY_Low >= 0)
      ry = (65536*RY_High + RY_Low)*0.001;
    else
      ry = (65536*(RY_High+1) + RY_Low)*0.001;

    if (RZ_Low >= 0)
      rz = (65536*RZ_High + RZ_Low)*0.001;
    else
      rz = (65536*(RZ_High+1) + RZ_Low)*0.001;

    Pose.push_back(posex);
    Pose.push_back(posey);
    Pose.push_back(posez);
    Pose.push_back(rx);
    Pose.push_back(ry);
    Pose.push_back(rz);

}

void HiwinLibmodbus::PTP(uint16_t type, uint16_t vel, uint16_t acc, uint16_t TOOL, uint16_t BASE, const std::vector<double> GOAL){
  const double* goal = &GOAL[0];
  return PTP(type, vel, acc, TOOL, BASE, goal);
}
void HiwinLibmodbus::PTP(uint16_t type, uint16_t vel, uint16_t acc, uint16_t TOOL, uint16_t BASE, const double *GOAL){
  // double Angle[6] = {joint1, joint2, joint3, joint4, joint5, joint6};
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t A_L[6] = {0};
  uint16_t A_H[6] = {0};
  double num = 0;

  for (int i = 0; i < 6; i++)
  {
    num = GOAL[i] - (int)GOAL[i];
    if (GOAL[i] >= 0)
    {
      // angle > 0 ex. 90
      A_L[i] = ((int)GOAL[i]*1000)%65536;
      A_L[i] = A_L[i] + (num*1000);
      if (A_L[i] > 32767)
      {
        A_L[i] = (((int)GOAL[i]*1000)%65536)-65536;
        A_L[i] = A_L[i] + (num*1000);
      }
      A_H[i] = (GOAL[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      A_L[i] = (((int)GOAL[i]*1000)%65536)+65536;
      A_L[i] = A_L[i] + (num*1000);
      A_H[i] = ((GOAL[i]*1000)/65536)-1;
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

void HiwinLibmodbus::LIN(uint16_t type, uint16_t vel, uint16_t acc, uint16_t TOOL, uint16_t BASE, const std::vector<double> GOAL){
  const double* goal = &GOAL[0];
  return LIN(type, vel, acc, TOOL, BASE, goal);
}
void HiwinLibmodbus::LIN(uint16_t type, uint16_t vel, uint16_t acc, uint16_t TOOL, uint16_t BASE, const double *GOAL){
  // double Angle[6] = {x, y, z, a, b, c};
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t A_L[6] = {0};
  uint16_t A_H[6] = {0};
  double num = 0;

  for (int i = 0; i < 6; i++)
  {
    num = GOAL[i] - (int)GOAL[i];
    if (GOAL[i] >= 0)
    {
      // angle > 0 ex. 90
      A_L[i] = ((int)GOAL[i]*1000)%65536;
      A_L[i] = A_L[i] + (num*1000);
      if (A_L[i] > 32767)
      {
        A_L[i] = (((int)GOAL[i]*1000)%65536)-65536;
        A_L[i] = A_L[i] + (num*1000);
      }
      A_H[i] = (GOAL[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      A_L[i] = (((int)GOAL[i]*1000)%65536)+65536;
      A_L[i] = A_L[i] + (num*1000);
      A_H[i] = ((GOAL[i]*1000)/65536)-1;
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


void HiwinLibmodbus::CIRC(uint16_t vel, uint16_t acc, uint16_t TOOL, uint16_t BASE, const std::vector<double> CIRC_s, const std::vector<double> CIRC_end){
  const double* circ_s = &CIRC_s[0];
  const double* circ_end = &CIRC_end[0];
  return CIRC(vel, acc, TOOL, BASE, circ_s, circ_end);
}
void HiwinLibmodbus::CIRC(uint16_t vel, uint16_t acc, uint16_t TOOL, uint16_t BASE, const double *CIRC_s, const double *CIRC_end){
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t start_L[6] = {0};
  uint16_t start_H[6] = {0};
  uint16_t end_L[6] = {0};
  uint16_t end_H[6] = {0};
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
void HiwinLibmodbus::JOG(uint16_t joint,uint16_t dir){
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t table[3] = {3, joint, dir};
  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 3, table);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}

void HiwinLibmodbus::SET_BASE(uint16_t base_num, const std::vector<double> POSE){
  const double* pose = &POSE[0];
  return SET_BASE(base_num, pose);
}
void HiwinLibmodbus::SET_BASE(uint16_t base_num, const double *POSE){
  // double Angle[6] = {joint1, joint2, joint3, joint4, joint5, joint6};
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t A_L[6] = {0};
  uint16_t A_H[6] = {0};
  double num = 0;

  for (int i = 0; i < 6; i++)
  {
    num = POSE[i] - (int)POSE[i];
    if (POSE[i] >= 0)
    {
      // angle > 0 ex. 90
      A_L[i] = ((int)POSE[i]*1000)%65536;
      A_L[i] = A_L[i] + (num*1000);
      if (A_L[i] > 32767)
      {
        A_L[i] = (((int)POSE[i]*1000)%65536)-65536;
        A_L[i] = A_L[i] + (num*1000);
      }
      A_H[i] = (POSE[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      A_L[i] = (((int)POSE[i]*1000)%65536)+65536;
      A_L[i] = A_L[i] + (num*1000);
      A_H[i] = ((POSE[i]*1000)/65536)-1;
    }
  }
  
  uint16_t table[14] = {102, base_num, 
                          A_L[0], A_H[0], 
                          A_L[1], A_H[1], 
                          A_L[2], A_H[2], 
                          A_L[3], A_H[3], 
                          A_L[4], A_H[4], 
                          A_L[5], A_H[5],};

  /*********************** test ***********************/
  // uint16_t table[26] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                       vel, acc, TOOL, BASE, 0, 0, 0, 0, 0, 0, 0, 0};
  /*********************** test ***********************/

  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 14, table);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}

void HiwinLibmodbus::SET_TOOL(uint16_t tool_num, const std::vector<double> POSE){
  const double* pose = &POSE[0];
  return SET_BASE(tool_num, pose);
}
void HiwinLibmodbus::SET_TOOL(uint16_t tool_num, const double *POSE){
  // double Angle[6] = {joint1, joint2, joint3, joint4, joint5, joint6};
  // TODO: the arguments should be defined, 
  // number can't be directly written here, people will confused what it is.
  uint16_t A_L[6] = {0};
  uint16_t A_H[6] = {0};
  double num = 0;

  for (int i = 0; i < 6; i++)
  {
    num = POSE[i] - (int)POSE[i];
    if (POSE[i] >= 0)
    {
      // angle > 0 ex. 90
      A_L[i] = ((int)POSE[i]*1000)%65536;
      A_L[i] = A_L[i] + (num*1000);
      if (A_L[i] > 32767)
      {
        A_L[i] = (((int)POSE[i]*1000)%65536)-65536;
        A_L[i] = A_L[i] + (num*1000);
      }
      A_H[i] = (POSE[i]*1000)/65536;
    }else
    {
      // angle < 0 ex.-90
      A_L[i] = (((int)POSE[i]*1000)%65536)+65536;
      A_L[i] = A_L[i] + (num*1000);
      A_H[i] = ((POSE[i]*1000)/65536)-1;
    }
  }
  
  uint16_t table[14] = {101, tool_num, 
                          A_L[0], A_H[0], 
                          A_L[1], A_H[1], 
                          A_L[2], A_H[2], 
                          A_L[3], A_H[3], 
                          A_L[4], A_H[4], 
                          A_L[5], A_H[5],};

  /*********************** test ***********************/
  // uint16_t table[26] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                       vel, acc, TOOL, BASE, 0, 0, 0, 0, 0, 0, 0, 0};
  /*********************** test ***********************/

  wrt_ = modbus_write_registers(ctx_, REGISTERS_ADDRESS, 14, table);
  wrt_ = modbus_write_register(ctx_, 200, 1);
}
// }  // namespace hiwin_libmodbus