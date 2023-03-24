#include <iostream>

#include "hiwin_libmodbus/hiwin_libmodbus.hpp"

extern "C" {

HiwinLibmodbus* HiwinLibmodbusConstructor()
{
  return new HiwinLibmodbus();
}

void HiwinLibmodbusDestructor(HiwinLibmodbus* ptr)
{
  delete ptr;
}

void Holding_Registers_init(HiwinLibmodbus* ptr){
  ptr->Holding_Registers_init();
}

int libModbus_Connect(HiwinLibmodbus* ptr, const wchar_t * ip_address){
  return ptr->libModbus_Connect(ip_address);
}

void Modbus_Close(HiwinLibmodbus* ptr){ 
  ptr->Modbus_Close();
}

// uint16_t* Arm_State_REGISTERS(HiwinLibmodbus* ptr){
//   // return ptr->Arm_State_REGISTERS();
// }

// uint16_t* Read_REGISTERS(HiwinLibmodbus* ptr, int addr){
//   return ptr->Read_REGISTERS(addr);
// }
void DO(HiwinLibmodbus* ptr, int DO_Num, int x){
  ptr->DO(DO_Num, x);
}

void HOME(HiwinLibmodbus* ptr){  
  ptr->HOME();
}

void PTP(HiwinLibmodbus* ptr, 
  int type, int vel, int acc, int TOOL, int BASE, double *Angle)
{
  ptr->PTP(type, vel, acc, TOOL, BASE, Angle);
}

void LIN(HiwinLibmodbus* ptr, 
  int type, int vel, int acc, int TOOL, int BASE, double *XYZ)
{
  ptr->LIN(type, vel, acc, TOOL, BASE, XYZ);
}

void CIRC(HiwinLibmodbus* ptr, 
  int vel, int acc, int TOOL, int BASE, double *CIRC_s, double *CIRC_end)
{
  ptr->CIRC(vel, acc, TOOL, BASE, CIRC_s, CIRC_end);
}

void JOG(HiwinLibmodbus* ptr, int joint, int dir){
  ptr->JOG(joint, dir);
}

} // end of extern "C"