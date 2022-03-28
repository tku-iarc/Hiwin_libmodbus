from calendar import c
from operator import mod
import numpy as np
from ctypes import *
import time
import rospy
'''
    DO(int DO_Num, int x)                                                         # 1 -> on ; 0 -> off                                          
    HOME(int state)                                                               # 1 RUN
    PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle)            # 0 -> joint ; 1 -> coordinate
    LIN(int type,double *XYZ, int vel, int acc, int TOOL, int BASE)               # 0 -> joint ; 1 -> coordinate
    CIRC(double *CIRC_s, double *CIRC_end, int vel, int acc, int TOOL, int BASE) 
    JOG(int joint,int dir)
'''

so_file = "./Hiwin_API.so"
modbus = CDLL(so_file)


if __name__ == "__main__":
    Arm_state = 0
    PTP_Angle = [0, 0, 0, 0, -90, 0]                 # ANGLE
    PTP_XYZ   = [204.049, 368, 293.5, 180, 0, 90]    # XYZABC
    LIN_Angle = [0, 0, 0, 0, -90, 90]                # ANGLE
    LIN_XYZ   = [204.049, 368, 110, 180, 0, 90]      # XYZABC
    CIRC_centre = [0, 460.823, 293.5, 180, 0, 90]      # CIRC centre point
    CIRC_end  = [-204.049, 368, 293.5, 180, 0, 90]   # CIRC end point
    IO_Port = 301 # D0
    
    
    C_PTP_Angle = (c_double * len(PTP_Angle))(*PTP_Angle)       # C Array
    C_PTP_XYZ = (c_double * len(PTP_XYZ))(*PTP_XYZ)             # C Array
    C_CIRC_centre = (c_double * len(CIRC_centre))(*CIRC_centre) # C Array
    C_CIRC_end = (c_double * len(CIRC_end))(*CIRC_end)          # C Array
    
    modbus.DO.argtypes = [c_int, c_int]
    modbus.PTP.argtypes = [c_int, c_int, c_int, c_int, c_int]
    modbus.CIRC.argtypes = [c_int, c_int, c_int, c_int]

    # while 1:
    modbus.libModbus_Connect()
    modbus.Holding_Registers_init()

    # modbus.PTP(0, 10, 10, 1, 0, C_PTP_Angle)
    # modbus.CIRC(10, 10, 1, 0, C_CIRC_centre, C_CIRC_end)

    # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
    while 1:
        # rospy.init_node('libmodbus_ROS')

        # modbus.Holding_Registers_init()
        # modbus.HOME() # 1 RUN
        # modbus.PTP(0, 200, 10, 1, 0, C_PTP_Angle)
        # modbus.DO(IO_Port,0) # 1 -> on ; 0 -> off
        if(modbus.Arm_State_REGISTERS() == 1):
            break

    modbus.Modbus_Close()
    print("Modbus Close")  

    

