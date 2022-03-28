#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
 
#include <modbus.h>
 
#define MODBUS_SERVER_IP       "192.168.0.1"
#define MODBUS_SERVER_PORT     502

#define REGISTERS_ADDRESS      201
#define ROBOT_MOVE_STATE       524
#define MOVE_STATE_LEN         1
#define MAX_READ_REGISTERS     1
// #define MODBUS_TIMEOUT_SEC     3
// #define MODBUS_TIMEOUT_USEC    0

modbus_t        *ctx;
int             wrt;
int             ret;

void Holding_Registers_init(){
    wrt = modbus_write_register(ctx, 201, 6);
    wrt = modbus_write_register(ctx, 200, 0);
}

int libModbus_Connect(){
    /***** set IP and Port *****/
    ctx = modbus_new_tcp(MODBUS_SERVER_IP, MODBUS_SERVER_PORT);
 
    /* Debug mode */
    modbus_set_debug(ctx, TRUE);

    /* set timeout */
    ret = modbus_set_response_timeout(ctx, 1, 0);

    if(ctx == NULL)
    {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
    }

    /********* TCP/IP Connect *********/
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connexion failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
}

void Modbus_Close(){ 
    modbus_free(ctx);
    modbus_close(ctx);
}

int Arm_State_REGISTERS(){
    uint16_t regs[MAX_READ_REGISTERS] = {0};
    ret = modbus_read_input_registers(ctx, ROBOT_MOVE_STATE, MOVE_STATE_LEN, regs);
    return ret;
}

int Read_REGISTERS(int addr){
    uint16_t regs[MAX_READ_REGISTERS] = {0};
    ret = modbus_read_input_registers(ctx, addr, MOVE_STATE_LEN, regs);
    return ret;
}

/************* Discrete Input *************/
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
void DO(int DO_Num, int x){
    wrt = modbus_write_bit(ctx, DO_Num, x);
    wrt = modbus_write_register(ctx, 200, 1);
}

/*************  Holding Register  *************/
void HOME(){  
    int state = 0;
    int arm_state = 0;
    uint16_t home_run[2] = {4, 1};
    uint16_t home_stop[2] = {4, 0};
    wrt = modbus_write_registers(ctx, REGISTERS_ADDRESS, 2, home_run);
    wrt = modbus_write_register(ctx, 200, 1);
}

void PTP(int type, int vel, int acc, int TOOL, int BASE, double *Angle){
    // double Angle[6] = {joint1, joint2, joint3, joint4, joint5, joint6};
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

    wrt = modbus_write_registers(ctx, REGISTERS_ADDRESS, 26, table);
    wrt = modbus_write_register(ctx, 200, 1);
}

void LIN(int type, int vel, int acc, int TOOL, int BASE, double *XYZ){
    // double Angle[6] = {x, y, z, a, b, c};
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

    wrt = modbus_write_registers(ctx, REGISTERS_ADDRESS, 26, table);
    wrt = modbus_write_register(ctx, 200, 1);
}

void CIRC(int vel, int acc, int TOOL, int BASE, double *CIRC_s, double *CIRC_end){
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
    wrt = modbus_write_registers(ctx, REGISTERS_ADDRESS, 37, table);
    wrt = modbus_write_register(ctx, 200, 1);
}

/*********************
    value   joint 
    0~5 -> A1~A6 
    value  Cartesian
    6~11 -> XYZABC 
*********************/
void JOG(int joint,int dir){
    uint16_t table[3] = {3, joint, dir};
    wrt = modbus_write_registers(ctx, REGISTERS_ADDRESS, 3, table);
    wrt = modbus_write_register(ctx, 200, 1);
}