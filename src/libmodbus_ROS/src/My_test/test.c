#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
 
#include <modbus.h>
 
#define MODBUS_SERVER_IP       "192.168.0.1"
#define MODBUS_SERVER_PORT     502

#define REGISTERS_ADDRESS      201
#define ROBOT_MOVE_STATE       524
#define MOVE_STATE_LEN         1
#define MAX_READ_REGISTERS     1

modbus_t        *ctx;
int             wrt;
int             ret;

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
void HOME(int state){
    uint16_t table[2] = {4, state};
    wrt = modbus_write_registers(ctx, REGISTERS_ADDRESS, 2, table);
    wrt = modbus_write_register(ctx, 200, 1);
}

void PTP(int type,double *Angle, int vel, int acc, int TOOL, int BASE){
    // int Angle[6] = {joint1, joint2, joint3, joint4, joint5, joint6};
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

void LIN(int type,double *XYZ, int vel, int acc, int TOOL, int BASE){
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

void CIRC(double *CIRC_s, double *CIRC_end, int vel, int acc, int TOOL, int BASE){
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

int main(int argc, char *argv[])
{
    // int number = 0;

    uint16_t regs[MAX_READ_REGISTERS] = {0};

    double PTP_Angle[6]={0, 0, 0, 0, -90, 90.83}; // ANGLE
    double PTP_XYZ[6]={204.049, 368, 293.5, 180, 0, 90}; // XYZABC
    double LIN_Angle[6]={0, 0, 0, 0, -90, 90}; // ANGLE
    double LIN_XYZ[6]={204.049, 368, 110, 180, 0, 90}; // XYZABC
    double CIRC_s[6]={0, 460.823, 293.5, 180, 0, 90}; // CIRC start point
    double CIRC_end[6]={-204.049, 368, 293.5, 180, 0, 90}; // CIRC end point

    /***** set IP and Port *****/
    ctx = modbus_new_tcp(MODBUS_SERVER_IP, MODBUS_SERVER_PORT);
 
    /* Debug mode */
    modbus_set_debug(ctx, TRUE);

    if(ctx == NULL)
    {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return -1;
    }

    /********* TCP/IP Connect *********/
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connexion failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return(-1);
    }
    
    // DO(300,1);
    // DO(300,0);
    // HOME(1);
    // PTP(0, PTP_Angle, 10, 10, 1, 0); // PTP Jount
    // PTP(1, PTP_XYZ, 50, 10, 1, 0);   // PTP Coordinate
    // LIN(0, Angle, 10, 10, 1, 0); // PTP Jount
    // LIN(1, XYZ, 500, 30, 3, 7);  // LIN Coordinate
    // CIRC(CIRC_s, CIRC_end, 100, 50, 1, 0);
    // JOG(5,0);
    
    // HOME(1);
    // while(1){
    // /**************************************/
    //     ret = modbus_read_input_registers(ctx, ROBOT_MOVE_STATE, MOVE_STATE_LEN, regs);
    //     switch(number){

    //         case 0:
    //             PTP(1, PTP_XYZ, 50, 10, 1, 0);
    //             if(ret == 1)
    //             {
    //                 number++;
    //                 printf("%d\n",number);
    //             }
    //             break;
            
    //         case 1:
    //             // CIRC(CIRC_s,CIRC_end, 600, 10, 1, 0);
    //             LIN(1, LIN_XYZ, 500, 10, 1, 0);  // LIN Coordinate
    //             if(ret == 1)
    //             {
    //                 number++;
    //                 printf("%d\n",number);
    //             }
    //             break;
            
    //         case 2:
    //             // HOME(1);
    //             LIN(1, PTP_XYZ, 500, 10, 1, 0);  // LIN Coordinate
    //             if(ret == 1)
    //             {
    //                 number++;
    //                 printf("%d\n",number);
    //             }
    //             break;

    //         case 3:
    //             HOME(1);
    //             if(ret == 1)
    //             {
    //                 number++;
    //                 printf("%d\n",number);
    //             }
    //             break;
            
    //         default :
    //             printf("default");
    //             break;
    //     }
    // /**************************************/
    // }

    /* Close the connection */
    modbus_close(ctx);
    modbus_free(ctx);
    return(0);
}