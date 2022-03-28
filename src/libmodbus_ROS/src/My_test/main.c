#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>
#include <time.h>
const uint16_t UT_INPUT_REGISTERS_ADDRESS = 0x1;
const uint16_t UT_BITS_ADDRESS = 0x04;
const uint16_t UT_INPUT_REGISTERS_NB = 0xA;
const uint16_t UT_INPUT_REGISTERS_TAB[] = { 0x000A };
int main(int argc, char const *argv[])
{
    int nb = 0x25;
    int rc = 0;
    modbus_t *ctx;
    uint8_t *tab_rp_bits;
    tab_rp_bits = (uint8_t *) malloc(nb * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb * sizeof(uint8_t));
    ctx = modbus_new_tcp("192.168.0.1", 502);
    if(ctx == NULL)
    {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return -1;
    }

    if(modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    while(1)
    {
        rc = modbus_write_bit(ctx, UT_BITS_ADDRESS, 1);
        if (rc != 1) 
        {
            printf("FAILED (nb points %d)\n", rc);
        }
        rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, 1, tab_rp_bits);
        printf("modbus_read_bits 1 \n modbus_read_bits: \n");
        if (rc != 1) 
        {
            printf("FAILED (nb points %d)\n", rc);

        }
        printf("tab_rp_bits [0] is %d\n",tab_rp_bits[0]);
        memset(tab_rp_bits, 0, nb * sizeof(uint8_t));
        sleep(1);
        rc = modbus_write_bit(ctx, UT_BITS_ADDRESS, 0);
        if (rc != 1) 
        {
            printf("FAILED (nb points %d)\n", rc);
        }
        rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, 1, tab_rp_bits);
        printf("modbus_read_bits 0 \n modbus_read_bits: \n");
        if (rc != 1) 
        {
            printf("FAILED (nb points %d)\n", rc);
        }
        printf("tab_rp_bits [0] is %d\n",tab_rp_bits[0]);
        sleep(1);
    }
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}
