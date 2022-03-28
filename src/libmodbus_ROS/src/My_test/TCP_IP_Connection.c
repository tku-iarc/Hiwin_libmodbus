#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <modbus.h>

#define MODBUS_SERVER_IP    "192.168.0.1"
#define MODBUS_SERVER_PORT  502
#define MODBUS_TIMEOUT_SEC  3
#define MODBUS_TIMEOUT_USEC 0
#define MODBUS_DEBUG        ON

/* MODBUS TCP/IP Connect */
void IP_Connect(TIMEOUT_SEC, TIMEOUT_USEC, DEBUG){
    /* Debug mode */
    modbus_set_debug(ctx, DEBUG);
 
    /* set timeout */
    timeout.tv_sec = TIMEOUT_SEC;
    timeout.tv_usec = TIMEOUT_USEC;
    modbus_get_byte_timeout(ctx, &timeout);

    timeout.tv_sec = TIMEOUT_SEC;
    timeout.tv_usec = TIMEOUT_USEC;
    modbus_set_response_timeout(ctx, &timeout);
 
    if(ctx == NULL){
        fprintf(stderr, "Unable to allocate libmodbus contetx\r\n");
    }

    /* connect */
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connexion failed: %s\r\n", modbus_strerror(errno));
        modbus_free(ctx);
        return(-1);
    }
}

modbus_t *ctx;
/* set device ID */
ctx = modbus_new_tcp(MODBUS_SERVER_IP, MODBUS_SERVER_PORT); // modbus_new_tcp(const char *ip, int port);

int main(int argc, char *argv[])
{
    IP_Connect(MODBUS_TIMEOUT_SEC, MODBUS_TIMEOUT_USEC, MODBUS_DEBUG);
    
    /* Close the connection */
    modbus_close(ctx);
    fprintf("OOOOOOOOKKKKKKKK");
    modbus_free(ctx);
}