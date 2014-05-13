#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/epoll.h>

#include "servo.h"

#ifndef PORT
    #define PORT "/dev/i2c-2"
#endif
#ifndef ADDR
    #define ADDR 0x52
#endif

#define error(x...) {fprintf(stderr, "\nE%d: ", __LINE__); fprintf(stderr, x); fprintf(stderr, "\n\n"); exit(1); }

struct nunchuck_packet {
    unsigned char js[2];
    unsigned char button[2];
    unsigned char acc[3];
};

int main(int argc, char* argv[])
{

    /* Servo motors */
    servo motor_1 = {.header = 8, .pin = 13};
    servo_init(&motor_1);

    // Initialization: open port
    int fd = open(PORT, O_RDWR);
    if (fd < 0)
        error("can't open %s - %m", PORT);
    if (ioctl(fd, I2C_SLAVE, ADDR) < 0)
        error("can't ioctl %s:0x%02x - %m", PORT, ADDR);
    if (write(fd, "\x40", 2) < 0)
        error("can't setup %s:0x%02x - %m", PORT, ADDR);

    //loop: Read 6 bytes, parse and print
    struct nunchuck_packet packet;
    int js_x, js_y, n;

    while (1) {
        // read one byte (at index i)
        n = read(fd, &packet, sizeof(struct nunchuck_packet));

        if (n<0)
            error("read error %s:0x%02x - %m", PORT, ADDR);
        if (!n)
            continue;
        // Decode incoming byte
        js_x = (packet.js[0] ^ 0x17) + 0x17;
        js_y = (packet.js[1] ^ 0x17) + 0x17;
        printf("joystick x: %d y:%d\n", js_x, js_y);

        write(fd, "", 1);
    }

    return 0;
}
