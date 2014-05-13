/*
 * mytest.c
 */

/******************************************************************************
* Include Files                                                               *
******************************************************************************/

// Standard header files
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include <linux/i2c-dev.h>

// Driver header file
#include "prussdrv.h"
#include "pruss_intc_mapping.h"

/******************************************************************************
* Explicit External Declarations                                              *
******************************************************************************/

/******************************************************************************
* Local Macro Declarations                                                    *
******************************************************************************/

#define PRU_NUM 	 0
//number of PRU cycles to achieve 50Hz
#define PERIOD_CYCLES	 	 3200000
//PWM value of 1060us
#define DUTY_CYCLES		 212000
//trigger value to tell the PRU to stop
#define STOP_FLAG		 0

//memory addresses
#define DDR_BASEADDR     0x80000000
#define OFFSET_DDR	 0x00001000
#define OFFSET_SHAREDRAM 2048		//equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4

#define PORT "/dev/i2c-2"
#define ADDR 0x52
/******************************************************************************
* Local Typedef Declarations                                                  *
******************************************************************************/


/******************************************************************************
* Local Function Declarations                                                 *
******************************************************************************/
#define error(x...) {fprintf(stderr, "\nE%d: ", __LINE__); fprintf(stderr, x); fprintf(stderr, "\n\n"); exit(1); }

static int LOCAL_exampleInit ( );
static unsigned short LOCAL_examplePassed ( unsigned short pruNum );

/******************************************************************************
* Local Variable Definitions                                                  *
******************************************************************************/
struct nunchuck_packet {
    unsigned char joystick_val[2];
    unsigned char joystick_button[2];
    unsigned char accel[3];
};


/******************************************************************************
* Intertupt Service Routines                                                  *
******************************************************************************/


/******************************************************************************
* Global Variable Definitions                                                 *
******************************************************************************/

static int mem_fd;
static void *ddrMem, *sharedMem;

static unsigned int *sharedMem_int;

/******************************************************************************
* Global Function Definitions                                                 *
******************************************************************************/

void intHandler(int dummy){
    exit(1);
}

static int linear_map(int val, int fromMin, int fromMax, int toMin, int toMax) {
    return (val - fromMin) * (toMax - toMin) / (fromMax-fromMin) + toMin ;
}

int main (int argc, char* argv[])
{

    unsigned int ret;
    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    printf("\nINFO: Starting single channel PWM on P9_27.\r\n");
    /* Initialize the PRU */
    prussdrv_init ();

    /* Open PRU Interrupt */
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret)
    {
        printf("prussdrv_open open failed\n");
        return (ret);
    }

    /* Get the interrupt initialized */
    prussdrv_pruintc_init(&pruss_intc_initdata);

    /* Initialize example */
    printf("\tINFO: Initializing PWM with default values.\r\n");
    //LOCAL_exampleInit(PRU_NUM);


//////////////////// LOAD VALUES INTO MEMORY /////////////////
    void *DDR_regaddr1, *DDR_regaddr2, *DDR_regaddr3;

    /* open the device */
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return 1;
    }

    /* map the DDR memory */
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, DDR_BASEADDR);
    if (ddrMem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return 1;
    }

    /* Store Addends in DDR memory location */
    DDR_regaddr1 = ddrMem + OFFSET_DDR;
    DDR_regaddr2 = ddrMem + OFFSET_DDR + 0x00000004;
    DDR_regaddr3 = ddrMem + OFFSET_DDR + 0x00000008;

    *(unsigned int*) DDR_regaddr1 = PERIOD_CYCLES;
    *(unsigned int*) DDR_regaddr2 = DUTY_CYCLES;
    *(unsigned int*) DDR_regaddr3 = STOP_FLAG;

    /* Execute example on PRU */
    printf("\tINFO: Starting PWM output on P9_27.\r\n");
    prussdrv_exec_program (PRU_NUM, "./prucode.bin");

    signal(SIGINT, intHandler);

    int fd = open(PORT, O_RDWR);
    if (fd < 0)
        error("can't open %s - %m", PORT);
    if (ioctl(fd, I2C_SLAVE, ADDR) < 0)
        error("can't ioctl %s:0x%02x - %m", PORT, ADDR);
    if (write(fd, "\x40", 2) < 0)
        error("can't setup %s:0x%02x - %m", PORT, ADDR);

    // again, keep supposedly safe values for start (although this might arm some motors)
    int last_input = 1060;
    int input = 1060;
    int converted = 212000;
    int loopkey = 1;

    struct nunchuck_packet packet;
    int js_x, js_y;

    while (loopkey) {

        *(unsigned int*) DDR_regaddr2 = converted;
        printf("Value entered: %i\n Value sent: %i\n", input, converted);

        ret = read(fd, &packet, sizeof(struct nunchuck_packet));
        if (ret <0)
            error("read error: %s : 0x%02x - %m ", PORT, ADDR);
        if (!ret)
            continue;

        js_x = (packet.joystick_val[0] ^ 0x17) + 0x17 ;
        js_y = (packet.joystick_val[1] ^ 0x17) + 0x17 ;
        input = linear_map(js_x, 0, 255, 0, 3000);
        printf("input : %d\n", input);
        last_input = input;

        //scanf(" %i", &loopkey);
        if(input > 100 && input < 10000){
            // sane value
            converted = input * 1000 / 5;  //input in microseconds converted to nanoseconds and then divided by 5 to get cycles
        } else {
            // restore sanity
            input = last_input;
        }

        write(fd, "", 1);
    }
    *(unsigned int*) DDR_regaddr3 = 1;  // set close register, wait for PRU to halt

    /* Wait until PRU0 has finished execution */
    printf("\tINFO: Waiting for HALT command.\r\n");
    prussdrv_pru_wait_event (PRU_EVTOUT_0);
    printf("\tINFO: PRU completed transfer.\r\n");
    prussdrv_pru_clear_event (PRU0_ARM_INTERRUPT);

    /*
    // Check if example passed
    if ( LOCAL_examplePassed(PRU_NUM) )
    {
        printf("Example executed succesfully.\r\n");
    }
    else
    {
        printf("Example failed.\r\n");
    }
    */

    /* Disable PRU and close memory mapping*/
    prussdrv_pru_disable(PRU_NUM);
    prussdrv_exit ();
    munmap(ddrMem, 0x0FFFFFFF);
    close(mem_fd);

    return(0);
}


