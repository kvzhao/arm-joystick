#
# Execute 'make' to create prucode.bin and mytest
# Other options:
# make clean
# make all
# make pru
# make project
# make prucode
# make clean
#

pru = prucode
project = pru_pwm

LIB_PATH = ./include
ASM_PATH = ./utils/
LIBRARIES = pthread prussdrv
INCLUDES = -I. -I${LIB_PATH}

EXTRA_DEFINE =
CCCFLAGS = $(EXTRA_DEFINE)
CROSS_COMPILE = arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc
CFLAGS = $(EXTRA_DEFINE)
PASM = $(ASM_PATH)pasm

all : $(pru) $(project)
pru : $(pru)
project: $(project)

$(project) : $(project:%=%.c)
	$(CC) $(CFLAGS) -c -o $@.o $@.c $(INCLUDES)
	$(CC) $@.o $(LIB_PATH:%=-L%) $(LIBRARIES:%=-l%) -o $@

clean :
	rm -rf *.o *.bin $(project) core *~

$(pru) : $(pru:%=%.p)
	$(PASM) -b $@.p

.SUFFIXES: .c.d

%.d: %.c
	$(SHELL) -ec "$(CC) -M $(CPPFLAGS) $< | sed 's/$*\\.o[ :]*/$@ &/g' > $@" -include $(SOURCES:.c=.d)

