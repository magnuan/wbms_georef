#
# WBMS
#
CC	= gcc
TOP	= $(PWD)
TARGETDIR ?= $(TOP)/target

CFLAGS  += -std=gnu99

PROGS = wbms_georef 

CFLAGS += -Wunused-variable
CFLAGS	+= -Wall 

#DEBUGGING
#CFLAGS += -g -O0

#Performance
CFLAGS	+= -O2

#CFLAGS += -DNO_INLINE

LDFLAGS	+= -lm  -lproj 

all: $(PROGS)

wbms_georef:  wbms_georef.o crc32.o cmath.o reson7k.o wbms_data.o velodyne.o time_functions.o linalg.o raytracing.o posmv.o xtf_nav.o wbm_tool_nav.o sim_nav.o sbet_nav.o proj_wrapper.o georef_tools.o sbf_output.o csv_output.o bin_output.o intensity_scaling.o non_posix.o

	$(CC) -o $@ $^ $(LDFLAGS)


clean:
	-rm -f $(PROGS) *.o *~
