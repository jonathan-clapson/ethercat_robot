BASEDIR = /home/controller/SOEM1.3.0
INCDIRS = $(BASEDIR)/soem/
LIBDIRS = $(BASEDIR)/lib/linux/
OSALDIR = $(BASEDIR)/osal/
OSHWDIR = $(BASEDIR)/oshw/linux/
CFLAGS = 

APPNAME = soem_main

all: 
	gcc $(CFLAGS) --std=gnu99 -o $(APPNAME) -I$(INCDIRS) -I$(OSALDIR) -I$(OSHWDIR) -L$(LIBDIRS) soem_main.c state_machine.c wago_steppers.c -lsoem -losal -loshw -lpthread

debug:
	gcc $(CFLAGS) -g --std=gnu99 -o $(APPNAME) -I$(INCDIRS) -I$(OSALDIR) -I$(OSHWDIR) -L$(LIBDIRS) soem_main.c -lsoem -losal -loshw -lpthread

clean:
	rm input_test
