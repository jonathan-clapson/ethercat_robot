BASEDIR = /home/controller/SOEM1.3.0
INCDIRS = $(BASEDIR)/soem/
LIBDIRS = $(BASEDIR)/lib/linux/
OSALDIR = $(BASEDIR)/osal/
OSHWDIR = $(BASEDIR)/oshw/linux/

APPNAME = input_test 

all: 
	gcc --std=gnu99 -o input_test -I$(INCDIRS) -I$(OSALDIR) -I$(OSHWDIR) -L$(LIBDIRS) input_test.c -lsoem -losal -loshw -lpthread

debug:
	gcc -g --std=gnu99 -o input_test -I$(INCDIRS) -I$(OSALDIR) -I$(OSHWDIR) -L$(LIBDIRS) input_test.c -lsoem -losal -loshw -lpthread

clean:
	rm input_test
