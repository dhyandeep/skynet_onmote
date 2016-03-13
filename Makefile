CONTIKI_PROJECT = dctincontiki
all: $(CONTIKI_PROJECT)

TARGET_LIBFILES += -lm
CONTIKI_NO_NET=1
SMALL=1
CONTIKI = /home/dhyandeepak/Desktop/Project/Tools/contiki-3.0
include $(CONTIKI)/Makefile.include
