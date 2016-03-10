CONTIKI_PROJECT = dctincontiki
all: $(CONTIKI_PROJECT)

TARGET_LIBFILES += -lm
CONTIKI = /home/dhyandeepak/Desktop/Project/Tools/contiki-3.0
include $(CONTIKI)/Makefile.include
