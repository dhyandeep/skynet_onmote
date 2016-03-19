all: dctincontiki

#Define CONTIKI_NO_NET=1 for a passthrough ipv6/6lowpan interface using fakeuip.c
#Define CONTIKI_WITH_IPV6 = 1 to include the uip6 stack (for rpl, internal webserver)
#Do make clean when switching to remove the duplicate library modules

CONTIKI_WITH_IPV6=0

CONTIKI = /home/dhyandeepak/Desktop/contiki-2.7/


include $(CONTIKI)/Makefile.include
