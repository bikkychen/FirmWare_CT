CC = iccavr
LIB = ilibw
CFLAGS =  -e -D__ICC_VERSION=722 -DATMega128  -l -g -Wf-intenum -MLongJump -MHasMul -MEnhanced -Wf-use_elpm -Wf-const_is_flash -DCONST="" 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -g -e:0x20000 -ucrtatmega.o -bfunc_lit:0x8c.0x1fc00 -dram_end:0x10ff -bdata:0x100.0x10ff -dhwstk_size:256 -beeprom:0.4096 -fihx_coff -S2
FILES = main.o 

MAIN:	$(FILES)
	$(CC) -o MAIN $(LFLAGS) @MAIN.lk   -lfpatm128 -lcatm128
main.o: E:\iccv7avr\include\iom128v.h E:\iccv7avr\include\macros.h E:\iccv7avr\include\AVRdef.h E:\iccv7avr\include\stdio.h E:\iccv7avr\include\stdarg.h E:\iccv7avr\include\_const.h
main.o:	main.c
	$(CC) -c $(CFLAGS) main.c
