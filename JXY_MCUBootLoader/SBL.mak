CC = iccavr
LIB = ilibw
CFLAGS =  -ID:\PROGRA~1\icc\include\ -e -D__ICC_VERSION=722 -DATMega128  -l -g -Wa-ID:\SZTA-5\soft\Ö÷¿Ø°å -Wf-intenum -MLongJump -MHasMul -MEnhanced -Wf-use_elpm -Wf-const_is_flash -DCONST="" 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -LD:\PROGRA~1\icc\lib\ -g -e:0x20000 -ucrtboot128.o -bvector:0x1e000.0x20000 -bfunc_lit:0x1e08c.0x20000 -dram_end:0x10ff -bdata:0x100.0x10ff -dhwstk_size:128 -beeprom:0.4096 -fihx_coff -S2
FILES = SBL.o 

SBL:	$(FILES)
	$(CC) -o SBL $(LFLAGS) @SBL.lk   -lfpatm128 -lcatm128
SBL.o: E:\iccv7avr\include\iom128v.h E:\iccv7avr\include\macros.h E:\iccv7avr\include\AVRdef.h E:\iccv7avr\include\stdio.h E:\iccv7avr\include\stdarg.h E:\iccv7avr\include\_const.h
SBL.o:	SBL.c
	$(CC) -c $(CFLAGS) SBL.c
