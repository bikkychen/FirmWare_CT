CC = iccavr
LIB = ilibw
CFLAGS =  -ID:\TS\TSNY\FirmWare_SYYC\USB_Load -IC:\iccv7avr\include -e -D__ICC_VERSION=722 -DATMega128  -l -A -g -Wf-intenum -MLongJump -MHasMul -MEnhanced -Wf-use_elpm 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -LC:\iccv7avr\lib -g -e:0x20000 -ucrtboot128.o -bvector:0x1e000.0x20000 -bfunc_lit:0x1e08c.0x20000 -dram_end:0x10ff -bdata:0x100.0x10ff -dhwstk_size:128 -beeprom:0.4096 -fihx_coff -S2
FILES = main.o DEVICE.o SPI_HW.o 

USB_LOAD:	$(FILES)
	$(CC) -o USB_LOAD $(LFLAGS) @USB_LOAD.lk   -lfpatm128 -lcatm128
main.o: D:\iccv7avr\include\macros.h D:\iccv7avr\include\AVRdef.h D:\iccv7avr\include\stdio.h D:\iccv7avr\include\stdarg.h D:\iccv7avr\include\_const.h .\HAL.H D:\iccv7avr\include\iom128v.h .\CH374INC.H
main.o:	main.c
	$(CC) -c $(CFLAGS) main.c
DEVICE.o: .\HAL.H D:\iccv7avr\include\iom128v.h .\CH374INC.H
DEVICE.o:	DEVICE.C
	$(CC) -c $(CFLAGS) DEVICE.C
SPI_HW.o: D:\iccv7avr\include\string.h D:\iccv7avr\include\_const.h D:\iccv7avr\include\stdio.h D:\iccv7avr\include\stdarg.h .\HAL.H D:\iccv7avr\include\iom128v.h .\CH374INC.H
SPI_HW.o:	SPI_HW.C
	$(CC) -c $(CFLAGS) SPI_HW.C
