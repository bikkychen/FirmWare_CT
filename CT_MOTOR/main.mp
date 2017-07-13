NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   000C =     12. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       0098  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0098   17A2 =   6050. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       004C  __text_start
       004C  __start
       006E  _exit
       006F  _InitialIO
       007C  _uart1_init
       008E  _uart1_rx_isr
       009E  _SendUart1
       00A5  _timer3_ovf_isr
       00FA  _Pwm_startup
       03C0  _SampleADC
       042F  _CheckMotor
       058E  _Start
       0608  _Gear2Thr
       064D  _main
       09E4  _E_comp
       0A32  _EEPROM_write
       0A3A  _EEPROM_read
       0A41  mod16s
       0A45  div16s
       0A5F  mod16u
       0A61  div16u
       0A62  xdiv16u
       0A7B  mod8u
       0A7D  div8u
       0A7E  xdiv8u
       0A93  elpm32
       0AA7  empy16s
       0AB7  push_xgsetF03C
       0AC0  pop_xgsetF03C
       0AC9  neg32
       0AD2  lsr32
       0ADE  fpint
       0B10  uint2fp
       0B12  int2fp
       0B1B  ulong2fp
       0B1D  long2fp
       0B34  unpacks
       0B3C  tstR16_R19
       0B44  unpacku
       0B5D  normalize_and_pack
       0B92  pack
       0BA2  saveFPRegs
       0BB1  restoreFPRegs
       0BC0  fpmule
       0C13  fpmule1
       0C1D  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0100   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  _Tt
       0100  __bss_start
       0101  _IntFlag
       0102  _R_dat
       0103  _MotorIdelCur
       0104  _MotorIdelGear
       0105  _Motor2Thr
       0106  _Motor1Thr
       0107  _Motor2Gear
       0108  _Motor1Gear
       0109  _Motor2Status
       010B  _Motor1Status
       010D  _myFIB
       0111  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   007C =    124. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
main.o	[ main.c ]
<library>	[ div16s.s, div16u.s, div8u.s, elpm32.s, emul16s.s, gpushb.s, lneg.s, lsr32.s, fpint.s, int2fp.s, fputil.s ]
[ fpmule.s, fpmule1.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x100

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

