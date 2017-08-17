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
                            text   0098   17D2 =   6098. bytes (rel,con,rom)

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
       09FC  _E_comp
       0A4A  _EEPROM_write
       0A52  _EEPROM_read
       0A59  mod16s
       0A5D  div16s
       0A77  mod16u
       0A79  div16u
       0A7A  xdiv16u
       0A93  mod8u
       0A95  div8u
       0A96  xdiv8u
       0AAB  elpm32
       0ABF  empy16s
       0ACF  push_xgsetF03C
       0AD8  pop_xgsetF03C
       0AE1  neg32
       0AEA  lsr32
       0AF6  fpint
       0B28  uint2fp
       0B2A  int2fp
       0B33  ulong2fp
       0B35  long2fp
       0B4C  unpacks
       0B54  tstR16_R19
       0B5C  unpacku
       0B75  normalize_and_pack
       0BAA  pack
       0BBA  saveFPRegs
       0BC9  restoreFPRegs
       0BD8  fpmule
       0C2B  fpmule1
       0C35  __text_end

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

