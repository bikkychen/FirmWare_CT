NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   0018 =     24. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       00A4  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   00A4   4AA4 =  19108. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0052  __start
       0052  __text_start
       0074  _exit
       0075  _InitialIO
       008D  _uart0_init
       009B  _uart0_rx_isr
       009C  _timer0_ovf_isr
       00C1  _timer3_ovf_isr
       010C  _SendUart0_2
       015C  _SendUart0_3
       01B2  _Pwm_startup
       02DD  _SampleADC
       034A  _CheckMotor
       04F6  _Start
       05DA  _Gear2Thr
       061F  _main
       1BB9  _E_comp
       1BFF  _usart0_isr
       1C20  _int3_isr
       1E37  _SendManchester
       223A  _EEPROM_write
       2242  _EEPROM_read
       2249  _crc16
       229C  push_arg4
       229E  push_arg2
       22A1  asr16
       22A8  mod16s
       22AC  div16s
       22C6  mod16u
       22C8  div16u
       22C9  xdiv16u
       22E2  elpm32
       22F6  empy16s
       2306  pop_xgsetF0FC
       2311  push_xgsetF0FC
       2313  push_xgset30FC
       2315  push_xgset00FC
       2317  push_xgset003C
       231C  push_xgsetF03C
       2325  pop_xgsetF03C
       232E  neg32
       2337  lsl32
       2343  lsr16
       234A  lsr32
       2356  fpsub
       2358  fpadd
       2359  fpadd_alt
       23C0  fpadd2
       23CB  fpint
       23FD  uint2fp
       23FF  int2fp
       2408  ulong2fp
       240A  long2fp
       2421  fpdiv1
       242A  fpdiv2
       2434  fpdiv1x
       2440  fpdiv2x
       24BB  unpacks
       24C3  tstR16_R19
       24CB  unpacku
       24E4  normalize_and_pack
       2519  pack
       2529  saveFPRegs
       2538  restoreFPRegs
       2547  fpmule
       259A  fpmule1
       25A4  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0100   023F =    575. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __bss_start
       0100  _crc16array
       0112  _crc16lo
       0113  _crc16hi
       0114  _lfib
       0118  _bf
       011A  _CompCounter
       011C  _coeff_count
       011E  _coeff_len
       0120  _EEPAdd
       0121  _Uart0TxData
       016B  _coeffdata
       02AB  _T2cn
       02AD  _k
       02AF  _Tt
       02B1  _IntFlag
       02B2  _Int_count
       02B3  _DataFlag
       02B5  _T_dat_buf
       02CB  _TimeOutFlag
       02CC  _T_com
       02CD  _R_dat
       02CF  _T_dat
       02D1  _Rx
       02E3  _Uart0RxCounter
       02E4  _Uart0RxData
       032E  _Timer0Counter
       0330  _T_Flag
       0331  _MotorIdelCur
       0332  _MotorIdelGear
       0333  _Motor2Thr
       0334  _Motor1Thr
       0335  _Motor2Gear
       0336  _Motor1Gear
       0337  _Motor2Status
       0339  _Motor1Status
       033B  _myFIB
       033F  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   0078 =    120. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
main.o	[ main.c ]
<library>	[ asave.s, asr16.s, div16s.s, div16u.s, elpm32.s, emul16s.s, gpop5.s, gpush.s, gpushb.s, lneg.s, lsl32.s ]
[ lsr16.s, lsr32.s, fpadd.s, fpadd2.s, fpint.s, int2fp.s, fpdiv_new.s, fputil.s, fpmule.s, fpmule1.s ]


User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x80

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

