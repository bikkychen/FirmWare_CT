NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   0014 =     20. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       00A0  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   00A0   5000 =  20480. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0050  __start
       0050  __text_start
       0072  _exit
       0073  _int0_isr
       0272  _InitialIO
       028A  _uart1_init
       029C  _uart1_rx_isr
       02C4  _SendTPS
       0312  _uart0_init
       0320  _uart0_rx_isr
       0341  _timer0_ovf_isr
       0366  _timer3_ovf_isr
       03C1  _SendUart0_2
       0411  _SendUart0_3
       0467  _Pwm_startup
       074F  _SampleADC
       07BE  _CheckMotor
       09DF  _Start
       0AC7  _Gear2Thr
       0B0C  _main
       20CA  _E_comp
       2144  _SendManchester
       2547  _EEPROM_write
       254F  _EEPROM_read
       2556  _crc16
       25A9  push_arg4
       25AB  push_arg2
       25AE  asr16
       25B5  mod16s
       25B9  div16s
       25D3  mod16u
       25D5  div16u
       25D6  xdiv16u
       25EF  elpm32
       2603  empy16s
       2613  pop_xgsetF0FC
       261E  push_xgsetF0FC
       2620  push_xgset30FC
       2622  push_xgset00FC
       2624  push_xgset003C
       2629  push_xgsetF00C
       2630  pop_xgsetF00C
       2637  push_xgsetF03C
       2640  pop_xgsetF03C
       2649  neg32
       2652  lsl32
       265E  lsr16
       2665  lsr32
       2671  lsr8
       2677  fpint
       26A9  uint2fp
       26AB  int2fp
       26B4  ulong2fp
       26B6  long2fp
       26CD  fpdiv1
       26D6  fpdiv2
       26E0  fpdiv1x
       26EC  fpdiv2x
       2767  unpacks
       276F  tstR16_R19
       2777  unpacku
       2790  normalize_and_pack
       27C5  pack
       27D5  saveFPRegs
       27E4  restoreFPRegs
       27F3  fpmule
       2846  fpmule1
       2850  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0100   0244 =    580. bytes (rel,con,ram)

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
       0331  _TPSBuf
       0335  _TPSCn
       0336  _MotorIdelCur
       0337  _MotorIdelGear
       0338  _Motor2Thr
       0339  _Motor1Thr
       033A  _Motor2Gear
       033B  _Motor1Gear
       033C  _Motor2Status
       033E  _Motor1Status
       0340  _myFIB
       0344  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   007C =    124. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
main.o	[ main.c ]
<library>	[ asave.s, asr16.s, div16s.s, div16u.s, elpm32.s, emul16s.s, gpop5.s, gpush.s, gpushz.s, gpushb.s, lneg.s ]
[ lsl32.s, lsr16.s, lsr32.s, lsr8.s, fpint.s, int2fp.s, fpdiv_new.s, fputil.s, fpmule.s, fpmule1.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x100

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

