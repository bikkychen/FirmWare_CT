NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   0004 =      4. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       0090  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0090   40E2 =  16610. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0048  __start
       0048  __text_start
       006A  _exit
       006B  _InitialIO
       0083  _uart0_init
       0091  _uart0_rx_isr
       0092  _timer0_ovf_isr
       00B7  _SendUart0_2
       0107  _SendUart0_3
       015D  _Pwm_startup
       0288  _SampleADC
       02F5  _CheckMotor
       0477  _Start
       0559  _main
       1827  _E_comp
       186D  _usart0_isr
       188E  _int3_isr
       1AA5  _SendManchester
       1EA8  _EEPROM_write
       1EB0  _EEPROM_read
       1EB7  _crc16
       1F0A  push_arg4
       1F0C  push_arg2
       1F0F  asr16
       1F16  elpm32
       1F2A  pop_xgsetF0FC
       1F35  push_xgsetF0FC
       1F37  push_xgset30FC
       1F39  push_xgset00FC
       1F3B  push_xgset003C
       1F40  push_xgsetF03C
       1F49  pop_xgsetF03C
       1F52  neg32
       1F5B  lsl32
       1F67  lsr16
       1F6E  lsr32
       1F7A  fpint
       1FAC  uint2fp
       1FAE  int2fp
       1FB7  ulong2fp
       1FB9  long2fp
       1FD0  unpacks
       1FD8  tstR16_R19
       1FE0  unpacku
       1FF9  normalize_and_pack
       202E  pack
       203E  saveFPRegs
       204D  restoreFPRegs
       205C  fpmule
       20AF  fpmule1
       20B9  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0100   0239 =    569. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __bss_start
       0100  _crc16array
       0110  _crc16lo
       0111  _crc16hi
       0112  _lfib
       0116  _bf
       0118  _CompCounter
       011A  _coeff_count
       011C  _coeff_len
       011E  _EEPAdd
       011F  _Uart0TxData
       0169  _coeffdata
       02A9  _T2cn
       02AB  _k
       02AD  _Tt
       02AF  _IntFlag
       02B0  _Int_count
       02B1  _DataFlag
       02B3  _T_dat_buf
       02C9  _TimeOutFlag
       02CA  _T_com
       02CB  _R_dat
       02CD  _T_dat
       02CF  _Rx
       02E1  _Uart0RxCounter
       02E2  _Uart0RxData
       032C  _Timer0Counter
       032E  _T_Flag
       032F  _Motor2Thr
       0330  _Motor1Thr
       0331  _Motor2Status
       0333  _Motor1Status
       0335  _myFIB
       0339  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   0060 =     96. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
main.o	[ main.c ]
<library>	[ asave.s, asr16.s, elpm32.s, gpop5.s, gpush.s, gpushb.s, lneg.s, lsl32.s, lsr16.s, lsr32.s, fpint.s, int2fp.s ]
[ fputil.s, fpmule.s, fpmule1.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x80

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

