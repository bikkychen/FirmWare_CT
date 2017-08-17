NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   008C   0010 =     16. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       008C  __lit_start
       009C  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   009C   4460 =  17504. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       004E  __start
       004E  __text_start
       0070  _exit
       0071  _int0_isr
       0270  _InitialIO
       027F  _uart1_init
       0291  _uart1_rx_isr
       02B3  _SendUart1
       02F4  _timer3_init
       0310  _SendTPS
       037A  _ReceiveTPS
       0455  _SampleTPS
       0497  _uart0_init
       04A5  _uart0_rx_isr
       04C6  _timer0_ovf_isr
       04EB  _SendUart0_2
       0535  _SendUart0_3
       0585  _SampleADC
       05F4  _Delay_ms
       063A  _Start
       065A  _main
       1B86  _SendManchester
       1FBF  _EEPROM_write
       1FC7  _EEPROM_read
       1FCE  _crc16
       2021  push_arg4
       2023  push_arg2
       2026  asr16
       202D  elpm32
       2041  pop_xgsetF0FC
       204C  push_xgsetF0FC
       204E  push_xgset30FC
       2050  push_xgset00FC
       2052  push_xgset003C
       2057  push_xgsetF00C
       205E  pop_xgsetF00C
       2065  push_xgsetF03C
       206E  pop_xgsetF03C
       2077  neg32
       2080  lsl32
       208C  lsl8
       2092  lsr16
       2099  lsr32
       20A5  fpint
       20D7  uint2fp
       20D9  int2fp
       20E2  ulong2fp
       20E4  long2fp
       20FB  fpdiv1
       2104  fpdiv2
       210E  fpdiv1x
       211A  fpdiv2x
       2195  unpacks
       219D  tstR16_R19
       21A5  unpacku
       21BE  normalize_and_pack
       21F3  pack
       2203  saveFPRegs
       2212  restoreFPRegs
       2221  fpmule
       2274  fpmule1
       227E  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0100   024C =    588. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __bss_start
       0100  _crc16array
       0120  _crc16lo
       0121  _crc16hi
       0122  _lfib
       0126  _bf
       0128  _CompCounter
       012A  _coeff_count
       012C  _coeff_len
       012E  _EEPAdd
       012F  _Uart0TxData
       0179  _coeffdata
       02B9  _T2cn
       02BB  _k
       02BD  _Tt
       02BF  _IntFlag
       02C0  _Int_count
       02C1  _DataFlag
       02C3  _T_dat_buf
       02D9  _TimeOutFlag
       02DA  _T_com
       02DB  _TPS_TData
       02DD  _TPS_PData
       02DF  _R_dat
       02E1  _T_dat
       02E3  _Rx
       02F5  _Uart1RxCounter
       02F6  _Uart1RxData
       02FA  _Uart0RxCounter
       02FB  _Uart0RxData
       0345  _Timer0Counter
       0347  _T_Flag
       0348  _myFIB
       034C  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   007C =    124. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtatmega.o	[ crtatmega.s ]
main.o	[ main.c ]
<library>	[ asave.s, asr16.s, elpm32.s, gpop5.s, gpush.s, gpushz.s, gpushb.s, lneg.s, lsl32.s, lsl8.s, lsr16.s, lsr32.s ]
[ fpint.s, int2fp.s, fpdiv_new.s, fputil.s, fpmule.s, fpmule1.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x100

User Base Address Definitions

func_lit = 0x8c
data = 0x100
eeprom:0.4096

