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
                            text   009C   3ED6 =  16086. bytes (rel,con,rom)

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
       18C1  _SendManchester
       1CFA  _EEPROM_write
       1D02  _EEPROM_read
       1D09  _crc16
       1D5C  push_arg4
       1D5E  push_arg2
       1D61  asr16
       1D68  elpm32
       1D7C  pop_xgsetF0FC
       1D87  push_xgsetF0FC
       1D89  push_xgset30FC
       1D8B  push_xgset00FC
       1D8D  push_xgset003C
       1D92  push_xgsetF00C
       1D99  pop_xgsetF00C
       1DA0  push_xgsetF03C
       1DA9  pop_xgsetF03C
       1DB2  neg32
       1DBB  lsl32
       1DC7  lsl8
       1DCD  lsr16
       1DD4  lsr32
       1DE0  fpint
       1E12  uint2fp
       1E14  int2fp
       1E1D  ulong2fp
       1E1F  long2fp
       1E36  fpdiv1
       1E3F  fpdiv2
       1E49  fpdiv1x
       1E55  fpdiv2x
       1ED0  unpacks
       1ED8  tstR16_R19
       1EE0  unpacku
       1EF9  normalize_and_pack
       1F2E  pack
       1F3E  saveFPRegs
       1F4D  restoreFPRegs
       1F5C  fpmule
       1FAF  fpmule1
       1FB9  __text_end

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

