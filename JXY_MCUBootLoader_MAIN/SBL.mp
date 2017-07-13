NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   1E08C   0004 =      4. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       1E08C  __idata_start
       1E090  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   1E090   1846 =   6214. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       F048  __text_start
       F048  __start
       F06E  _exit
       F06F  _boot_page_ew
       F07A  _boot_page_fill
       F086  _wait_page_rw_ok
       F094  _write_one_page
       F0CF  _uart1_init
       F0E1  _uart1_rx_isr
       F0FF  _uart0_init
       F10D  _usart0_isr
       F12A  _quit
       F132  _Start
       F163  _crc16
       F1B6  _timer3_ovf_isr
       F1CE  _SendUart0
       F1FE  _SendUart1
       F239  _SetTimer3
       F255  _main
       F642  _int0_isr
       F841  _SendManchester
       FBCC  _Delay30Ms
       FBE0  push_arg4
       FBE2  push_arg2
       FBE5  empy16s
       FBF5  empy32u
       FBF5  empy32s
       FC29  pop_xgsetF0FC
       FC34  push_xgsetF0FC
       FC36  push_xgset30FC
       FC38  push_xgset00FC
       FC3A  push_xgset003C
       FC3F  long_prolog
       FC4D  long_epilog
       FC58  tstzero1
       FC5E  tstzero2
       FC64  lsr16
       FC6B  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0100   0004 =      4. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __data_start
       0100  _address
       0104  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0104   0245 =    581. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0104  __bss_start
       0104  _T2cn
       0106  _flash_buf
       0208  _TimeOutFlag
       0209  _Timer1Counter
       020B  _Timer0Counter
       020D  _Uart1RxData
       028D  _Uart0RxData
       030D  _Uart1RxCounter
       030E  _Uart0RxCounter
       030F  _DSP_PageCn
       0310  _Timer3Cn
       0312  _DownloadSpeed
       0313  _crc16lo
       0314  _crc16hi
       0315  _DataReCn
       0317  _UpdateBegin
       0318  _Tt
       031A  _IntFlag
       031B  _Int_count
       031C  _T_dat_buf
       0332  _T_com
       0333  _R_dat
       0335  _T_dat
       0337  _Rx
       0349  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   1E000   007C =    124. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtboot128.o	[ crtboot128.s ]
SBL.o	[ SBL.c ]
<library>	[ asave.s, emul16s.s, emul32.s, gpop5.s, gpush.s, longutil.s, lsr16.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x100

User Base Address Definitions

vector = 0x1e000
func_lit = 0x1e08c
data = 0x100
eeprom:0.4096

