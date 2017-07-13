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
                            text   1E090   176C =   5996. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       F048  __start
       F048  __text_start
       F06E  _exit
       F06F  _boot_page_ew
       F07A  _boot_page_fill
       F086  _wait_page_rw_ok
       F094  _write_one_page
       F0CE  _InitialIO
       F0E6  _init_USART0
       F0F4  _quit
       F0FC  _Start
       F1D5  _crc16
       F228  _timer3_ovf_isr
       F240  _SendUart0
       F270  _SetTimer3
       F28C  _main
       F580  _int3_isr
       F797  _E_comp
       F7DD  _SendManchester
       FB38  _Delay30Ms
       FB4C  _usart0_isr
       FB69  push_arg4
       FB6B  push_arg2
       FB6E  empy16s
       FB7E  empy32u
       FB7E  empy32s
       FBB2  pop_xgsetF0FC
       FBBD  push_xgsetF0FC
       FBBF  push_xgset30FC
       FBC1  push_xgset00FC
       FBC3  push_xgset003C
       FBC8  push_xgset300C
       FBCD  pop_xgset300C
       FBD2  long_prolog
       FBE0  long_epilog
       FBEB  tstzero1
       FBF1  tstzero2
       FBF7  lsr16
       FBFE  __text_end

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
                             bss   0104   0148 =    328. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0104  __bss_start
       0104  _T2cn
       0106  _flash_buf
       0208  _TimeOutFlag
       0209  _Timer0Counter
       020B  _Uart0RxData
       020D  _Uart0RxCounter
       020E  _DSP_PageCn
       020F  _Timer3Cn
       0211  _DownloadSpeed
       0212  _crc16lo
       0213  _crc16hi
       0214  _DataReCn
       0216  _UpdateBegin
       0217  _Tt
       0219  _IntFlag
       021A  _Int_count
       021B  _T_dat_buf
       0231  _T_com
       0232  _R_dat
       0234  _T_dat
       0236  _Rx
       0248  _Motor2Status
       024A  _Motor1Status
       024C  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   1E000   0078 =    120. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtboot128.o	[ crtboot128.s ]
SBL.o	[ SBL.c ]
<library>	[ asave.s, emul16s.s, emul32.s, gpop5.s, gpush.s, gpushx.s, longutil.s, lsr16.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x80

User Base Address Definitions

vector = 0x1e000
func_lit = 0x1e08c
data = 0x100
eeprom:0.4096

