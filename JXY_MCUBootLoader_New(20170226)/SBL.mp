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
                            text   1E090   195E =   6494. bytes (rel,con,rom)

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
       F679  _int3_isr
       F890  _E_comp
       F8D6  _SendManchester
       FC31  _Delay30Ms
       FC45  _usart0_isr
       FC62  push_arg4
       FC64  push_arg2
       FC67  empy16s
       FC77  empy32u
       FC77  empy32s
       FCAB  pop_xgsetF0FC
       FCB6  push_xgsetF0FC
       FCB8  push_xgset30FC
       FCBA  push_xgset00FC
       FCBC  push_xgset003C
       FCC1  push_xgset300C
       FCC6  pop_xgset300C
       FCCB  long_prolog
       FCD9  long_epilog
       FCE4  tstzero1
       FCEA  tstzero2
       FCF0  lsr16
       FCF7  __text_end

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

