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
                            text   1E090   0534 =   1332. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       F048  __start
       F048  __text_start
       F06E  _exit
       F06F  _boot_page_ew
       F07A  _boot_page_fill
       F086  _wait_page_rw_ok
       F094  _write_one_page
       F0CF  _uart1_init
       F0E1  _uart1_rx_isr
       F0F6  _quit
       F0FE  _timer3_ovf_isr
       F116  _SendUart1
       F11D  _SetTimer3_500ms
       F139  _SetTimer3_3s
       F155  _main
       F260  _Delay30Ms
       F274  push_arg4
       F276  push_arg2
       F279  empy16s
       F289  empy32s
       F289  empy32u
       F2BD  long_prolog
       F2CB  long_epilog
       F2D6  tstzero1
       F2DC  tstzero2
       F2E2  __text_end

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
                             bss   0104   0191 =    401. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0104  __bss_start
       0104  _T2cn
       0106  _flash_buf
       0208  _TimeOutFlag
       0209  _Uart1RxData
       0289  _Uart1RxCounter
       028A  _Timer3Cn
       028C  _DownloadSpeed
       028D  _crc16lo
       028E  _crc16hi
       028F  _DataReCn
       0291  _UpdateBegin
       0292  _IntFlag
       0293  _R_dat
       0295  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   1E000   007C =    124. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

E:\iccv7avr\lib\crtboot128.o	[ crtboot128.s ]
SBL.o	[ SBL.c ]
<library>	[ asave.s, emul16s.s, emul32.s, longutil.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x100

User Base Address Definitions

vector = 0x1e000
func_lit = 0x1e08c
data = 0x100
eeprom:0.4096

