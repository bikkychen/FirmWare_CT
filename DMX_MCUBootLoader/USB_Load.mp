NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   1E08C   0004 =      4. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       1E08C  __lit_start
       1E090  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   1E090   0065 =    101. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       1E090  __idata_start
       1E0F5  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   1E0F6   140A =   5130. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       F07B  __text_start
       F07B  __start
       F0A1  _exit
       F0A2  _uart0_init
       F0AF  _port_init
       F0C4  _DelayMs
       F103  _crc16
       F156  _timer3_ovf_isr
       F16E  _SetTimer3
       F18A  _init_devices
       F19B  _boot_page_ew
       F1A6  _boot_page_fill
       F1B2  _wait_page_rw_ok
       F1C0  _write_one_page
       F1FB  _delay
       F203  _quit
       F20C  _main
       F4D0  _int6_isr
       F773  _Init374Device
       F793  _USBStart
       F7C3  _Read374Byte
       F7D6  _Write374Byte
       F7E7  _Read374Block
       F809  _Write374Block
       F829  _Write374Block2
       F849  _EP2SendConst
       F86B  _EP2Send
       F88D  push_arg4
       F88F  push_arg2
       F892  elpm32
       F8A6  empy16s
       F8B6  empy32u
       F8B6  empy32s
       F8EA  pop_xgset003C
       F8EF  pop_xgsetF0FC
       F8FA  push_xgsetF0FC
       F8FC  push_xgset30FC
       F8FE  push_xgset00FC
       F900  push_xgset003C
       F905  push_xgsetF00C
       F90C  pop_xgsetF00C
       F913  neg32
       F91C  long_prolog
       F92A  long_epilog
       F935  tstzero1
       F93B  tstzero2
       F941  fpint
       F973  uint2fp
       F975  int2fp
       F97E  ulong2fp
       F980  long2fp
       F997  unpacks
       F99F  tstR16_R19
       F9A7  unpacku
       F9C0  normalize_and_pack
       F9F5  pack
       FA05  saveFPRegs
       FA14  restoreFPRegs
       FA23  fpmule
       FA76  fpmule1
       FA80  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0100   0065 =    101. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __data_start
       0100  _address
       0104  _UpdateFlag
       0105  _EP2SendBusy
       0106  _MyDevDescr
       0118  _MyCfgDescr
       013F  _MyLangDescr
       0143  _MyManuInfo
       0151  _MyProdInfo
       0163  _UsbConfig
       0164  _UsbReset
       0165  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0165   0190 =    400. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0165  __bss_start
       0165  _Timer3Cn
       0167  _DownloadSpeed
       0168  _crc16lo
       0169  _crc16hi
       016A  _DataReCn
       016C  _UpdateBegin
       016D  _RxData
       01AF  _tx_data
       01EF  _flash_buf
       02EF  _flash_bufferPoint
       02F1  DEVICE.C:SetupReq
       02F2  DEVICE.C:SetupLen
       02F3  DEVICE.C:pDescr
       02F5  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   1E000   0078 =    120. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

D:\iccv7avr\lib\crtboot128.o	[ crtboot128.s ]
main.o	[ main.c ]
DEVICE.o	[ DEVICE.C ]
SPI_HW.o	[ SPI_HW.C ]
<library>	[ asave.s, elpm32.s, emul16s.s, emul32.s, gpop2.s, gpop5.s, gpush.s, gpushz.s, lneg.s, longutil.s, fpint.s ]
[ int2fp.s, fputil.s, fpmule.s, fpmule1.s ]

User Global Definitions

ram_end = 0x10ff
hwstk_size = 0x80

User Base Address Definitions

vector = 0x1e000
func_lit = 0x1e08c
data = 0x100
eeprom:0.4096

