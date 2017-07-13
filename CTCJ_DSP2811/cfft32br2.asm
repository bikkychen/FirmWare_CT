;========================================================================
;
; File Name     : cfft_br2c.asm
; 
; Originator    : Advanced Embeeded Control 
;                 Texas Instruments 
; 
; Description   : This file contains source code, to bit reverse the N-point real sequence
;                 stored in alternate memory locations.
;
; Date          : 26/2/2002
;======================================================================
; 
; 
; Routine Name  : Generic Function      
; Routine Type  : C Callable
; 
; Usage :
; void FFTC_brev2(int *src, int *dst, ushort n)
; This function bit reverses the N-point real sequence stored in alternate memory locations 
; to carry out N-point complex FFT computation. If src=dst, then in-place bit reversal is performed.
; The destination array should be aligned to 2N words. 
;=====================================================================
; INPUT ARRAY
;         |Xr(0)|
;         |Xi(0)|
;         |Xr(1)|
;         |Xi(1)|
;            :
;            :
;=====================================================================

                .def _CFFT32_brev2
 
_CFFT32_brev2:  
                                    ; XAR4->src, XAR5->dst, AL=size
            MOVL    XAR0,XAR4       ; XAR0->src
            NOP     *,ARP5
            
            CMPR    0               ; TC=1, if src==dst
            BF      off_place,NTC   ; If not equal, jump to off place bit rev

; IN-PLACE BIT-REVERSAL
in_place:
            MOVZ    AR7,AL          ; AR0=size
            SUBB    XAR7,#1         ; XAR7=size-1
            LSL     ACC,#1          ; ACC=2*size
            MOVZ    AR6,AL          ; AR0=2*size
            
next_data1: 
            MOVL    XT,*XAR4        ; XT=*src
            MOVL    P,*XAR5         ; P=*dst
            MOVL    XAR0,XAR4       ; XAR0=src
            CMPR    2               ; check src<dst
            BF      noswap,TC       ; if src<dst, do not swap *src & *dst
            
            MOVL    *XAR4,P         ; *src=*dst
            MOVL    *XAR5,XT        ; *dst=*src
           
noswap:     
            ADDB    XAR4,#4         ; XAR4=src+2
            MOVZ    AR0,AR6         ; AR0=2*size
            NOP     *BR0++          ; dst=br(dst+2*size)
            BANZ    next_data1,AR7-- ; ARP=AR3, AR3->src+2, AR7=AR7-1
            LRETR
            
; OFF-PLACE BIT-REVERSAL            
off_place:  
            MOVZ    AR7,AL          ; AR0=size
            SUBB    XAR7,#1         ; XAR7=size-1
            LSL     ACC,#1          ; ACC=2*size
            MOVZ    AR0,AL          ; AR0=2*size
            
next_data2: MOVL    ACC,*XAR4++     ; XAR4=src+1, ACCL=*src
            ADDB    XAR4,#2         ; XAR4=src+2
            NOP     *,ARP5          ; ARP=XAR5
            MOVL    *BR0++,ACC      ; XAR5=br(dst+size)
            BANZ    next_data2,AR7--                 
            LRETR

