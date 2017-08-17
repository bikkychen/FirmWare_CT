;============================================================================
;
; File Name     : rfft_acqc.asm
; 
; Originator    : Advanced Embeeded Control 
;                 Texas Instruments 
; 
; Description   : This file contains to acquire 2N samples and store as N complex valued 
;                 sequence in bit-reversed order                  
;               
; Date          : 26/2/2002 (dd/mm/yyyy)
;===========================================================================    
; Routine Type  : C Callable        
;
; Description   :
; void FFTR_acq(FFTRACQ_handle) 
; This function acquires 2N samples and stores it in a buffer as N-point complex sequence
; in bit reversed order. This is used for real FFT computation.
; The buffer should be aligned to 2N words
; 
;=====================================================================
; typedef   struct {
;       int acqflag;
;       int count;
;       int size;
;       int input; 
;       long *tempptr;
;       long *buffptr;
;       void (*update)(void *); 
;       }FFTRACQ;  
;
;======================================================================

                .def    _RFFT32_acq       ; 
                
_RFFT32_acq:           
            MOV     ACC,*XAR4++     ; ACC=acqflag
            SBF     noacq,EQ
                              

            TBIT    *XAR4,#0      ; if MOD(count,2)==1, set TC
            DEC     *XAR4           ; count=count-1
 
            MOVZ    AR0,*+XAR4[7]   ; AR0=size  
            MOVL    XAR5,*+XAR4[3]  ; XAR5=tempptr
            MOVL    ACC,*+XAR4[1]   ; ACC=input

            BF    oddseq,TC
            MOVL    *XAR5,ACC       ; *tempptr=input

            MOV     ACC,*XAR4++     ; ACC=count
            SBF     acqover,EQ      ; if count=0, acquisition is complete  

noacq:      LRETR
                                 
oddseq:     
            MOVL    *+XAR5[2],ACC   ; *tempptr=input
            NOP     *BR0++
            MOVL    *+XAR4[3],XAR5

            MOV     ACC,*XAR4++     ; ACC=count
            SBF     acqover,EQ      ; if count=0, acquisition is complete  
            
            LRETR
    
acqover:    MOV     ACC,*+XAR4[6]
            MOV     *--XAR4,ACC     ; count=size
            MOV     *--XAR4,#0      ; acqflag=0
            
            MOVL    XAR5,*+XAR4[6]  ; XAR5=buffptr
            MOVL    *+XAR4[4],XAR5  ; tempptr=buffptr
            LRETR
            
            
            
            
