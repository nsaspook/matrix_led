#line 1 "mcc_generated_files/pin_manager.c"
#line 1 "mcc_generated_files/pin_manager.c"

#line 24 "mcc_generated_files/pin_manager.c"
 


#line 47 "mcc_generated_files/pin_manager.c"
 

#line 1 "mcc_generated_files/pin_manager.h"

#line 22 "mcc_generated_files/pin_manager.h"
 


#line 45 "mcc_generated_files/pin_manager.h"
 


#line 49 "mcc_generated_files/pin_manager.h"


#line 52 "mcc_generated_files/pin_manager.h"
 



#line 57 "mcc_generated_files/pin_manager.h"
#line 58 "mcc_generated_files/pin_manager.h"

#line 60 "mcc_generated_files/pin_manager.h"
#line 61 "mcc_generated_files/pin_manager.h"

#line 63 "mcc_generated_files/pin_manager.h"
#line 64 "mcc_generated_files/pin_manager.h"

#line 66 "mcc_generated_files/pin_manager.h"
#line 67 "mcc_generated_files/pin_manager.h"


#line 77 "mcc_generated_files/pin_manager.h"
 
void PIN_MANAGER_Initialize (void);


#line 89 "mcc_generated_files/pin_manager.h"
 
void PIN_MANAGER_IOC(void);



#line 95 "mcc_generated_files/pin_manager.h"

#line 97 "mcc_generated_files/pin_manager.h"
 #line 49 "mcc_generated_files/pin_manager.c"






void PIN_MANAGER_Initialize(void)
{
    
#line 59 "mcc_generated_files/pin_manager.c"
 
    LATE = 0x00;
    LATD = 0x00;
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;

    
#line 68 "mcc_generated_files/pin_manager.c"
 
    TRISE = 0x07;
    TRISA = 0xEF;
    TRISB = 0xFF;
    TRISC = 0xFF;
    TRISD = 0xFF;

    
#line 77 "mcc_generated_files/pin_manager.c"
 
    ANCON0 = 0xFF;
    ANCON1 = 0x7F;

    
#line 83 "mcc_generated_files/pin_manager.c"
 
    WPUB = 0x00;
    INTCON2bits.nRBPU = 1;






   
    
}
  
void PIN_MANAGER_IOC(void)
{   
	
    INTCONbits.RBIF = 0;
}


#line 104 "mcc_generated_files/pin_manager.c"
 