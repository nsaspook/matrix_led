#line 1 "mcc_generated_files/mcc.c"
#line 1 "mcc_generated_files/mcc.c"

#line 22 "mcc_generated_files/mcc.c"
 


#line 45 "mcc_generated_files/mcc.c"
 

#line 1 "mcc_generated_files/mcc.h"

#line 22 "mcc_generated_files/mcc.h"
 


#line 45 "mcc_generated_files/mcc.h"
 


#line 49 "mcc_generated_files/mcc.h"

#line 1 "mcc_generated_files/device_config.h"

#line 22 "mcc_generated_files/device_config.h"
 


#line 45 "mcc_generated_files/device_config.h"
 


#line 49 "mcc_generated_files/device_config.h"

#line 51 "mcc_generated_files/device_config.h"

#line 53 "mcc_generated_files/device_config.h"

#line 55 "mcc_generated_files/device_config.h"
 
#line 50 "mcc_generated_files/mcc.h"

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
 #line 51 "mcc_generated_files/mcc.h"








#line 68 "mcc_generated_files/mcc.h"
 
void SYSTEM_Initialize(void);


#line 81 "mcc_generated_files/mcc.h"
 
void OSCILLATOR_Initialize(void);

#line 85 "mcc_generated_files/mcc.h"

#line 87 "mcc_generated_files/mcc.h"
 #line 47 "mcc_generated_files/mcc.c"



void SYSTEM_Initialize(void)
{

    PIN_MANAGER_Initialize();
    OSCILLATOR_Initialize();
}

void OSCILLATOR_Initialize(void)
{
    
    OSCCON = 0x60;
    
    OSCCON2 = 0x00;
    
    OSCTUNE = 0x00;
    
    REFOCON = 0x00;
}



#line 72 "mcc_generated_files/mcc.c"
 
