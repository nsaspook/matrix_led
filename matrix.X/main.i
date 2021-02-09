#line 1 "main.c"
#line 1 "main.c"

#line 19 "main.c"
 


#line 42 "main.c"
 

#line 1 "./mcc_generated_files/mcc.h"

#line 22 "./mcc_generated_files/mcc.h"
 


#line 45 "./mcc_generated_files/mcc.h"
 


#line 49 "./mcc_generated_files/mcc.h"










#line 68 "./mcc_generated_files/mcc.h"
 
void SYSTEM_Initialize(void);


#line 81 "./mcc_generated_files/mcc.h"
 
void OSCILLATOR_Initialize(void);

#line 85 "./mcc_generated_files/mcc.h"

#line 87 "./mcc_generated_files/mcc.h"
 #line 44 "main.c"



#line 48 "main.c"
 
void main(void)
{
    
    SYSTEM_Initialize();

    
    
    

    
    INTERRUPT_GlobalInterruptHighEnable();

    
    INTERRUPT_GlobalInterruptLowEnable();

    
    

    
    

    
    

    
    

    while (1)
    {
        
    }
}

#line 83 "main.c"
 