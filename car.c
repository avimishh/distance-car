 

#include <intrins.h>
#include "aduc842.h"
#include "globals.h"
#include "car.h"

//#define  T0_VALUE  63693 + 23     /*   63693=2mSEC    64615=1mSEC      60928=5mSEC AT 11.0592MC*/
//#define  T0_VALUE_ADUC842  61341 + 24      // -10486 = +55050 5mSEC, -4194=61341 2mSec   AT 2.097152MC  ADUC842
//#define  T0_VALUE_ADUC842  23593 + 24      // -41943 = +23593 20mSEC, -4194=61341 2mSec   AT 2.097152MC  ADUC842
//#define  T0_VALUE_ADUC842  48759 + 24      // -16777=48759 2mSec   500hz,       AT 8.388608MC  ADUC842

#define  T0_VALUE_ADUC842  23593 + 24      // -41943=23593 5mSec 200hz,   AT 8.388608MC   



#define HEADER 0xff


#define LED_ACTIVE_VAL     0
#define LED_NO_ACTIVE_VAL  1

                       /* timers */
#define rangeTimer          0

#define totalTimers 1

/* globals */
bit newData, t0_interrupt, tiFlag, saveLimitsRun, enableSendSamples, pseudoSensor;
    
byte charInKlotBuf, frame, timer0Lo, timer0Hi, timeVal, maxDelta ;
word  loLimit, hiLimit, mDelay, midLimit, pseudoSensorVal;

byte idata klotBuf[10];
byte paramBuf[4];

word timer[totalTimers];

#define MAX_FRAME     15

byte code paramTab[] = { 0, 1, 1, 1, 1, 2, 2, 1,  1, 4,  1 } ;
                    /*   0  1  2  3  4  5  6  7   8  9   10    */
                    

#define LED_ACTIVE_VAL     0
#define LED_NO_ACTIVE_VAL  1


void time(word w) /* 100 uSec = 0.1mSec */
  {
    byte i;
    word w1;

    for (w1=1; w1<=w; w1++)
      {
       for (i=0; i <= timeVal; i++)  //25 for 2mhz.  100 for 8mhz.
               ;
      }
  } /* end time */
/*************************/

word timerVal(byte n)
 {
   word w;

   t0_interrupt=1;
   while (t0_interrupt)
    {
     t0_interrupt=0;
     w = timer[n];
    }
    return(w);
 }/* end timerVal */
/********************/

void loadTimer (byte n, word w)
 {
   t0_interrupt=1;
   while (t0_interrupt)
    {
      t0_interrupt=0;
      timer[n] = w;
    }
 }
  /* end loadTimer */
/************************/

      
void transmitByte (byte n)
  {


    while (!tiFlag)  /*wait until uart free */
     {

     }

   tiFlag = 0;
   SBUF = n;  /*TRANSMIT 1 CHAR*/


  } /*end void transmitByte (byte n)*/
/***************************************/


void transmitCrLf()
 {
   transmitByte(CR);
   transmitByte(LF);
 }// end void transmitCrLf()
/*****************************/ 

void transmitStringNow( char code *ptr, bit withCrLf)  //   transmitStringNow("wait!  now", 1); // with CR LF
 {
   char c;
      
   while (*ptr)
     {
        c = *ptr;
        transmitByte(c);
        ptr++; //next elements.
     }    
     
   if (withCrLf)
       transmitCrLf();
                          
 }/* end transmitStringNow  */
/*******************************/ 


void delay(byte n) // cycle for ADUC842: 20 + 10*n  , include call and return.
{
 byte i;
 
 for (i=0; i<n; i++)
   {

   }  
}// end delay()
/**********************/



word adc842 (byte n)
 {
   word w;
//   byte i;
   
#ifdef TEST_ADC
    w = (word) n<<8; // n*256
    w += n;   // n*257  2 byte with chan number.
     
    return w;
#endif

     
   ADCCON2 = n; /* request channel */

   SCONV = 1; /* start convert */

   while (ADCCON3 >= 0x80) /* wait for busy */
       ;

   w = ADCDATAH & 0x0f; /* only 4 low bits */
   w <<= 8; 
   w += ADCDATAL;

   return w;

 } /* end adc842 */
 /*******************/



void dac812(bit select, word w) /* select=1 for DAC1 */
 {
   byte i;

   HIGH(w,i)
   if (select)
    {
      DAC1H = i;  /* msb first without update */
      DAC1L = w; /*low  -> update DAC */
    }
   else /* for DAC0 */
    {
      DAC0H = i;
      DAC0L = w; /*low  -> update DAC */
    }

 } /* end dac812(word w) */
 /*************************/


 bit charFromKlotBuf (byte data *n ) /* return false if empty */
  {
   static byte charOutKlotBufPtr=0;

   if (!charInKlotBuf)
     return (0);
   *n = klotBuf[charOutKlotBufPtr];
   charOutKlotBufPtr ++;
   if ( charOutKlotBufPtr> sizeof(klotBuf)-1 ) /*last*/
      charOutKlotBufPtr = 0;
   charInKlotBuf--;
   return (1);
  } /* end byte charFromKlotBuf ()
 /**********************************/


void charToKlotBuf (byte n)  using 2
 {
   static byte charInKlotBufPtr=0;
   if ( charInKlotBuf >= sizeof(klotBuf) )
    {
       return;
    }
   klotBuf[charInKlotBufPtr] = n;
   charInKlotBufPtr ++;
   if (charInKlotBufPtr > sizeof(klotBuf) - 1) /*last*/
        charInKlotBufPtr = 0;
   charInKlotBuf++;
 } /* end charToKlotBuf */
/***************************/

void klot () interrupt 4 using 2
 {
   byte i;

   if (TI)
     {
       TI=0;
       tiFlag = 1;
     }

   if (!RI)
     return;
   RI = 0;
   i = SBUF;
   charToKlotBuf(i);

 } /* end void klot ()  */
 /********************************/

#ifdef XXX
bit parseMsgOld()
 {
   byte i;
   static byte paramBufPtr=0;
   static byte klotPhase=0, cntParam;

   if ( !charFromKlotBuf (&i) )   /* return false if empty */
        return (0);

   if (i == HEADER)
        klotPhase = 0;

   switch (klotPhase)
    {
     case 0: //wait for rcv HEADER.
             if (i != HEADER)
               return (0);
             paramBufPtr = 0;
             klotPhase = 1;
             return (0);

      case 1:
             frame = i;
             if (frame > MAX_FRAME)
              {
                 klotPhase=0;
                 return(0);
              }

             cntParam = paramTab[frame];
             if (!cntParam)
              {
                 klotPhase=0;
                 return(1);    // new_command without parameters.
               }
              klotPhase = 2;
              return(0);

      case 2:
              paramBuf[paramBufPtr] = i;
              cntParam--;
              paramBufPtr++;
              if (!cntParam) 
               {
                  klotPhase = 0;
                  return (1);  // newCommand.
               }
              return(0);

     default:
             return(0);

   } /* end switch */
   
} /* end bit parseMsgOld() */
/**************************/
#endif 

 
bit parseMsg()
 {
   byte i;
   //bit ok; 
   static byte paramBufPtr=0;
   static byte klotPhase=0, cntParam;

   while ( charFromKlotBuf (&i) )   /* return false if FIFO empty */
     {
      if (i == HEADER)
           klotPhase = 0;
   
      switch (klotPhase)
       {
         case 0: //wait for rcv HEADER.
                if (i != HEADER)
                  {      
                   //ok = 0;
                   break;
                  }
                paramBufPtr = 0;
                klotPhase = 1;
                //ok = 0;
                break;
   
         case 1:
                frame = i;
                if (frame > MAX_FRAME)
                 {
                    klotPhase=0;
                    //ok = 0;
                    break;
                 }
   
                cntParam = paramTab[frame];
                if (!cntParam)    // new command without parameters.
                 {
                    klotPhase=0;
                    //ok = 1;
                    return(1);
                 }
                 klotPhase = 2;
                 //ok = 0;
                 break;
   
         case 2:
                 paramBuf[paramBufPtr] = i;
                 paramBufPtr++;
                 cntParam--;
                 if (!cntParam)    // new Command.
                  {
                     klotPhase = 0;
                     //ok = 1;
                     return(1);
                  }
                 //ok = 0;
                 break;
   
      } /* end switch */

     }// end while() . no more data in FIFO.

    return (0);
   
} /* end bit parseMsg() */
/**************************/


void psika() interrupt 1 using 1     /* evry 5 msec   */
  {
    byte i;
 
    TL0 = timer0Lo;
    TH0 = timer0Hi;
    TR0=1;

      
    for (i=0; i< totalTimers; i++)
      {
        if (timer[i])
           timer[i]--;
      }
   
    t0_interrupt = 1;  // was interrupt.

    newData = 1;
   
  }  /*  END PSIKA */
/*********************/


void transmitWord(word w) 
  {
   byte i;

   i = w; //lo
   transmitByte(i);
     
   i = w>>8; // hi
   transmitByte(i);
   
  } /* end void transmitWordCheck(word w) */
/********************************************/  


void transmitAscii (word w)     //transmit ascii
{
  byte n=0, array[5];

  do
    {
    array[n] = w % 10;
    w = w / 10;
    n++;
    }while (w);
  n--;
  for ( ; n < 5 ; n--)
    transmitByte (array[n] + 0x30);
           
} /* end void transmitAscii (word w) */
 /***************************/


void moveClose()
{
   POWER_LED = LED_NO_ACTIVE_VAL; // look for action
   MOTORS_PORT = MOVE_LEFT;
   time(mDelay);
   POWER_LED = LED_ACTIVE_VAL;   // look for action
   MOTORS_PORT = MOVE_FORWARD;
   time(mDelay);
   MOTORS_PORT = MOVE_RIGHT;	
   time(mDelay);
   MOTORS_PORT = MOVE_FORWARD;
   time(5000);// 0.5Sec
} /* end void moveClose() */
 /***************************/
              
void moveAway()
{
 POWER_LED = LED_NO_ACTIVE_VAL; // look for action
	MOTORS_PORT = MOVE_RIGHT;
 time(mDelay);
 POWER_LED = LED_ACTIVE_VAL;   // look for action
	MOTORS_PORT = MOVE_FORWARD;
	time(mDelay);
	MOTORS_PORT = MOVE_LEFT;
	time(mDelay);
	MOTORS_PORT = MOVE_FORWARD;
 time(5000);// 0.5Sec
} /* end void moveAway() */
 /***************************/

word distanceSensorVal(bit untilOk)  //  
  {
    word idata vec[4];
    word w, min , max; 
    byte n;
    bit ok;

    word result = 0;
    do
      {
        w = adc842(DISTANCE_SENSOR_REAR);
        for (n=0; n <= 3; n++)
          {
            time(10); //1mSec
            w = adc842(DISTANCE_SENSOR_REAR);
            vec[n] = w;
          }
      
    
         /* calculate min, max */
        min = 4095; max = 0;
        for (n=0; n <= 3; n++)
          {
            w = vec[n];
            if (w < min) min = w;
            if (w > max) max = w;
          }
    
         /* check validity */
        ok =  ( (max - min) <= maxDelta );
        if ( (!ok) & (!untilOk) )   // for only one trial.
                return 5000;
        }
       while ( (!ok));

     /* good result */
     result = 0;
     for (n=0; n <= 3; n++)
        result += vec[n];
     result += 2;  // rounded result
     result >>= 2;  // devide by 4
     return (result);
  }/* word distanceSensorVal()  */
/*************************************/


void commandExec ()
 {
   byte i;
   word w;

   if ( !parseMsg() )
        return;

   switch (frame)
    {
      case 0: // send IDLE 
              transmitByte (HEADER);
              transmitByte (1);
              transmitByte (2);
              transmitByte (3);
              break;
      	
      
      case 1: //// moveAway()  n times.    n=0 infinit time.
              while(!paramBuf[0])
                {
                   moveAway();
                   time(20000);
                   if ( charFromKlotBuf (&i )  )  /* return false if empty */
                     {
                       MOTORS_PORT = MOVE_STOP;
                       return;
                      }
                } 
                            
              for(i=0; i< paramBuf[0];i++)
                {
                  moveAway();
                  //time(20000);
               }
              MOTORS_PORT = MOVE_STOP;

              transmitByte (HEADER);
              transmitByte (1);
              break;
                    
      case 2: // saveLimitsRun function
              saveLimitsRun = paramBuf[0];
              MOTORS_PORT = MOVE_STOP;

              transmitByte (HEADER);
              transmitByte (2);
              break;
                 
      case 3: //// moveClose()  n times.    n=0 infinit time.
              while(!paramBuf[0])
                {
                   moveClose();
                   //time(20000);
                   if ( charFromKlotBuf (&i )  )/* return false if empty */
                     {
                       MOTORS_PORT = MOVE_STOP;
                       return;
                      }
                } 
                            
              for(i=0; i< paramBuf[0];i++)
                {
                  moveClose();
                  time(20000);
               }
              MOTORS_PORT = MOVE_STOP;
              transmitByte (HEADER);
              transmitByte (3);
              break;
                   

      case 4://// motors time delay
              mDelay = paramBuf[0];
              mDelay *= 1000;
              transmitByte (HEADER);
              transmitByte (4);
              break;

      case 5:// send distance sensor n times  , n=0 for infinit.
              while(!paramBuf[0])
                {
                  w = distanceSensorVal(0);  // '0' for 0ne trial.
                  if(paramBuf[1])
                      w >>= 4;
                  transmitAscii (w);
                  transmitByte(' ');
                  time(10000);// 1Sec
                  if ( charFromKlotBuf (&i )  )/* return false if empty */
                          return;
                } 
                            
              for(i=0; i< paramBuf[0];i++)
                {
                  w = adc842 (DISTANCE_SENSOR_REAR);
                  w = adc842(DISTANCE_SENSOR_REAR);  // twice, ADUC operation.
                    if(paramBuf[1])
                      w >>= 4;
                  transmitAscii (w);
                  transmitByte(' ');
                  time(10000);// 1Sec
               }
              transmitByte (HEADER);
              transmitByte (5);
              break;
      
      case 6: // run forward/revers  n time (second), 0 for infinit.
              if(paramBuf[0])           //  run forward else run reverse.
                  MOTORS_PORT = MOVE_FORWARD;
              else
                  MOTORS_PORT = MOVE_BACK;

              while(!paramBuf[1])     // for infinit
                {
                  if ( charFromKlotBuf (&i )  )/* return false if empty */
                    {
                       MOTORS_PORT = MOVE_STOP;
                       return;
                    }
                }

              POWER_LED = LED_NO_ACTIVE_VAL;
              for(i=0; i< paramBuf[1];i++)
                {                        
                  time(10000);        // 1 sec
                }

              POWER_LED = LED_ACTIVE_VAL;
              MOTORS_PORT = MOVE_STOP;
        		    transmitByte (HEADER);
       			    transmitByte (6);
       			    break;
             
      case 7: //correct time()  value.
	             timeVal = paramBuf[0];    
       			    transmitByte (HEADER);
              transmitByte (7);
              break;
	 
       case 8: // enable/disable saveLimits()  with pseudo values.
              pseudoSensor =  paramBuf[0];
       			    transmitByte (HEADER);
              transmitByte (8);
              break;

       case 9: // set pseudo Sensor Value
              pseudoSensorVal = 0;
              for(i=0; i <= 3; i++ )   // convert 4 digits to word.
                {
                  pseudoSensorVal *= 10;
                  pseudoSensorVal += paramBuf[i];
                }
       			    transmitByte (HEADER);
              transmitByte (9);
              break;

       case 10: // maxDelta value;
              maxDelta = paramBuf[0];
       			    transmitByte (HEADER);
              transmitByte (10);
              break;
                                     
      default:  
              transmitStringNow("command not exist!", 1);
              break;


    } /* end switch */

 } /* end commandExec */
/***********************/


void callOnce() // do nothing - only for compiler !!
  {
   byte i;
   
   if (i == i ) // so, this function never called 
       return;

			transmitWord(0);
   delay(1); // cycle for ADUC842: 20 + 10*n  , include call and return.
   dac812(1, 1); /* select=1 for DAC1 */
  } /* end void callOnce(); */
  /******************************/

void calibrate842()
{
  /* offset */
  ADCCON1 = 0x8c; // ADC on; ADCCLK set to divide by 32, 4 acquisition clock
  ADCCON2 = 0x0B; //select internal AGND
  ADCCON3 = 0x25; //select offset calibration 31 averages per bit, offset calibration, calibration start!!
  
  while (ADCCON3 >= 0x80) /* wait for busy */
       ;
  
  /* gain */
  ADCCON2 = 0x0C; //select internal VREF
  ADCCON3 = 0x27; //select gain calibration, 31 averages per bit, calibration start!!

  while (ADCCON3 >= 0x80) /* wait for busy */
       ;
  
}//end void calibrate842()
/***************************/


void init ()
  {
    byte i;
                            /* 8031 FUNCTION */
    IE=0;
    TCON=0;          /*DISABLE TIMERS*/
    TH1 = TL1 = 254;      /* NOT USED - T2 SETUP FOR 38800 BPS.       28800 = 254    ,  9600B=250 */
    
    /*   UART stop bit */
    //SCON=0xDE;        /* UART IN MODE 3,   9 BIT    -->> 2 STOP bit */
    //SCON=0x5E;        /* UART IN MODE 1,   8 BIT    -->> 1 STOP bit */
    SCON=0x5E;        /* 1 STOP BIT */
    
    PCON=0x80;/* DOUBLE RATE*/
    
     LOW(T0_VALUE_ADUC842, timer0Lo);// 5 mSEC 
     HIGH(T0_VALUE_ADUC842, timer0Hi);
    
    
    TMOD =0x21;         /*T0 MODE1,T1 MOD 2*/
    ET0=1; PT0 = 0;     /*    T0 INTERRUPT ,   T0 priority */
    ES = 1; PS = 1;     /*   UART interrupt,    UART priority */
    
    /*                          BPS  SETUP    */
    
   // 115200 BPS @ 2.097152 Mhz
   // T3CON = 0x80;  //DIV=0
   // T3FD = 9;
   
     /* 115200 BPS at 8.388608Mhz  */
       T3CON = 0x82; //0x80 + DIV
       T3FD = 9;
      
   
     /*  setting core clock  PLLCON:
         0=16.777216      1=8.388608,       2=4.194304         3=2.097152,       
         4=1.048576       5=0.524288        6=0.262144         7=0.131072   
     */
   
   //  PLLCON = 3; // default:3 for 2.097152Mhz
       PLLCON = 1;   //  for 8.388608Mhz
   
   
                  /* enable aduc842  ADC, (ck1,ck2)=4 , internal VREF = 1001-1100=0x9c    */
   /*
     ADCCON1: bit7:power enable, bit6:'1' for external ref, bit5+bit4: ck1+ck0 ADC clock devider,
              bit3+bit2:aq1+aq0 trake&hold time, bit1:T2c T2 trigger, bit0: EXC external trigger
               ADC clock:  400Khz to 8.3Mhz
   */
   //ADCCON1 = 0xfc; /* 1001-1100  enable ADC, (ck1,ck2)=devide2  */
   ADCCON1 = 0xbc;  // internal VREF
   
              /* ADC   aduc842 calibration */
   calibrate842();   // change ADCCON1 !!
   //ADCCON1 = 0xfc; /* again !! calibrate() change it.  1001-1100  enable ADC, (ck1,ck2)=devide2  */
   ADCCON1 = 0xbc;  
   


   /* enable DAC */
   //DACCON = 0x0d; /* dac0=ON, dac1=off, range: 0 - Vref */
   DACCON = 0x1f; /* dac0=dac1=ON, range: 0 - Vref */
   
   
   timeVal = 75; //125; //function time() , correct value from commandExec() .
   /* blink power led */
   
   for(i=1; i<=5; i++)  // enshure stabilize analog system. 
    {
      POWER_LED = LED_ACTIVE_VAL;
      time(5000); /* 0.5Sec */
   
      POWER_LED = LED_NO_ACTIVE_VAL;
      time(5000); /* 0.5Sec */
    }
   
   POWER_LED = LED_ACTIVE_VAL;

       
   /* enable SPI  */
   
     /* for aduc842 :   osc/2  00=2,  01=4,  10=8,  11=16 */
   SPR1 = 0; SPR0 = 0;
   SPIM = 1; // master mode.
   CPOL = 0; // clock low in idle time.
   CPHA = 0; // transmit at trailing adge, receive in leading adge.
   
   SPE = 1; // enable (disable I2C).
   
   loLimit =  606*2; //  for 2.5 vref 40 cm  about 0.74V  = 1212      for 5 vref = 606
   hiLimit =  417*2;	//  for 2.5 vref 60 cm  about  0.51V = 835       for 5 vref = 417
   midLimit = 498*2; //  50 cm about 6.1v for 5 vref 498 
   pseudoSensorVal = 500;// value for pseudo Sensor.
   pseudoSensor = 0; 
                      
   
   enableSendSamples = 1;
   callOnce();
   CFG842 = 1; //CFG842.0 = 1,  for internal XDATA  ram.
   
   saveLimitsRun = 0; // stop this function 
   MOTORS_PORT = MOVE_FORWARD;
   time(20000); // 2 Sec
   MOTORS_PORT = MOVE_STOP;
   time(2000);
   MOTORS_PORT = MOVE_BACK;
   time(20000);
   MOTORS_PORT = MOVE_STOP;
   maxDelta = 10;
   
   mDelay = 10000; // 1 sec for savelimits fuction

   TR0 = 1;
   EA = 1;     /* ENABLE INTERRUPT*/

  } /* END INIT */
/*******************/

void saveLimits()
  {
   static word s0;
   static byte p = 0;
   static bit  awayClose;   // away=1,  close=0.

   if (!saveLimitsRun)
     return;

   switch (p)
     {
       case 0:
         s0 = distanceSensorVal(1);  // '1' for 'until ok '.
         if (pseudoSensor)
            s0 = pseudoSensorVal;

         transmitAscii (s0);
         transmitByte(' ');     

       		if ((s0 <= loLimit) && (s0 >= hiLimit))		// in valid range.  40cm to 60 cm
           {
        		  MOTORS_PORT = MOVE_FORWARD;
            transmitByte(' ');
            transmitByte('N');
            transmitByte(' ');
            loadTimer (rangeTimer,100 ); // 0.5Sec before saveLimits()  again.
            break;
           }
         // for invalid range. 
         p = 1;
         awayClose = (s0 > loLimit);
         break;

       case 1:
         			if(awayClose)  
              {
                transmitByte(' ');
                transmitByte('A');
                transmitByte(' ');
           			  moveAway(); 
               }
            else  
               {
                transmitByte(' ');
                transmitByte('C');
                transmitByte(' ');
           			  moveClose();
                }

            /* check for good result  */
            s0 = distanceSensorVal(1);  // '1' for 'until ok '.
            if (pseudoSensor)
                s0 = pseudoSensorVal;

            transmitAscii(s0);
            transmitByte(' '); 
            if (awayClose)
              {
                if (s0 > midLimit) // need more correct.
                    break;
              }
            else
             {
                if (s0 < midLimit) // need more correct.
                break;
             }

            /* good result */
            loadTimer (rangeTimer,100 ); // 0.5Sec before saveLimits()  again.
            p = 0;
    }
  }/* end  void saveLimits()  */
  /*********************************/


void main()
{

 init();
 
 for(;;)
  {
    if (!timerVal(rangeTimer))  // timer finished.
      {
        saveLimits();
      }
    commandExec();

  }
}
    