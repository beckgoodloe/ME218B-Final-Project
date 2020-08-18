/****************************************************************************
 Module
   I2CService.c

 Revision
   1.0.1

 Description
   Sample I2C interface file based on the TCS34725 Color sensor

 Notes
    Implements a generic interface based on lists of steps to be executed
    in response to external requests. The basic state machine will step
    though executing the steps with pauses based on waiting for the I2C 
    step to complete or on time, since some devices require a delay between
    issued commands.
 
    
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// standard library includes
#include <stdint.h>
#include <stdbool.h>

// gets us address of I2C peripheral
#include "inc/hw_memmap.h"

//includes for TivaWare library references
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"

// The usual Framework headers
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"

// The info on the device that we will be talking to
#include "TCS3472x.h"

//Finally our service header
#include "I2CService.h"


/*----------------------------- Module Defines ----------------------------*/
#define DEVICE_ADDR TCS3472x_ADDRESS

// do transaction with/without stop generation
#define DO_STOP  true
#define NO_STOP  false

// do transaction with/without start generation
#define DO_START  true
#define NO_START  false

// are we aiting on busy or time?
#define BUSY_WAIT  true
#define TIME_WAIT  false

// confirm that these match the order in the Sequence Lists
#define INIT_INDEX 0
#define READ_CLR   1
#define READ_RED   2
#define READ_GRN   3
#define READ_BLU   4


// Integration Time, in ms for TCS3472x
// make sure that this matches constant programmed in Init
#define INT_TIME 350

/*----------------------------- Module Types ----------------------------*/
typedef enum    /* definitions for the possible steps in command sequence */
{
  CMD_Write8,     /* write 8 bits to I2C with start & stop */
  CMD_Write8NS,   /* write 8 bits to I2C with no start but with stop */  
  CMD_WriteMult,  /* write 8 bits to I2C with start but with-out stop */
  CMD_WriteMultNS,/* write 8 bits to I2C with no start or stop */
  CMD_Read8,      /* read 8 bits to I2C with start & stop */
  CMD_Read8NS,    /* read 8 bits to I2C with no start but with stop */  
  CMD_ReadMult,   /* read 8 bits to I2C with start but with-out stop */
  CMD_ReadMultNS, /* read 8 bits to I2C with no start or stop */
  CMD_GetResult,  /* fetches the result of the last read operation */
  CMD_Form16,     /* combine 2 8-bit temp values into 16bit result */
  CMD_NOP,        /* No-Operation, used for time-only delays */
  CMD_EOS         /* End Of Sequence marker */
} CMD_t;

typedef struct  /* definition of each step */
{
  CMD_t Command;
  uint8_t Value;
  uint16_t WaitTime;
  bool BusyWait;
  void * Result;
}StepDefinition_t;

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.
*/
static void I2C0_Init(void);
static void I2C0_Write1Byte( uint8_t slave_addr, uint8_t value, bool InclStart, bool InclStop);
static void I2C0_Read1Byte( uint8_t slave_addr, bool InclStart,  bool InclStop);
static void InterpretCommand(StepDefinition_t CurrentStep);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match that of enum in header file
static I2CState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// These are the local varaibles for the color sensor
static uint8_t readOne;   // temp location for low byte of 16 bit reads
static uint8_t readTwo;   // temp location for hi byte of 16 bit reads
static uint16_t ClearValue;
static uint16_t RedValue;
static uint16_t GreenValue;
static uint16_t BlueValue;

// index for which of the posible command sequences we are executing
static uint8_t CommandIndex;
// index for stepping through the steps in a command
static uint8_t StepIndex;
// Content of the current step in the sequence
static StepDefinition_t CurrentStep;

static const StepDefinition_t SequenceLists[5][9] = {
  /* First up is the power-up initialization sequence */
  { {CMD_WriteMult, (TCS3472x_ENABLE_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the Enable register */
    {CMD_Write8NS, TCS3472x_ENABLE_PON, 0, BUSY_WAIT, NULL}, /* set the PON bit to power up */
    {CMD_NOP, 0, 4, TIME_WAIT, NULL}, /* time wait for min. 3ms with timer uncertainty */
    {CMD_WriteMult, (TCS3472x_ATIME_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the ATIME register */
    {CMD_Write8NS, TCS3472x_INT_TIME_350MS, 0, BUSY_WAIT, NULL}, /* program the integration time */
    {CMD_WriteMult, (TCS3472x_ENABLE_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the Enable register */
    {CMD_Write8NS, (TCS3472x_ENABLE_PON | TCS3472x_ENABLE_AEN), 0, BUSY_WAIT, NULL}, /* set the PON & AEN bits to start conversion */
    {CMD_NOP, 0, (INT_TIME+1), TIME_WAIT, NULL}, /* time wait for min. INT_TIME with timer uncertainty */
    {CMD_EOS, 0, 0, TIME_WAIT, NULL} /* mark the end of this sequence */
  },
  /* next is read the clear results */
  { {CMD_WriteMult, (TCS3472x_CDATAL_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the lo byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the low byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readOne}, /* fetch result and move to next step */
    {CMD_WriteMult, (TCS3472x_CDATAH_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the hi byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the hi byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readTwo}, /* fetch result and move to next step */
    {CMD_Form16, 0, 0, TIME_WAIT, &ClearValue}, /* combine the 2 8-bit resutls */
    {CMD_EOS, 0, 0, TIME_WAIT, NULL} /* mark the end of this sequence */
  },
  /* next is read the red results */
  { {CMD_WriteMult, (TCS3472x_RDATAL_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the lo byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the low byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readOne}, /* fetch result and move to next step */
    {CMD_WriteMult, (TCS3472x_RDATAH_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the hi byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the hi byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readTwo}, /* fetch result and move to next step */
    {CMD_Form16, 0, 0, TIME_WAIT, &RedValue}, /* combine the 2 8-bit resutls */
    {CMD_EOS, 0, 0, TIME_WAIT, NULL} /* mark the end of this sequence */
  },
  /* next is read the green results */
  { {CMD_WriteMult, (TCS3472x_GDATAL_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the lo byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the low byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readOne}, /* fetch result and move to next step */
    {CMD_WriteMult, (TCS3472x_GDATAH_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the hi byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the hi byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readTwo}, /* fetch result and move to next step */
    {CMD_Form16, 0, 0, TIME_WAIT, &GreenValue}, /* combine the 2 8-bit resutls */
    {CMD_EOS, 0, 0, TIME_WAIT, NULL} /* mark the end of this sequence */
  },
  /* next is read the blue results */
  { {CMD_WriteMult, (TCS3472x_BDATAL_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the lo byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the low byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readOne}, /* fetch result and move to next step */
    {CMD_WriteMult, (TCS3472x_BDATAH_REG | TCS3472x_COMMAND_BIT), 0, BUSY_WAIT, NULL}, /* select the hi byte result register */
    {CMD_Read8, 0, 0, BUSY_WAIT, NULL}, /* read the hi byte value */
    {CMD_GetResult, 0, 0, TIME_WAIT, &readTwo}, /* fetch result and move to next step */
    {CMD_Form16, 0, 0, TIME_WAIT, &BlueValue}, /* combine the 2 8-bit resutls */
    {CMD_EOS, 0, 0, TIME_WAIT, NULL} /* mark the end of this sequence */
  }
};

// this flag is used by the event checker to detect the busy-not-busy transition
static bool I2C0_Busy;

// add a deferral queue for up to 5 pending deferrals of commands
// +1 to allow for ovehead in queue
// make sure that the I2Cservice queue is at least this size as well.
static ES_Event_t DeferralQueue[5 + 1];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitI2CService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 01/15/19, 15:18
****************************************************************************/
bool InitI2CService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // initialize deferral queue to allow for commands that arrive before
  // the last sequence was conpleted
  ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue));
  // put us into the Initial PseudoState
  CurrentState = InitPState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostI2CService

 Parameters
     EF_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 01/15/19, 15:20
****************************************************************************/
bool PostI2CService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunI2CService

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/19, 15:23
****************************************************************************/
ES_Event_t RunI2CService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // Initialize the I2C module
        I2C0_Init();
        // now put the machine into the idle state
        CurrentState = Idle;
        // and post a message to ourselves to start the init sequnce
        { 
          ES_Event_t ThisEvent;
          ThisEvent.EventType = EV_I2C_InitSensor;
          PostI2CService( ThisEvent );
        }
      }
    } // end of InitPState processing
    break;

    case Idle:
    {
      switch (ThisEvent.EventType)
      {
        case EV_I2C_InitSensor:  
        {  
          // Set up the indices into the command sequence lists
          CommandIndex = INIT_INDEX; // start with the initialization sequence
        }
        break;

        case EV_I2C_ReadClear:  
        {  
          // Set up the indices into the command sequence lists
          CommandIndex = READ_CLR; // read the clear result
        }
        break;

        case EV_I2C_ReadRed:  
        {  
          // Set up the indices into the command sequence lists
          CommandIndex = READ_RED; // read the clear result
        }
        break;

        case EV_I2C_ReadGreen:  
        {  
          // Set up the indices into the command sequence lists
          CommandIndex = READ_GRN; // read the clear result
        }
        break;

        case EV_I2C_ReadBlue:  
        {  
          // Set up the indices into the command sequence lists
          CommandIndex = READ_BLU; // read the clear result
        }
        break;

        case EV_I2C_ReadAll:  
        {  
          // we read all by reading each color + clear
          ES_Event_t ThisEvent;
          ThisEvent.EventType = EV_I2C_ReadClear;
          PostI2CService( ThisEvent );
          ThisEvent.EventType = EV_I2C_ReadRed;
          PostI2CService( ThisEvent );
          ThisEvent.EventType = EV_I2C_ReadGreen;
          PostI2CService( ThisEvent );
          ThisEvent.EventType = EV_I2C_ReadBlue;
          PostI2CService( ThisEvent );
          CommandIndex = 0xff; // we didn't set up a list so skip that stuff
        }
        break;

        case ES_TIMEOUT:  /* This is how we get repeating measurements */
        {  
          if(I2C_TIMER == ThisEvent.EventParam) // our timer?
          {
            ES_Event_t ThisEvent;
            ThisEvent.EventType = EV_I2C_ReadAll;
            PostI2CService( ThisEvent );
          }
          CommandIndex = 0xff; // we didn't set up a list so skip that stuff
        }
        default: // these are events that we don't process so flag that
        {
          CommandIndex = 0xff; // flag event as one we don't process
        }  
        break;
      }  // end of switch on ThisEvent
      
      // code common to all legal transitions
      if (0xff != CommandIndex) // don't do this unless we are setting up a list
      {
        StepIndex = 0; //start at the first step
          
        // put the machine into the interpreting state
        CurrentState = Interpreting;
        // and post a message to ourselves to take the next step in the sequence
        { 
          ES_Event_t ThisEvent;
          ThisEvent.EventType = EV_I2C_NextStep;
          PostI2CService( ThisEvent );
        }
      }
    } // end of Idle state processing
    break;

    case Interpreting:        
    {
      switch (ThisEvent.EventType)
      {
        case EV_I2C_NextStep:  
        {   
          // fetch the current instruction and inc the index to the next instruction
          CurrentStep = SequenceLists[CommandIndex][StepIndex++];
          InterpretCommand(CurrentStep);
        }
        break;

        case EV_I2C_Wait4Busy:  
        {   
          CurrentState = Waiting4Busy;
        }
        break;

        case EV_I2C_Wait4Time:  
        {   
          if( 0 != CurrentStep.WaitTime )
          {
            CurrentState = Waiting4Time;
            ES_Timer_InitTimer(I2C_TIMER, CurrentStep.WaitTime);
          }else
          {   // if wait time is 0, then move to the next step immediately      
            ES_Event_t ThisEvent;
            ThisEvent.EventType = EV_I2C_NextStep;
            PostI2CService( ThisEvent );
            CurrentState = Interpreting;
          }
        }
        break;

        case EV_I2C_EOS:  
        {   
          CurrentState = Idle;
          // recall any events that were deferred while processing
          ES_RecallEvents(MyPriority, DeferralQueue);
        }
        break;

        // if we didn't process it in this state, then defer it.
        default:
        {
          ES_DeferEvent(DeferralQueue, ThisEvent);
        }  
        break;
      }  // end of switch on ThisEvent
    } // end of Interpreting state processing
    break;
    
    case Waiting4Busy:        
    {
      if( EV_I2C_StepFinished == ThisEvent.EventType)
      {
        // if no error, do the next step in the sequence
        if (I2C_MASTER_ERR_NONE == ROM_I2CMasterErr(I2C0_BASE))
        {  
          ES_Event_t ThisEvent;
          ThisEvent.EventType = EV_I2C_NextStep;
          PostI2CService( ThisEvent );
          CurrentState = Interpreting;
        }else // otherwise go back to Idle
        { 
          puts("Error in I2C");
          CurrentState = Idle;
        }
      }
      else
      {
        // if we didn't process it in this state, then defer it.
        ES_DeferEvent(DeferralQueue, ThisEvent);
      }
    } // end of Waiting4Busy state processing
    break;
    
    case Waiting4Time:        
    {
      if( (ES_TIMEOUT == ThisEvent.EventType) &&
          (I2C_TIMER == ThisEvent.EventParam))
      {
        ES_Event_t ThisEvent;
        ThisEvent.EventType = EV_I2C_NextStep;
        PostI2CService( ThisEvent );
        CurrentState = Interpreting;
      }else 
      {
        // if we didn't process it in this state, then defer it.
        ES_DeferEvent(DeferralQueue, ThisEvent);
      }
    } // end of Waiting4Time state processing
    break;

    // repeat state pattern as required for other states
    default:
    {
    }
    break;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryI2CService

 Parameters
     None

 Returns
     I2CState_t The current state of the state machine

 Description
     returns the current state of the I2C state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
I2CState_t QueryI2CService(void)
{
  return CurrentState;
}

// Event checker to test when an I2C tranfer completes
bool IsI2C0Finished(void)
{
  if (( true == I2C0_Busy ) && (true != ROM_I2CMasterBusy(I2C0_BASE)))
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType = EV_I2C_StepFinished;
    PostI2CService( ThisEvent );
    I2C0_Busy = false;   
    return true;
  }else
  {
    return false;
  }
}

uint16_t I2C_GetClearValue( void )
{
  return(ClearValue);
}
  
uint16_t I2C_GetRedValue( void )
{
  return(RedValue);
}
  
uint16_t I2C_GetGreenValue( void )
{
  return(GreenValue);
}
  
uint16_t I2C_GetBlueValue( void )
{
  return(BlueValue);
}


/***************************************************************************
 private functions
 ***************************************************************************/
// Initialize the I2C module using the TivaWare library
static void I2C0_Init(void){
  //enable I2C module 0
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

  //wait for the clock to be ready
  while (ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0) != true)
  {}

  //reset I2C module 0
  ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

  //enable GPIO port that contains I2C 0, Port B
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
 
  //wait for the clock on the port to be ready
  while (ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB) != true)
  {}

  // Configure the pin muxing for I2C0 functions on port B2 and B3.
  ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
		
  // Configure the pins for I2C, including enabling the PU on SCL.
  ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // SCL
  ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);    // SDA
				
  // Enable and initialize the I2C0 master module.  Use the system clock for
  // the I2C0 module.  The last parameter sets the I2C data transfer rate.
  // If false the data rate is set to 100kbps and if true the data rate will
  // be set to 400kbps.
  ROM_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
     
}

// set up device address and write 1 byte
static void I2C0_Write1Byte( uint8_t slave_addr, uint8_t value, bool InclStart, 
                             bool InclStop)
{
  uint32_t I2C_Command; // what command to send to the HW
  
  //specify address of the slave device.
  //Writing is indicated by the false in the last parameter
  ROM_I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

  //specify data to be written
  ROM_I2CMasterDataPut(I2C0_BASE, value);

  // are we sending a start?
  if( true == InclStart )
  {
    //are we sending a stop?
    if( true == InclStop )
    {
      // this will generate a (possibly repeated) start followed by a Stop
      I2C_Command = I2C_MASTER_CMD_SINGLE_SEND;
    }else
    {
    // this will send a (possibly repeated) start with no Stop
      I2C_Command = I2C_MASTER_CMD_BURST_SEND_START;
    } 
  }else  // no start here
  {
    //are we sending a stop?
    if( true == InclStop )
    {
      // this will send a byte followed by a Stop
      I2C_Command = I2C_MASTER_CMD_BURST_SEND_FINISH;
    }else
    {
    // this will send a byte with no Stop
      I2C_Command = I2C_MASTER_CMD_BURST_SEND_CONT;
    }  
  }
  // now issue the command
  ROM_I2CMasterControl(I2C0_BASE, I2C_Command);
  I2C0_Busy = true;
}

// set up device address and read 1 byte
static void I2C0_Read1Byte( uint8_t slave_addr, bool InclStart, 
                             bool InclStop)
{
  uint32_t I2C_Command; // what command to send to the HW
  
  //specify address of the slave device.
  //reading is indicated by the true in the last parameter
  ROM_I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

  // are we sending a start?
  if( true == InclStart )
  {
    //are we sending a stop?
    if( true == InclStop )
    {
      // this will generate a (possibly repeated) start followed by a Stop
      I2C_Command = I2C_MASTER_CMD_SINGLE_RECEIVE;
    }else
    {
    // this will generate a (possibly repeated) start with no Stop
      I2C_Command = I2C_MASTER_CMD_BURST_RECEIVE_START;
    } 
  }else  // no start here
  {
    //are we sending a stop?
    if( true == InclStop )
    {
      // this will receive a byte followed by issuing a Stop
      I2C_Command = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
    }else
    {
    // this will receive a byte with no Stop
      I2C_Command = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
    }  
  }
  // now issue the command
  ROM_I2CMasterControl(I2C0_BASE, I2C_Command);
  I2C0_Busy = true;
}

// I pulled this code out into a function in order to compact the state
// machine code and make it easier to see its structure.
static void InterpretCommand(StepDefinition_t CurrentStep)
{
  switch (CurrentStep.Command) // this is the interpretation of the commands
  {
    case CMD_Write8:
    {
      I2C0_Write1Byte(DEVICE_ADDR, CurrentStep.Value, DO_START, DO_STOP);
    }
    break;

    case CMD_Write8NS:
    {
      I2C0_Write1Byte(DEVICE_ADDR, CurrentStep.Value, NO_START, DO_STOP);
    }
    break;

    case CMD_WriteMult:
    {
      I2C0_Write1Byte(DEVICE_ADDR, CurrentStep.Value, DO_START, NO_STOP);
    }
    break;

    case CMD_WriteMultNS:
    {
      I2C0_Write1Byte(DEVICE_ADDR, CurrentStep.Value, NO_START, NO_STOP);
    }
    break;

    case CMD_Read8:
    {
      I2C0_Read1Byte(DEVICE_ADDR, DO_START, DO_STOP);
    }
    break;

    case CMD_Read8NS:
    {
      I2C0_Read1Byte(DEVICE_ADDR, NO_START, DO_STOP);
    }
    break;

    case CMD_ReadMult:
    {
      I2C0_Read1Byte(DEVICE_ADDR, DO_START, NO_STOP);
    }
    break;

    case CMD_ReadMultNS:
    {
      I2C0_Read1Byte(DEVICE_ADDR, NO_START, NO_STOP);
    }
    break;

    case CMD_GetResult:
    {
      // getting the result is immediate, so move to next step right away
      ES_Event_t ThisEvent;
      ThisEvent.EventType = EV_I2C_NextStep;
      PostI2CService( ThisEvent );
      // don't forget to grab the result :-)
      *(uint8_t *)CurrentStep.Result = (uint8_t)I2CMasterDataGet(I2C0_BASE);
    }
    break;

    case CMD_Form16:
    {
      // this operation is immediate, so move to next step right away
      ES_Event_t ThisEvent;
      ThisEvent.EventType = EV_I2C_NextStep;
      PostI2CService( ThisEvent );
      // don't forget to combine the  bytes
      *(uint16_t *)CurrentStep.Result = (((uint16_t)readTwo <<8) | readOne);
    }
    break;

    case CMD_NOP:
    {
      // and post a message to ourselves that the sequence is compelte
      ES_Event_t ThisEvent;
      ThisEvent.EventType = EV_I2C_Wait4Time;
      PostI2CService( ThisEvent );
    }
    break;

    case CMD_EOS:
    {
      // and post a message to ourselves that the sequence is compelte
      ES_Event_t ThisEvent;
      ThisEvent.EventType = EV_I2C_EOS;
      PostI2CService( ThisEvent );
      // and start a timer for the next read
      ES_Timer_InitTimer(I2C_TIMER, INT_TIME);
    }
    break;

    default:
    {
    }  
    break;
  }  // end of command interrpretation switch
  if (true == CurrentStep.BusyWait)
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType = EV_I2C_Wait4Busy;
    PostI2CService( ThisEvent );
  }
}
