#include <string.h>
//#include "os.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#
/**
 * \file active.c
 * \brief A Skeleton Implementation of an RTOS
 * 
 * \mainpage A Skeleton Implementation of a "Full-Served" RTOS Model
 * This is an example of how to implement context-switching based on a 
 * full-served model. That is, the RTOS is implemented by an independent
 * "kernel" task, which has its own stack and calls the appropriate kernel 
 * function on behalf of the user task.
 *
 * \author Dr. Mantis Cheng
 * \date 29 September 2006
 *
 * ChangeLog: Modified by Alexander M. Hoole, October 2006.
 *			  -Rectified errors and enabled context switching.
 *			  -LED Testing code added for development (remove later).
 *
 * \section Implementation Note
 * This example uses the ATMEL AT90USB1287 instruction set as an example
 * for implementing the context switching mechanism. 
 * This code is ready to be loaded onto an AT90USBKey.  Once loaded the 
 * RTOS scheduling code will alternate lighting of the GREEN LED light on
 * LED D2 and D5 whenever the corresponding PING and PONG tasks are running.
 * (See the file "cswitch.S" for details.)
 */

typedef void (*voidfuncptr) (void);      /* pointer to void f(void) */ 

#define WORKSPACE     256
#define MAXPROCESS   8
typedef unsigned int PID;        // always non-zero if it is valid
typedef unsigned int MUTEX;      // always non-zero if it is valid
typedef unsigned char PRIORITY;
typedef unsigned int EVENT;      // always non-zero if it is valid
typedef unsigned int TICK;
const unsigned int PT;
const unsigned int PPP[]; 
static uint8_t slot_task_finished = 0; // indicates if peridoic task has run



//typedef unsigned char PRIORITY;


/*===========
  * RTOS Internal
  *===========
  */

/**
  * This internal kernel function is the context switching mechanism.
  * It is done in a "funny" way in that it consists two halves: the top half
  * is called "Exit_Kernel()", and the bottom half is called "Enter_Kernel()".
  * When kernel calls this function, it starts the top half (i.e., exit). Right in
  * the middle, "Cp" is activated; as a result, Cp is running and the kernel is
  * suspended in the middle of this function. When Cp makes a system call,
  * it enters the kernel via the Enter_Kernel() software interrupt into
  * the middle of this function, where the kernel was suspended.
  * After executing the bottom half, the context of Cp is saved and the context
  * of the kernel is restore. Hence, when this function returns, kernel is active
  * again, but Cp is not running any more. 
  * (See file "switch.S" for details.)
  */
void Task_Terminate(void);
extern void CSwitch();
extern void Exit_Kernel();    /* this is the same as CSwitch() */

/* Prototype */
//void Task_Terminate(void);

/** 
  * This external function could be implemented in two ways:
  *  1) as an external function call, which is called by Kernel API call stubs;
  *  2) as an inline macro which maps the call into a "software interrupt";
  *       as for the AVR processor, we could use the external interrupt feature,
  *       i.e., INT0 pin.
  *  Note: Interrupts are assumed to be disabled upon calling Enter_Kernel().
  *     This is the case if it is implemented by software interrupt. However,
  *     as an external function call, it must be done explicitly. When Enter_Kernel()
  *     returns, then interrupts will be re-enabled by Enter_Kernel().
  */ 
extern void Enter_Kernel();

#define Disable_Interrupt()		asm volatile ("cli"::)
#define Enable_Interrupt()		asm volatile ("sei"::)
  

/**
  *  This is the set of states that a task can be in at any given time.
  */
typedef enum process_states 
{ 
   DEAD = 0, 
   READY, 
   RUNNING,
   BLOCKED,
   SUSPENDED,
   SUSPENDEDBLOCKED,
   SUSPENDEDREADY 
} PROCESS_STATES;

/**
  * This is the set of kernel requests, i.e., a request code for each system call.
  */
typedef enum kernel_request_type 
{
   NONE = 0,
   CREATE,
   NEXT,
   SLEEP,
   SUSPEND,
   GETARG,
   RESUME,
   YIELD,
   TERMINATE
} KERNEL_REQUEST_TYPE;



/**
  * Each task is represented by a process descriptor, which contains all
  * relevant information about this task. For convenience, we also store
  * the task's stack, i.e., its workspace, in here.
  */
typedef struct td_struct ProcessDescriptor;
//typedef struct td_struct sleepnode;


struct td_struct
{
   unsigned char *sp;   /* stack pointer into the "workSpace" */
   unsigned char workSpace[WORKSPACE]; 
   TICK ticks;
   PROCESS_STATES state;
   PRIORITY original; //priority of created function
   PRIORITY inherited; // Inherited for inversion problem
   int arg;
   int sleeping; //sleeping tasks will be labelled with 1
   voidfuncptr  code;   /* function to be executed as a task */
   KERNEL_REQUEST_TYPE request;
   PID p;
   ProcessDescriptor* next; //Next process holding this task
   int PID;
   int susp; //Boolean for indicating suspension
} ;


typedef struct{
	
	 ProcessDescriptor* head;
	 ProcessDescriptor* tail;
	
} queue_t;


typedef struct sleep_struct {
	
	ProcessDescriptor* task;
	struct sleep_struct* next;
	
} sleep_struct ;
/**
  * This table contains ALL process descriptors. It doesn't matter what
  * state a task is in.
  */
static  ProcessDescriptor Process[MAXPROCESS];

/**
  * The process descriptor of the currently RUNNING task.
  */
volatile static  ProcessDescriptor* Cp; 
static ProcessDescriptor  task_desc[MAXPROCESS + 1];
/** The special "idle task" at the end of the descriptors array. */
static ProcessDescriptor* idle_task = &task_desc[MAXPROCESS];


//Data from currently running task

static queue_t Dead_tasks; // terminated tasks

static queue_t Round_Robin;// round robin task queue

static queue_t system_tasks; // system task queue

static queue_t sleeping_tasks; // sleeping task queue

static queue_t event_queue; // tasks waiting on events

static volatile uint8_t ticks_remaining; // time remaining in current slot?



/** 
  * Since this is a "full-served" model, the kernel is executing using its own
  * stack. We can allocate a new workspace for this kernel stack, or we can
  * use the stack of the "main()" function, i.e., the initial C runtime stack.
  * (Note: This and the following stack pointers are used primarily by the
  *   context switching code, i.e., CSwitch(), which is written in assembly
  *   language.)
  */         
volatile unsigned char *KernelSp;

/**
  * This is a "shadow" copy of the stack pointer of "Cp", the currently
  * running task. During context switching, we need to save and restore
  * it into the appropriate process descriptor.
  */
volatile unsigned char *CurrentSp;

/** index to next task to run */
volatile static unsigned int NextP;  

/** 1 if kernel has been started; 0 otherwise. */
volatile static unsigned int KernelActive;  

/** number of tasks created so far */
volatile static unsigned int Tasks;  

/**
 * When creating a new task, it is important to initialize its stack just like
 * it has called "Enter_Kernel()"; so that when we switch to it later, we
 * can just restore its execution context on its stack.
 * (See file "cswitch.S" for details.)
 */

/*Chane to include Priority*/

void Kernel_Create_Task_At(  ProcessDescriptor *p, voidfuncptr f ) 
{   
   unsigned char *sp;



   //Changed -2 to -1 to fix off by one error.
   sp = (unsigned char *) &(p->workSpace[WORKSPACE-1]);



   /*----BEGIN of NEW CODE----*/
   //Initialize the workspace (i.e., stack) and PD here!

   //Clear the contents of the workspace
   memset(&(p->workSpace),0,WORKSPACE);

   //Notice that we are placing the address (16-bit) of the functions
   //onto the stack in reverse byte order (least significant first, followed
   //by most significant).  This is because the "return" assembly instructions 
   //(rtn and rti) pop addresses off in BIG ENDIAN (most sig. first, least sig. 
   //second), even though the AT90 is LITTLE ENDIAN machine.

   //Store terminate at the bottom of stack to protect against stack underrun.
   *(unsigned char *)sp-- = ((unsigned int)Task_Terminate) & 0xff;
   *(unsigned char *)sp-- = (((unsigned int)Task_Terminate) >> 8) & 0xff;
   *(unsigned char *)sp-- = 0x00;

   //Place return address of function at bottom of stack
   *(unsigned char *)sp-- = ((unsigned int)f) & 0xff;
   *(unsigned char *)sp-- = (((unsigned int)f) >> 8) & 0xff;
   *(unsigned char *)sp-- = 0x00;


   //Place stack pointer at top of stack
   sp = sp - 34;

      
   p->sp = sp;		/* stack pointer into the "workSpace" */
   p->code = f;		/* function to be executed as a task */
   p->request = NONE;

   /*----END of NEW CODE----*/

   p->state = READY;

}


/**
  *  Create a new task
  */
//This is probably good for Task_Create()
static void Kernel_Create_Task( voidfuncptr f ) 
{
   int x;

   if (Tasks == MAXPROCESS) return;  /* Too many tasks! */

   /* find a DEAD PD that we can use  */
   for (x = 0; x < MAXPROCESS; x++) {
       if (Process[x].state == DEAD) break;
   }

   ++Tasks;
   Kernel_Create_Task_At( &(Process[x]), f );

}

static void enqueue(queue_t* input_queue, ProcessDescriptor* input_process){
	
	input_process->next = NULL;
	
	
	if(input_queue->head == NULL){
		
		input_queue->head = input_process;
		input_queue->tail = input_process;
	}
	else{
		
		input_queue->tail->next = input_process;
		input_queue->tail = input_process;
		
	}
	
	
}
static void enqueue_sleep(queue_t* input_queue, sleep_struct* input_sleepnode){
	
	sleep_struct* tmp = NULL;
	sleep_struct* tmp2 = NULL;
	TICK before;
	TICK after;
	
	if(input_queue->head == NULL){
		
		input_queue->head = input_sleepnode->task;
		input_queue->tail = input_sleepnode->task;
		PORTL |= (1<< DDL1);
	}
	else{
		
		before = input_queue->head->ticks;
		
		after = input_queue->head->next->ticks;
	
		
		if (input_sleepnode->task->ticks > before)
		{
			
			tmp = input_queue->head;
			input_queue->head = input_sleepnode;
			input_sleepnode->next = tmp;
			
		}
		else{
			for(tmp = input_queue->head;tmp->next != NULL; tmp=tmp->next)
			{
				
				before = after;
				after = tmp->task->next->ticks;
			}
				
				if (input_sleepnode->task->ticks >= before && input_sleepnode->task->ticks  <= after)
				
				{
					tmp2 = tmp;
					
					tmp->next = input_sleepnode;
					input_sleepnode->next = tmp2;
					
					
				}
				else if(tmp->next == NULL && tmp->task->ticks <= input_sleepnode->task->ticks){
					
					input_queue->tail->next = input_sleepnode;
					input_queue->tail = input_sleepnode;
					
				}
				
				
				
			}
		}
		
	
		
	}
	
	



static ProcessDescriptor* dequeue(queue_t* input_queue){
	
	ProcessDescriptor* processpointer = input_queue->head;
	
	if(input_queue->head != NULL)
	{
		input_queue->head = input_queue->head->next;
		processpointer->next = NULL;
		
	}
	
	return processpointer;
}

static void new_dispatch()
{
	if (Cp->state != RUNNING || Cp == idle_task ){
		
		if (system_tasks.head != NULL)
		{
			Cp = dequeue(&system_tasks);
		}
		else if (!slot_task_finished && PT > 0 ) // There is some more to add here, but I don't know what they're doing
		{
			// Current task equals something from the PPP array
			
		}
		
		else if(Round_Robin.head != NULL){
			
			Cp = dequeue(&Round_Robin);
		}
		else{
		
			Cp = idle_task;
		}
		
	}
	Cp->state = RUNNING;
	
}


/**
  * This internal kernel function is a part of the "scheduler". It chooses the 
  * next task to run, i.e., Cp.
  */


static void Dispatch()
{
     /* find the next READY task
       * Note: if there is no READY task, then this will loop forever!.
       */
   while(Process[NextP].state != READY) {
      NextP = (NextP + 1) % MAXPROCESS;
   }

   Cp = &(Process[NextP]);
   CurrentSp = Cp->sp;
   Cp->state = RUNNING;

   NextP = (NextP + 1) % MAXPROCESS;
}

/**
  * This internal kernel function is the "main" driving loop of this full-served
  * model architecture. Basically, on OS_Start(), the kernel repeatedly
  * requests the next user task's next system call and then invokes the
  * corresponding kernel function on its behalf.
  *
  * This is the main loop of our kernel, called by OS_Start().
  */
static void Next_Kernel_Request() 
{
   Dispatch();  /* select a new task to run */

   while(1) {
       Cp->request = NONE; /* clear its request */

       /* activate this newly selected task */
       CurrentSp = Cp->sp;
       Exit_Kernel();    /* or CSwitch() */

       /* if this task makes a system call, it will return to here! */

        /* save the Cp's stack pointer */
       Cp->sp = CurrentSp;

       switch(Cp->request){
       case CREATE:
           Kernel_Create_Task( Cp->code );
           break;
       case NEXT:
	     case NONE:
           /* NONE could be caused by a timer interrupt */
          Cp->state = READY;
          Dispatch();
          break;
		   case SUSPEND:
    			Cp->susp = 1;
    			if (Cp->state == RUNNING){
    				Cp->state = READY;
    			}
        case RESUME:
          Cp->susp = 0;
       case TERMINATE:
          /* deallocate all resources used by this task */
          Cp->state = DEAD;
          Dispatch();
          break;
       default:
          /* Houston! we have a problem here! */
          break;
       }
    } 
}


/*================
  * RTOS  API  and Stubs
  *================
  */

/**
  * This function initializes the RTOS and must be called before any other
  * system calls.
  */
void OS_Init() 
{
   int x;
	DDRA = (1<<PA0);
	DDRB = (1<<DDB7);
	DDRL |= (1<<DDL1);// pin 48 test
	
	DDRA = (1<<PA1);
	PORTA &= ~(1<<PA1);
   Tasks = 0;
   KernelActive = 0;
   NextP = 0;
	//Reminder: Clear the memory for the task on creation.
   for (x = 0; x < MAXPROCESS; x++) {
      memset(&(Process[x]),0,sizeof(ProcessDescriptor));
      Process[x].state = DEAD;
   }
}


/**
  * This function starts the RTOS after creating a few tasks.
  */
void OS_Start() 
{   
   if ( (! KernelActive) && (Tasks > 0)) {
       Disable_Interrupt();
      /* we may have to initialize the interrupt vector for Enter_Kernel() here. */

      /* here we go...  */
      KernelActive = 1;
      Next_Kernel_Request();
      /* NEVER RETURNS!!! */
   }
}


/**
  * For this example, we only support cooperatively multitasking, i.e.,
  * each task gives up its share of the processor voluntarily by calling
  * Task_Next().
  */
void Task_Create( voidfuncptr f, PRIORITY py, int arg)
{
   if (KernelActive ) {
     Disable_Interrupt();
     Cp ->request = CREATE;
     Cp->code = f;
     Enter_Kernel();
   } else { 
      /* call the RTOS function directly */
      Kernel_Create_Task( f );
   }
}

/**
  * The calling task gives up its share of the processor voluntarily.
  */
void Task_Next() 
{
   if (KernelActive) {
     Disable_Interrupt();
     Cp ->request = NEXT;
     Enter_Kernel();
  }
}

/*
ISR(TIMER0_COMPA_vect){
	Disable_Interrupt();
	
	Task_Next();
	Enable_Interrupt();
}*/


/**
  * The calling task terminates itself.
  */
void Task_Terminate() 
{
   if (KernelActive) {
      Disable_Interrupt();
      Cp -> request = TERMINATE;
      Enter_Kernel();
     /* never returns here! */
   }
}

void Task_Suspend(PID p)
{
  //Suspend Task
  if(Cp->PID == p && KernelActive){
    Disable_Interrupt();
    Cp->request = SUSPEND;
    Enter_Kernel();
    Enable_Interrupt();
  }
}

ProcessDescriptor* get_Task(PID p){
  int x;
  ProcessDescriptor* pd = NULL;
  for (x=0;x<MAXPROCESS;x++){
    if (Process[x].PID == p){
      pd = &Process[x];
      break;
    }
  }
  return pd;
}

void Task_Resume(PID p){
	ProcessDescriptor* pd;
	
	//If pd is not NULL (found in list)
	if ((pd = get_Task(p))){
		pd->request = RESUME;
	}
}

int Task_Get_Arg(PID p){
	ProcessDescriptor* pd;
	int arg = NULL;
	//Get Argument
	if ((pd = get_Task(p))){
		arg = pd->arg;
	}
	return arg;
}

void Task_Yield(){
	//Yield to Priority Tasks
}

void Task_Sleep(TICK t){
	//Put the task to sleep for a certain amount of time
	//Eventually to take a clock tick parameter
	struct sleep_struct newnode;
	Cp->ticks += t;
	newnode.task = Cp;
	enqueue_sleep(&sleeping_tasks, &newnode);
	Task_Suspend(Cp->PID);
	//current process can only call sleep on itself
	//Iterate through sleep queue and place in timer fashion	
}

void TIMER3_COMPA_vect(void){
  struct sleep_struct* temp_node;
  temp_node->task = sleeping_tasks.head;
  
  if (temp_node->task != NULL){
    while (temp_node != NULL){
      temp_node->task->ticks -= 1;
      
      if (temp_node->task->ticks == 0){
        Disable_Interrupt();
        temp_node->task->request = RESUME;
        Enter_Kernel();
        Enable_Interrupt();
        break;
      }
      temp_node = temp_node->next;
    }
  }
}


/*============
  * A Simple Test 
  *============
  */

/**
  * A cooperative "Ping" task.
  * Added testing code for LEDs.
  */
void Ping()
{
	//Enable_Interrupt();
	int  x ;
	for(;;){
		//LED on
		//PORTA |= (1<<PA0);
		PORTA &= ~(1<<PA0);
		PORTB &= ~(1<<DDB7);
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		//Task_Sleep(50);
		Task_Next();
	}
}


/**
  * A cooperative "Pong" task.
  * Added testing code for LEDs.
  */
void Pong()
{
	//Enable_Interrupt();
	int  x;
	for(;;) {
		//LED off
		//PORTA &= ~(1<<PA0);
		PORTA |= (1<<PA0);
		PORTB |= (1<<DDB7);
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		for( x=0; x < 32000; ++x );   /* do nothing */
		Task_Next();
	}
}


/**
  * This function creates two cooperative tasks, "Ping" and "Pong". Both
  * will run forever.
  */
void main() 
{
	
   OS_Init();
   Task_Create( Ping, 8, 8 );
   
   Task_Create( Pong, 8, 8 );
   
  
   OS_Start();
}

