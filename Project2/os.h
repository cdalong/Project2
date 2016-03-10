#ifndef _OS_H_  
#define _OS_H_  
   
#define MAXTHREAD     16       
#define WORKSPACE     256   // in bytes, per THREAD
#define MAXMUTEX      8 
#define MAXEVENT      8      
#define MSECPERTICK   10   // resolution of a system tick in milliseconds
#define MINPRIORITY   10   // 0 is the highest priority, 10 the lowest


#ifndef NULL
#define NULL          0   /* undefined */
#endif

typedef unsigned int PID;        // always non-zero if it is valid
typedef unsigned int MUTEX;      // always non-zero if it is valid
typedef unsigned char PRIORITY;
typedef unsigned int EVENT;      // always non-zero if it is valid
typedef unsigned int TICK;

// void OS_Init(void);      redefined as main()
void OS_Abort(void);

//void  Task_Create( void (*f)(void), PRIORITY py, int arg);
void Task_Terminate(void);
void Task_Yield(void);
int  Task_GetArg(void);
void Task_Suspend( PID p );          
void Task_Resume( PID p );

void Task_Sleep(TICK t);  // sleep time is at least t*MSECPERTICK

MUTEX Mutex_Init(void);
void Mutex_Lock(MUTEX m);
void Mutex_Unlock(MUTEX m);

EVENT Event_Init(void);
void Event_Wait(EVENT e);
void Event_Signal(EVENT e);

#endif /* _OS_H_ */
