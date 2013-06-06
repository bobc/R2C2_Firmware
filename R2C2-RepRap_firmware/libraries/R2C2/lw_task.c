/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
// **************************************************************************
// Description:
//
// **************************************************************************

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "lw_rtos.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define MAX_TASKS       8

//
#define TASK_NAME_LEN   6

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef enum {
    TS_DEAD,    // marks free slots in task table
    TS_INIT,    // 
    TS_RUNNING 
    } tTaskState;

typedef struct {
    tTaskState  State;
    fpTaskInit  fInit;
    fpTaskPoll  fPoll;
    void        *pTaskParameters;
    uint16_t    StackSize;
    uint16_t    Priority;
    // char Name [TASK_NAME_LEN];
    //
    uint16_t    WakeupCounter;
} tTaskDef;

typedef tTaskDef *priv_tTaskHandle;

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

static tTaskDef Tasks[MAX_TASKS];
static uint16_t NumTasks; 

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief  Create a task
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------

LW_RTOS_RESULT lw_TaskCreate (  fpTaskPoll    TaskPoll,
                                fpTaskInit    TaskInit,
                              const char * const pName,
                              uint16_t       StackSize,
                              void           *pvParameters,
                              uint16_t       uPriority,
                              tTaskHandle    *pTaskId)

{
    if (NumTasks < MAX_TASKS)
    {
        //TODO look for empty task slots
        priv_tTaskHandle pCurTask = &Tasks[NumTasks];
        NumTasks++;
        
        pCurTask->fInit = TaskInit;
        pCurTask->fPoll = TaskPoll;
        pCurTask->pTaskParameters = pvParameters;
        pCurTask->Priority = uPriority;
        pCurTask->StackSize = StackSize;

        // strncpy (pCurTask->Name, pName, sizeof(pCurTask->Name));      
        pCurTask->State = TS_INIT;
        
        if (pTaskId != NULL)
            *pTaskId = pCurTask;
        
        return LWR_OK;
    }
    else
        return LWR_ERROR;
}

// --------------------------------------------------------------------------
//! @brief  Delete a task
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------

LW_RTOS_RESULT lw_TaskDelete (tTaskHandle TaskId)
{
    priv_tTaskHandle pTask = TaskId; 

    if (pTask != NULL)
    {
        pTask->State = TS_DEAD;
        NumTasks --;
        
        return LWR_OK;
    }
    else
        return LWR_ERROR;
}

// --------------------------------------------------------------------------
//! @brief  Run the task scheduler.
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------

LW_RTOS_RESULT lw_TaskScheduler (void)
{
    uint16_t            taskIndex;
    priv_tTaskHandle     pCurTask;
    
    
    taskIndex = 0;
    
    for (;;)
    {
        pCurTask = &Tasks[taskIndex];
    
        switch (pCurTask->State)
        {
        case TS_DEAD: break;
        
        case TS_INIT:
            pCurTask->fInit (pCurTask->pTaskParameters);
            //TODO check result?
            pCurTask->State = TS_RUNNING;
            pCurTask->WakeupCounter = pCurTask->Priority;
            break;

        case TS_RUNNING:
        
            if (pCurTask->WakeupCounter == 0)
            {
                pCurTask->fPoll (pCurTask->pTaskParameters);
                pCurTask->WakeupCounter = pCurTask->Priority;
            }
            else
                pCurTask->WakeupCounter--;
            break;    
        }
        
        taskIndex++;
        if (taskIndex == MAX_TASKS) // ***
            taskIndex = 0;                
    }
    
    return LWR_ERROR;
}
