#define  OS_CPU_GLOBALS
#include "config.h"


#if OS_SELF_EN == 0 
int const _OSFunctionAddr[1] = {0};
int const _UsrFunctionAddr[1] = {0};
#endif

/*********************************************************************************************************
** ��������: OSTaskStkInit
** ��������: �����ջ��ʼ�����룬����������ʧ�ܻ�ʹϵͳ����
** �䡡��: task  : ����ʼִ�еĵ�ַ
**         pdata �����ݸ�����Ĳ���
**         ptos  ������Ķ�ջ��ʼλ��
**         opt   �����Ӳ�������ǰ�汾���ڱ��������ã���������μ�OSTaskCreateExt()��opt����
** �䡡��: ջ��ָ��λ��
** ȫ�ֱ���:
** ����ģ��: 
**
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

OS_STK *OSTaskStkInit (void (*task)(void *pd), void *pdata, OS_STK *ptos, INT16U opt)
{
    OS_STK *stk;

    opt    = opt;                           /* 'opt'  û��ʹ�á������Ǳ������������    */
    stk    = ptos;                          /* ��ȡ��ջָ��                                       */

                                            /* �������񻷾���ADS1.2ʹ�����ݼ���ջ       */
    *stk = (OS_STK) task;                   /*  pc  */
    *--stk = (OS_STK) task;                 /*  lr  */

    *--stk = 0;                             /*  r12  */
    *--stk = 0;                             /*  r11  */
    *--stk = 0;                             /*  r10  */
    *--stk = 0;                             /*  r9   */
    *--stk = 0;                             /*  r8   */
    *--stk = 0;                             /*  r7   */
    *--stk = 0;                             /*  r6   */
    *--stk = 0;                             /*  r5   */
    *--stk = 0;                             /*  r4   */
    *--stk = 0;                             /*  r3   */
    *--stk = 0;                             /*  r2   */
    *--stk = 0;                             /*  r1   */
    *--stk = (unsigned int) pdata;          /*  r0,��һ������ʹ��R0����   */
    *--stk = (USER_USING_MODE|0x00);	    /*  spsr������ IRQ, FIQ �ж�   */
    *--stk = 0;                             /*  ���жϼ�����OsEnterSum;    */

    return (stk);
}


/*********************************************************************************************************
** ��������: SWI_Exception
** ��������: ���ж��쳣��������ṩһЩϵͳ����
**           
** �䡡��:  SWI_Num:���ܺ�
**          Regs[0] Ϊ��һ��������Ҳ�Ƿ���ֵ
**          Regs[1] Ϊ�ڶ�������
**          Regs[2] Ϊ����������
**          Regs[3] Ϊ���ĸ�����
** �䡡��: ���ݹ��ܶ���
**         
** ȫ�ֱ���: ��
** ����ģ��: ��
**
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#if OS_SELF_EN > 0 
extern int const _OSFunctionAddr[];
extern int const _UsrFunctionAddr[];
#endif

/*********************************************************************************************************
** ��������: OSStartHighRdy
** ��������: uC/OS-II����ʱʹ��OSStartHighRdy���е�һ������,
**           ʵ���ǲ���swi 1ָ��
** �䡡��:   ��
** �䡡�� :  ��
** ȫ�ֱ���: ��
** ����ģ��: ��
** 
**-------------------------------------------------------------------------------------------------------
********************************************************************************************************/

        void OSStartHighRdy(void)
{
    _OSStartHighRdy();
}

/*********************************************************************************************************
** ��������: _TaskIsARM
** ��������: ���������ARM����
**
** �䡡��:   ��
** �䡡�� :  ��
** ȫ�ֱ���: ��
** ����ģ��: ��
** 
** ������: ������
** �ա���: 2004��8��27��
**-------------------------------------------------------------------------------------------------------
** �ޡ���: 
** �ա���: 
**-------------------------------------------------------------------------------------------------------
********************************************************************************************************/
        void _TaskIsARM(INT8U prio)
{
    OS_TCB   *ptcb;

    if (prio <= OS_LOWEST_PRIO)
    {
        ptcb = OSTCBPrioTbl[prio];
        if (ptcb != NULL)
        {
            ptcb -> OSTCBStkPtr[1] &= ~(1 << 5);
        }
    }
}
/*********************************************************************************************************
** ��������: _TaskIsARM
** ��������: ������THUMB����
**
** �䡡��:   ��
** �䡡�� :  ��
** ȫ�ֱ���: ��
** ����ģ��: ��
** 
** ������: ������
** �ա���: 2004��8��27��
**-------------------------------------------------------------------------------------------------------
** �ޡ���: 
** �ա���: 
**-------------------------------------------------------------------------------------------------------
********************************************************************************************************/
        void _TaskIsTHUMB(INT8U prio)
{
    OS_TCB   *ptcb;

    if (prio <= OS_LOWEST_PRIO)
    {
        ptcb = OSTCBPrioTbl[prio];
        if (ptcb != NULL)
        {
            ptcb -> OSTCBStkPtr[1] |= (1 << 5);
        }
    }
}

/* ����ΪһЩ���Ӻ�����ȫ��Ϊ�պ���������˵���뿴������� */

#if OS_CPU_HOOKS_EN
/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                            (BEGINNING)
*
* Description: This function is called by OSInit() at the beginning of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
#if OS_VERSION > 203
void OSInitHookBegin (void)
{
}
#endif

/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                               (END)
*
* Description: This function is called by OSInit() at the end of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
#if OS_VERSION > 203
void OSInitHookEnd (void)
{
}
#endif


/*
*********************************************************************************************************
*                                          TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
void OSTaskCreateHook (OS_TCB *ptcb)
{
    ptcb = ptcb;                       /* Prevent compiler warning                                     */
}


/*
*********************************************************************************************************
*                                           TASK DELETION HOOK
*
* Description: This function is called when a task is deleted.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
void OSTaskDelHook (OS_TCB *ptcb)
{
    ptcb = ptcb;                       /* Prevent compiler warning                                     */
}

/*
*********************************************************************************************************
*                                           TASK SWITCH HOOK
*
* Description: This function is called when a task switch is performed.  This allows you to perform other
*              operations during a context switch.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are disabled during this call.
*              2) It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                 will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the 
*                 task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/
void OSTaskSwHook (void)
{
}

/*
*********************************************************************************************************
*                                           STATISTIC TASK HOOK
*
* Description: This function is called every second by uC/OS-II's statistics task.  This allows your 
*              application to add functionality to the statistics task.
*
* Arguments  : none
*********************************************************************************************************
*/
void OSTaskStatHook (void)
{
}

/*
*********************************************************************************************************
*                                           OSTCBInit() HOOK
*
* Description: This function is called by OSTCBInit() after setting up most of the TCB.
*
* Arguments  : ptcb    is a pointer to the TCB of the task being created.
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
#if OS_VERSION > 203
void OSTCBInitHook (OS_TCB *ptcb)
{
    ptcb = ptcb;                                           /* Prevent Compiler warning                 */
}
#endif


/*
*********************************************************************************************************
*                                               TICK HOOK
*
* Description: This function is called every tick.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
void OSTimeTickHook (void)
{
}


/*
*********************************************************************************************************
*                                             IDLE TASK HOOK
*
* Description: This function is called by the idle task.  This hook has been added to allow you to do  
*              such things as STOP the CPU to conserve power.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are enabled during this call.
*********************************************************************************************************
*/
#if OS_VERSION >= 251
void OSTaskIdleHook (void)
{
}
#endif
#endif
/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/

