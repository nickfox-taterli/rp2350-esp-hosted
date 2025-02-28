/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

#if !NO_SYS

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#if defined(LWIP_PROVIDE_ERRNO)
int errno;
#endif

/*-----------------------------------------------------------------------------------*/
// 创建一个空的邮箱
err_t sys_mbox_new(sys_mbox_t* mbox, int size)
{
    *mbox = xQueueCreate(size, sizeof(void *)); // 使用FreeRTOS的队列作为邮箱
    if (*mbox == NULL)
    {
#if SYS_STATS
    ++lwip_stats.sys.mbox.err;
#endif /* SYS_STATS */
        return ERR_MEM;
    }

#if SYS_STATS
  ++lwip_stats.sys.mbox.used;
  if (lwip_stats.sys.mbox.max < lwip_stats.sys.mbox.used)
  {
    lwip_stats.sys.mbox.max = lwip_stats.sys.mbox.used;
  }
#endif /* SYS_STATS */

    return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/*
  释放邮箱。如果邮箱在释放时仍有消息存在，表明lwIP中存在编程错误，开发者应被通知。
*/
void sys_mbox_free(sys_mbox_t* mbox)
{
    if (uxQueueMessagesWaiting(*mbox))
    {
        /* 断点行。永远不应该在这里中断！ */
        portNOP();
#if SYS_STATS
    lwip_stats.sys.mbox.err++;
#endif /* SYS_STATS */
    }
    vQueueDelete(*mbox); // 删除队列
#if SYS_STATS
  --lwip_stats.sys.mbox.used;
#endif /* SYS_STATS */
}

/*-----------------------------------------------------------------------------------*/
// 将消息投递到邮箱
void sys_mbox_post(sys_mbox_t* mbox, void* data)
{
    while (xQueueSend(*mbox, &data, portMAX_DELAY) != pdPASS); // 阻塞直到消息成功发送
}

/*-----------------------------------------------------------------------------------*/
// 尝试将消息投递到邮箱
err_t sys_mbox_trypost(sys_mbox_t* mbox, void* msg)
{
    if (xQueueSend(*mbox, &msg, 0) == pdPASS) // 非阻塞发送
    {
        return ERR_OK;
    }
    else
    {
        // 无法投递，队列可能已满
#if SYS_STATS
    lwip_stats.sys.mbox.err++;
#endif /* SYS_STATS */
        return ERR_MEM;
    }
}

/*-----------------------------------------------------------------------------------*/
// 从中断服务例程中尝试将消息投递到邮箱
err_t sys_mbox_trypost_fromisr(sys_mbox_t* mbox, void* msg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(*mbox, &msg, &xHigherPriorityTaskWoken) == pdPASS)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return ERR_OK;
    }
    else
    {
        // 无法投递，队列可能已满
#if SYS_STATS
    lwip_stats.sys.mbox.err++;
#endif /* SYS_STATS */
        return ERR_MEM;
    }
}

/*-----------------------------------------------------------------------------------*/
/*
  阻塞线程直到邮箱中有消息到达，但不会阻塞超过"timeout"毫秒（类似于sys_arch_sem_wait()函数）。
  "msg"参数是一个结果参数，由函数设置（即通过"*msg = ptr"）。如果"msg"参数为NULL，表示消息应被丢弃。

  返回值与sys_arch_sem_wait()函数相同：
  等待的毫秒数，如果超时则返回SYS_ARCH_TIMEOUT。

  注意：lwIP实现了一个类似名称的函数sys_mbox_fetch()。
*/
u32_t sys_arch_mbox_fetch(sys_mbox_t* mbox, void** msg, u32_t timeout)
{
    TickType_t starttime = xTaskGetTickCount();
    if (timeout != 0)
    {
        if (xQueueReceive(*mbox, msg, pdMS_TO_TICKS(timeout)) == pdPASS)
        {
            return (xTaskGetTickCount() - starttime);
        }
        else
        {
            return SYS_ARCH_TIMEOUT;
        }
    }
    else
    {
        while (xQueueReceive(*mbox, msg, portMAX_DELAY) != pdPASS);
        return (xTaskGetTickCount() - starttime);
    }
}

/*-----------------------------------------------------------------------------------*/
/*
  类似于sys_arch_mbox_fetch，但如果消息没有立即准备好，我们将返回SYS_MBOX_EMPTY。
  成功时返回0。
*/
u32_t sys_arch_mbox_tryfetch(sys_mbox_t* mbox, void** msg)
{
    if (xQueueReceive(*mbox, msg, 0) == pdPASS)
    {
        return ERR_OK;
    }
    else
    {
        return SYS_MBOX_EMPTY;
    }
}

/*-----------------------------------------------------------------------------------*/
int sys_mbox_valid(sys_mbox_t* mbox)
{
    if (*mbox == NULL)
        return 0;
    else
        return 1;
}

/*-----------------------------------------------------------------------------------*/
void sys_mbox_set_invalid(sys_mbox_t* mbox)
{
    *mbox = NULL;
}

/*-----------------------------------------------------------------------------------*/
// 创建一个新的信号量。"count"参数指定信号量的初始状态。
err_t sys_sem_new(sys_sem_t* sem, u8_t count)
{
    *sem = xSemaphoreCreateCounting(UINT16_MAX, count); // 创建计数信号量
    if (*sem == NULL)
    {
#if SYS_STATS
    ++lwip_stats.sys.sem.err;
#endif /* SYS_STATS */
        return ERR_MEM;
    }

    if (count == 0) // 表示信号量不能被获取
    {
        xSemaphoreTake(*sem, 0);
    }

#if SYS_STATS
  ++lwip_stats.sys.sem.used;
  if (lwip_stats.sys.sem.max < lwip_stats.sys.sem.used)
  {
    lwip_stats.sys.sem.max = lwip_stats.sys.sem.used;
  }
#endif /* SYS_STATS */

    return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/*
  阻塞线程等待信号量被触发。如果"timeout"参数非零，线程应仅在指定时间内阻塞（以毫秒为单位）。

  如果"timeout"参数非零，返回值是等待信号量被触发的毫秒数。如果信号量在指定时间内未被触发，返回值为SYS_ARCH_TIMEOUT。
  如果线程不需要等待信号量（即信号量已被触发），函数可能返回零。

  注意：lwIP实现了一个类似名称的函数sys_sem_wait()，它使用sys_arch_sem_wait()函数。
*/
u32_t sys_arch_sem_wait(sys_sem_t* sem, u32_t timeout)
{
    TickType_t starttime = xTaskGetTickCount();
    if (timeout != 0)
    {
        if (xSemaphoreTake(*sem, pdMS_TO_TICKS(timeout)) == pdPASS)
        {
            return (xTaskGetTickCount() - starttime);
        }
        else
        {
            return SYS_ARCH_TIMEOUT;
        }
    }
    else
    {
        while (xSemaphoreTake(*sem, portMAX_DELAY) != pdPASS);
        return (xTaskGetTickCount() - starttime);
    }
}

/*-----------------------------------------------------------------------------------*/
// 触发信号量
void sys_sem_signal(sys_sem_t* sem)
{
    xSemaphoreGive(*sem);
}

/*-----------------------------------------------------------------------------------*/
// 释放信号量
void sys_sem_free(sys_sem_t* sem)
{
#if SYS_STATS
  --lwip_stats.sys.sem.used;
#endif /* SYS_STATS */
    vSemaphoreDelete(*sem);
}

/*-----------------------------------------------------------------------------------*/
int sys_sem_valid(sys_sem_t* sem)
{
    if (*sem == NULL)
        return 0;
    else
        return 1;
}

/*-----------------------------------------------------------------------------------*/
void sys_sem_set_invalid(sys_sem_t* sem)
{
    *sem = NULL;
}

/*-----------------------------------------------------------------------------------*/
SemaphoreHandle_t lwip_sys_mutex;

// 初始化系统架构
void sys_init(void)
{
    lwip_sys_mutex = xSemaphoreCreateMutex(); // 创建互斥锁
}

/*-----------------------------------------------------------------------------------*/
/* 互斥锁 */
/*-----------------------------------------------------------------------------------*/
#if LWIP_COMPAT_MUTEX == 0
/* 创建一个新的互斥锁 */
err_t sys_mutex_new(sys_mutex_t* mutex)
{
    *mutex = xSemaphoreCreateMutex(); // 创建互斥锁
    if (*mutex == NULL)
    {
#if SYS_STATS
    ++lwip_stats.sys.mutex.err;
#endif /* SYS_STATS */
        return ERR_MEM;
    }

#if SYS_STATS
  ++lwip_stats.sys.mutex.used;
  if (lwip_stats.sys.mutex.max < lwip_stats.sys.mutex.used)
  {
    lwip_stats.sys.mutex.max = lwip_stats.sys.mutex.used;
  }
#endif /* SYS_STATS */
    return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/* 释放互斥锁 */
void sys_mutex_free(sys_mutex_t* mutex)
{
#if SYS_STATS
  --lwip_stats.sys.mutex.used;
#endif /* SYS_STATS */
    vSemaphoreDelete(*mutex);
}

/*-----------------------------------------------------------------------------------*/
/* 锁定互斥锁 */
void sys_mutex_lock(sys_mutex_t* mutex)
{
    xSemaphoreTake(*mutex, portMAX_DELAY);
}

/*-----------------------------------------------------------------------------------*/
/* 解锁互斥锁 */
void sys_mutex_unlock(sys_mutex_t* mutex)
{
    xSemaphoreGive(*mutex);
}
#endif /*LWIP_COMPAT_MUTEX*/

/*
  启动一个优先级为"prio"的新线程，该线程将从"thread()"函数开始执行。"arg"参数将作为参数传递给thread()函数。
  返回新线程的ID。ID和优先级都是系统相关的。
*/
sys_thread_t sys_thread_new(const char* name, lwip_thread_fn thread, void* arg, int stacksize, int prio)
{
    sys_thread_t task;
    xTaskCreate(thread, name, stacksize, arg, prio, &task); // 创建新任务
}

/*
  这个可选函数执行“快速”临界区保护并返回先前的保护级别。此函数仅在非常短的临界区中调用。
  支持基于ISR的驱动程序的嵌入式系统可能希望通过禁用中断来实现此功能。基于任务的系统可能希望通过使用互斥锁或禁用任务来实现此功能。
  此函数应支持从同一任务或中断的递归调用。换句话说，sys_arch_protect()可以在已经保护的情况下调用。在这种情况下，返回值表示它已经被保护。

  sys_arch_protect()仅在您的端口支持操作系统时才需要。
*/
sys_prot_t sys_arch_protect(void)
{
    taskENTER_CRITICAL();
    return (sys_prot_t)1;
}

/*
  这个可选函数将临界区保护设置为pval指定的值。有关更多信息，请参阅sys_arch_protect()的文档。
  此函数仅在您的端口支持操作系统时才需要。
*/
void sys_arch_unprotect(sys_prot_t pval)
{
    (void)pval;
    taskEXIT_CRITICAL();
}

u32_t sys_now(void)
{
    return xTaskGetTickCount();
}

#endif /* !NO_SYS */
