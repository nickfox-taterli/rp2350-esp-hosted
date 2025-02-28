// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "mempool.h"
#include "esp_wrapper.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static char * MEM_TAG = "mpool";

struct mempool * mempool_create(uint32_t block_size)
{
#ifdef H_USE_MEMPOOL
    // 使用 FreeRTOS 的 pvPortMalloc 替代 g_h.funcs->_h_malloc
    struct mempool * new = (struct mempool *)pvPortMalloc(MEMPOOL_ALIGNED(sizeof(struct mempool)));

    if (!new) {
        ESP_LOGE(MEM_TAG, "Prob to create mempool size(%u)", MEMPOOL_ALIGNED(sizeof(struct mempool)));
        return NULL;
    }

    if (!IS_MEMPOOL_ALIGNED((long)new)) {
        ESP_LOGV(MEM_TAG, "Nonaligned");
        vPortFree(new);  // 使用 FreeRTOS 的 vPortFree 替代 g_h.funcs->_h_free
        new = (struct mempool *)pvPortMalloc(MEMPOOL_ALIGNED(sizeof(struct mempool)));
    }

    if (!new) {
        ESP_LOGE(MEM_TAG, "failed to create mempool size(%u)", MEMPOOL_ALIGNED(sizeof(struct mempool)));
        return NULL;
    }

    // 使用 FreeRTOS 的互斥锁替代 g_h.funcs->_h_create_lock_mempool
    new->spinlock = xSemaphoreCreateMutex();

    new->block_size = MEMPOOL_ALIGNED(block_size);
    SLIST_INIT(&(new->head));

    ESP_LOGV(MEM_TAG, "Create mempool %p with block_size:%lu", new, (unsigned long int)block_size);
    return new;
#else
    return NULL;
#endif
}

void mempool_destroy(struct mempool* mp)
{
#ifdef H_USE_MEMPOOL
    void * node1 = NULL;

    if (!mp)
        return;

    ESP_LOGV(MEM_TAG, "Destroy mempool %p", mp);

    while ((node1 = SLIST_FIRST(&(mp->head))) != NULL) {
        SLIST_REMOVE_HEAD(&(mp->head), entries);
        vPortFree(node1);  // 使用 FreeRTOS 的 vPortFree 替代 g_h.funcs->_h_free
    }
    SLIST_INIT(&(mp->head));

    vPortFree(mp);  // 使用 FreeRTOS 的 vPortFree 替代 g_h.funcs->_h_free
#endif
}

void * mempool_alloc(struct mempool* mp, int nbytes, int need_memset)
{
    void *buf = NULL;

#ifdef H_USE_MEMPOOL
    if (!mp || mp->block_size < nbytes)
        return NULL;

    // 使用 FreeRTOS 的 xSemaphoreTake 替代 g_h.funcs->_h_lock_mempool
    xSemaphoreTake(mp->spinlock, portMAX_DELAY);

    if (!SLIST_EMPTY(&(mp->head))) {
        buf = SLIST_FIRST(&(mp->head));
        SLIST_REMOVE_HEAD(&(mp->head), entries);

        // 使用 FreeRTOS 的 xSemaphoreGive 替代 g_h.funcs->_h_unlock_mempool
        xSemaphoreGive(mp->spinlock);

#if H_MEM_STATS
        h_stats_g.mp_stats.num_reuse++;
        ESP_LOGV(MEM_TAG, "%p: num_reuse: %lu", mp, (unsigned long int)(h_stats_g.mp_stats.num_reuse));
#endif
    } else {
        xSemaphoreGive(mp->spinlock);

        buf = pvPortMalloc(MEMPOOL_ALIGNED(mp->block_size));  // 使用 FreeRTOS 的 pvPortMalloc 替代 MEM_ALLOC
#if H_MEM_STATS
        h_stats_g.mp_stats.num_fresh_alloc++;
        ESP_LOGV(MEM_TAG, "%p: num_alloc: %lu", mp, (unsigned long int)(h_stats_g.mp_stats.num_fresh_alloc));
#endif
    }
#else
    buf = pvPortMalloc(MEMPOOL_ALIGNED(nbytes));  // 使用 FreeRTOS 的 pvPortMalloc 替代 g_h.funcs->_h_malloc_align
#endif
    ESP_LOGV(MEM_TAG, "alloc %u bytes at %p", nbytes, buf);

    if (buf && need_memset)
        memset(buf, 0, nbytes);  // 使用标准库的 memset 替代 g_h.funcs->_h_memset

    return buf;
}

void mempool_free(struct mempool* mp, void *mem)
{
    if (!mem)
        return;
#ifdef H_USE_MEMPOOL
    if (!mp)
        return;

    xSemaphoreTake(mp->spinlock, portMAX_DELAY);

    SLIST_INSERT_HEAD(&(mp->head), (struct mempool_entry *)mem, entries);

    xSemaphoreGive(mp->spinlock);

#if H_MEM_STATS
    h_stats_g.mp_stats.num_free++;
    ESP_LOGV(MEM_TAG, "%p: num_ret: %lu", mp, (unsigned long int)(h_stats_g.mp_stats.num_free));
#endif

#else
    ESP_LOGV(MEM_TAG, "free at %p", mem);
    vPortFree(mem);  // 使用 FreeRTOS 的 vPortFree 替代 g_h.funcs->_h_free_align
#endif
}
