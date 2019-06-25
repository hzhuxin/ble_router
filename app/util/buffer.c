/*!
 *    @file  buffer.c
 *   @brief  The generic buffer module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  01/29/2018
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "buffer.h"

/* Defines -------------------------------------------------------------------*/
#define _ARRAY_SIZE(a)  (sizeof(a) / sizeof(*a))

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Deinit buffer
 *
 * @return 0 if success
 */
static int deinit (buffer_t* buf) {
  if(buf->memory && buf->fromheap) {
    buf->fromheap = false;
    _BUFFER_FREE(buf->memory);
  }

  buf->initiated = false;
  return 0;
}

/**
 * @brief Clear buffer
 *
 * @return 0 if success
 */
static int clear (buffer_t* buf) {
  buf->read = 0;
  buf->write = 0;
  buf->count = 0;

  return 0;
}

/**
 * @brief Write items from buffer
 *
 * @return number of items written
 */
static int write (buffer_t* buf, void* item, int cnt) {
  // Check input
  if(!buf || !buf->initiated) {
    return -1;
  }
  if(!item || cnt < 0) {
    return -1;
  }

  // Check number of items
  cnt = cnt > buf->capacity - buf->count ? buf->capacity - buf->count : cnt;

  // Read items
  if(cnt > 0) {
    int n = buf->write + cnt > buf->capacity ? buf->capacity - buf->write : cnt;
    memcpy((char*)buf->memory + buf->write * buf->itemsize, item,  n * buf->itemsize);
    if(n < cnt) {
      memcpy(buf->memory, (char*)item + n * buf->itemsize, (cnt - n) * buf->itemsize);
    }

    buf->count += cnt;
    buf->write += cnt;
    buf->write = buf->write < buf->capacity ? buf->write : buf->write - buf->capacity;
  }

  return cnt;
}

/**
 * @brief Read items from buffer
 *
 * @return number of items read
 */
static int read (buffer_t* buf, void* item, int cnt) {
  // Check input
  if(!buf || !buf->initiated) {
    return -1;
  }
  if(!item || cnt < 0) {
    return -1;
  }

  // Check number of items
  cnt = cnt > buf->count ? buf->count : cnt;

  // Read items
  if(cnt > 0) {
    int n = buf->read + cnt > buf->capacity ? buf->capacity - buf->read : cnt;
    memcpy(item, (char*)buf->memory + buf->read * buf->itemsize, n * buf->itemsize);
    if(n < cnt) {
      memcpy((char*)item + n * buf->itemsize, buf->memory, (cnt - n) * buf->itemsize);
    }
  }

  return cnt;
}

/**
 * @brief Remove items from buffer
 *
 * @return number of items removed
 */
static int _remove (buffer_t* buf, int cnt) {
  // Check input
  if(!buf || !buf->initiated) {
    return -1;
  }
  if(cnt < 0) {
    return -1;
  }

  // Check number of items
  cnt = cnt > buf->count ? buf->count : cnt;

  // Remove items
  buf->count -= cnt;
  buf->read += cnt;
  buf->read = buf->read < buf->capacity ? buf->read : buf->read - buf->capacity;

  return cnt;
}

/**
 * @brief Push to buffer
 *
 * @return number of items pushed
 */
static int push (buffer_t* buf, void* item) {
  return write(buf, item, 1);
}

/**
 * @brief Pop from buffer
 *
 * @return the item poped
 */
static int pop (buffer_t* buf, void* item) {
  int ret = read(buf, item, 1);

  if(ret == 1) {
    ret = _remove(buf, 1);
  }

  return ret;
}

/**
 * @brief Peek an item from buffer
 *
 * @return the item peeked
 */
static void* get (buffer_t* buf, int oset) {
  // Check input
  if(!buf || !buf->initiated) {
    return NULL;
  }

  // Check number of items
  if(oset < 0) {
    oset += buf->count;
  }
  if(oset < 0 || oset >= buf->count) {
    return NULL;
  }

  // Read item
  return (char*)buf->memory + ((buf->read + oset) % buf->capacity) * buf->itemsize;
}

/**
 * @brief Iterate through all items, call a function
 *
 * @return number of items iterated
 */
static int iterate (buffer_t* buf, int step, buffer_iter_func_t func) {
  // Check input
  if(!buf || !buf->initiated) {
    return -1;
  }
  if(!func || !step) {
    return -1;
  }

  // Iterate items
  int init = step > 0 ? 0 : -1;
  int iter = (buf->count + 1) / abs(step);
  for(int i = 0; i < iter; i++) {
    int oset = init + step * i;
    void* item = get(buf, oset);
    if(!item || !func(oset, item)) {
      break;
    }
  }

  return iter;
}

/* Global variables ----------------------------------------------------------*/
const buffer_ops_t buf_ops = {
  .deinit = deinit,
  .clear = clear,
  .write = write,
  .read = read,
  .remove = _remove,
  .push = push,
  .pop = pop,
  .iterate = iterate,
  .get = get,
};

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Create a buffer
 *
 * @param itemsize  size of an item
 * @param capacity  number of items the buffer can store
 *
 * @return pointer to the buffer object
 */
buffer_t* buffer_new(int itemsize, int capacity) {
  buffer_t* buf = pvPortMalloc(sizeof(buffer_t));
  if(!buf) {
    return NULL;
  }

  buf->memory = pvPortMalloc(itemsize * capacity);
  if(!buf->memory) {
    vPortFree(buf);
    return NULL;
  }

  buf->itemsize = itemsize;
  buf->capacity = capacity;
  buf->fromheap = true;
  buf->initiated = true;
  buf->read = 0;
  buf->write = 0;
  buf->count = 0;
  buf->ops= &buf_ops;

  return buf;
}


/**
 * @brief Free a buffer
 *
 * @return None
 */
void buffer_delete(buffer_t* buf) {
  if(!buf) {
    return;
  }

  if(buf->memory && buf->fromheap) {
    vPortFree(buf->memory);
    buf->fromheap = false;
    buf->memory = NULL;
  }

  vPortFree(buf);
}
