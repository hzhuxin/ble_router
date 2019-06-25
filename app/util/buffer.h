/*!
 *    @file  buffer.h
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

#ifndef __BUFFER_H__
#define __BUFFER_H__

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/

/**
 * @brief Iterate callback, gets called when iterating through items
 *
 * @param oset  offset in the buffer
 * @param item  pointer to the item
 *
 * @return true if the iteration should continue
 */
typedef bool(*buffer_iter_func_t)(int oset, void* item);

struct buffer_ops;

typedef struct bufer {
    const struct buffer_ops* ops;
    void* memory;
    int16_t capacity;
    int16_t itemsize;
    int16_t count;
    int16_t write;
    int16_t read;
    int8_t  fromheap;
    int8_t  initiated;
} buffer_t;

// Operation pointer
typedef struct buffer_ops {

    // Deinit the object, free memory if necessary
    int (*deinit) (buffer_t* buf);
    // Clear, remove all items
    int (*clear) (buffer_t* buf);

    // Write items, return the number of items written
    int (*write) (buffer_t* buf, void* item, int cnt);
    // Read items, return the number of items read
    int (*read) (buffer_t* buf, void* item, int cnt);
    // Remove items, return the number of items read
    int (*remove) (buffer_t* buf, int cnt);

    // Push an item, return the number of items pushed
    int (*push) (buffer_t* buf, void* item);
    // Pop an item, return the number of items poped
    int (*pop) (buffer_t* buf, void* item);

    // Get an item at given offset(may be negative), return a pointer to the item if succeed
    void* (*get) (buffer_t* buf, int oset);
    // Iterate through the buffer, call a function on every item, interrupts if the funcion returns false
    int (*iterate) (buffer_t* buf, int step, buffer_iter_func_t func);
} buffer_ops_t;

/* Global variables ----------------------------------------------------------*/
extern const buffer_ops_t buf_ops;

/* Defines -------------------------------------------------------------------*/
// Memory allocate
#ifndef _BUFFER_MALLOC
#define _BUFFER_MALLOC      malloc
#endif
// Memory free
#ifndef _BUFFER_FREE
#define _BUFFER_FREE        free
#endif

/**
 * @brief Create a buffer from heap memory
 *
 * @param itemsize  size of an item
 * @param capacity  number of items the buffer can store
 *
 * @return the buffer object
 */
#define BUFFER_CREATE(_itemsize, _capacity)                     \
    {                                                           \
        .itemsize = (_itemsize),                                \
        .capacity = (_capacity),                                \
        .memory = _BUFFER_MALLOC((_itemsize) * (_capacity)),    \
        .fromheap = true,                                       \
        .initiated = true,                                      \
        .read = 0,                                              \
        .write = 0,                                             \
        .count = 0,                                             \
        .ops= &buf_ops,                                         \
    }

/**
 * @brief Create a buffer from given memory
 *
 * @param itemsize  size of an item
 * @param capacity  number of items the buffer can store
 * @param memory    memory to store items
 *
 * @return the buffer object
 */
#define BUFFER_CREATE_STATIC(_itemsize, _capacity, _memory)     \
    {                                                           \
        .itemsize = (_itemsize),                                \
        .capacity = (_capacity),                                \
        .memory = (_memory),                                    \
        .fromheap = false,                                      \
        .initiated = true,                                      \
        .read = 0,                                              \
        .write = 0,                                             \
        .count = 0,                                             \
        .ops= &buf_ops,                                         \
    }


void buffer_delete(buffer_t* buf);
buffer_t* buffer_new(int itemsize, int capacity);

#endif // #ifndef __BUFFER_H__

