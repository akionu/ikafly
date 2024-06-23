#pragma once

#include <stdio.h>
#include <string.h>
#include "hardware/timer.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"

#define MAX_FILE 10

class Logging {
public:
    const uint32_t offset_base = 0x200000;
    typedef struct {
        uint8_t* name[8];
        uint32_t offset = 0;
        uint32_t head = 0;
        uint32_t size = 0;
        bool erase = false; 
    } file_list_t;
    file_list_t file_list[MAX_FILE];
    
    bool init();
    bool make(uint8_t* file_num, const char name[8], const uint32_t max_size);
    bool open(const uint8_t file_num);
    bool read(const uint8_t file_num, const void* buf, const uint32_t size);
    bool write(const uint8_t file_num, const void* buf, const uint32_t size); // write from first
    bool append(const uint8_t file_num, const void* buf, const uint32_t size); // write from middle
    bool erase(const uint8_t file_num, const uint32_t size);
    bool erase_sector_safe(const uint32_t offset);

private:
    SemaphoreHandle_t flashMutex;
    uint8_t file_num_now;
    bool flashMutex_init();
    bool flashMutex_lock();
    bool flashMutex_unlock();
    static void erase_sector_unsafe(void *p);
    static void write_page_unsafe(void *p);
    bool write_page_safe(const uint32_t offset, uint8_t* buf);
    bool write_less_than_page_safe(const uint32_t offset, uint8_t* buf, uint32_t size);
    void read_page(const uint32_t offset, uint8_t* buf, uint32_t size);
    uint8_t file_list_head;
    // ew stands for erase, write
    struct flash_ew_t {
    	uint32_t offset;
    	const uint8_t* buf;
    } ew;
    uint8_t tmp[FLASH_PAGE_SIZE];
};
