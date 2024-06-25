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

#define MAX_FILE 5
#define OFFSET_BASE_INTERNAL 0x200000

class Logging {
public:
    const uint32_t offset_base = OFFSET_BASE_INTERNAL+0x100;
    struct {
        uint8_t* name[8];
        uint32_t offset = 0;
        uint32_t head = 0;
        uint32_t size = 0;
    } file_list[MAX_FILE];
    
    bool init();
    bool mount();
    void unmount();
    bool make(uint8_t* file_num, const char name[8], const uint32_t max_size);
    bool find(uint8_t* file_num, const char query_name[8], const uint8_t size_query_name);
    bool read(const uint8_t file_num, const void* buf, const uint32_t size);
    bool write(const uint8_t file_num, const void* buf, const uint32_t size); // write from first
    bool append(const uint8_t file_num, const void* buf, const uint32_t size); // write from middle
    void erase(const uint8_t file_num, const uint32_t size);
    bool nuke(); // erase filesystem & all files permanently

private:
    bool flashMutex_init();
    bool flashMutex_lock();
    bool flashMutex_unlock();
    static void erase_sector_unsafe(void *p);
    static void write_page_unsafe(void *p);
    bool erase_sector_safe(const uint32_t offset, uint32_t size);
    bool write_page_safe(const uint32_t offset, uint8_t* buf);
    bool write_less_than_page_safe(const uint32_t offset, uint8_t* buf, uint32_t size);
    bool write_page_internal_safe(const uint32_t offset, uint8_t* buf, uint32_t size);
    bool write_fs_data();
    bool is_first_write_fs = true;
    void read_page(const uint32_t offset, void* buf, uint32_t size);

    SemaphoreHandle_t flashMutex;
    uint8_t file_list_head;
    uint8_t tmp[FLASH_PAGE_SIZE];
    // ew stands for erase, write
    struct flash_ew_t {
    	uint32_t offset;
    	const uint8_t* buf;
        uint32_t size;
    };
    const uint32_t offset_base_internal = OFFSET_BASE_INTERNAL;
};
