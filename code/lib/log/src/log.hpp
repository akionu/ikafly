#pragma once

#include <cstdio>
#include <cstring>
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
#define MAX_NAME_LEN 8
#define PERIOD_SAVE_FS 10 // save to fs each 10 writes
#define OFFSET_BASE_INTERNAL 0x200000
#define FS_SIZE 512

class Logging {
public:
    Logging();
    const uint32_t offset_base = OFFSET_BASE_INTERNAL;
    uint8_t fsdata[FS_SIZE] = {0};
    uint32_t* fsdata_offset = (uint32_t*)fsdata;
    uint32_t* fsdata_head = (uint32_t*)((uint8_t*)fsdata_offset + sizeof(uint32_t)*MAX_FILE);
    uint32_t* fsdata_max_size = (uint32_t*)((uint8_t*)fsdata_head + sizeof(uint32_t)*MAX_FILE);
    uint8_t* fsdata_name = (uint8_t*)fsdata_max_size + sizeof(uint32_t)*MAX_FILE;
    const uint32_t filesys_size = 3*sizeof(uint32_t)*MAX_FILE+sizeof(uint8_t)*MAX_NAME_LEN*MAX_FILE;

//    struct s_file_list {
//        uint32_t offset = 0;
//        uint32_t head = 0;
//        uint32_t max_size = 0;
//        uint8_t* name[12];
//    } file_list[MAX_FILE];
    
    bool init();
    bool mount();
    bool unmount();
    bool make(uint8_t* file_num, const char name[8], const uint32_t max_size);
    bool find(uint8_t* file_num, const char query_name[8], const uint8_t size_query_name);
    bool read(const uint8_t file_num, const void* buf, const uint32_t size);
    bool write(const uint8_t file_num, const void* buf, const uint32_t size); // write from first
    bool append(const uint8_t file_num, const void* buf, const uint32_t size); // write from middle
    void erase(const uint8_t file_num, const uint32_t size);
    bool nuke(); // erase filesystem & all files permanently
    bool save_fs_force(); // write file states into flash

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
    bool read_page(const uint32_t offset, uint8_t* buf, uint32_t size);
    uint32_t swap(uint32_t dat);
    void show_files(uint8_t from, uint8_t size);
    
    uint8_t* fsdata_get_name(uint8_t file_num);

    SemaphoreHandle_t flashMutex;
    uint8_t fs_n = 0; // file number of the file system
//    uint8_t file_list_head;
    uint8_t file_num_head;
    uint8_t write_cnt_from_last_save_fs = 0;
    // ew stands for erase, write
    struct flash_ew_t {
    	uint32_t offset;
    	const uint8_t* buf;
        uint32_t size;
    };
};
