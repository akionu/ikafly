#pragma once

#include <cstdio>
#include <cstring>
#include <cstdlib>
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
    typedef uint8_t file_num_t;
    const uint32_t offset_base = OFFSET_BASE_INTERNAL;
    uint8_t fsdata[FS_SIZE] = {0};
    uint32_t* fsdata_offset = (uint32_t*)fsdata;
    uint32_t* fsdata_head = (uint32_t*)((uint8_t*)fsdata_offset + sizeof(uint32_t)*MAX_FILE);
    uint32_t* fsdata_max_size = (uint32_t*)((uint8_t*)fsdata_head + sizeof(uint32_t)*MAX_FILE);
    uint8_t* fsdata_name = (uint8_t*)fsdata_max_size + sizeof(uint32_t)*MAX_FILE;
    const uint32_t filesys_size = 3*sizeof(uint32_t)*MAX_FILE+sizeof(uint8_t)*MAX_NAME_LEN*MAX_FILE;
    
    /**
     * @brief initialize filesystem
     *
     * @return true if success, false if fails
     */
    bool init();


    /**
     * @brief mount filesystem if exists
     *
     * @return true if success, false if not found
     */
    bool mount();

    /**
     * @brief unmount filesystem
     *
     * @return always true
     */
    bool unmount();

    /**
     * @brief make a new file in the filesystem
     *
     * @param[out] file_num filenumber for made file
     * @param[in] name filename 
     * @param[in] max_size maximum size to be written
     * @return true if success, false if fails
     */
    bool make(file_num_t* file_num, const char name[8], const uint32_t max_size);

    /**
     * @brief find a file should be exists
     *
     * @param[out] file_num filenumber of found file
     * @param[in] query_name filename to be in search
     * @param[in] size_query_name size of query_name, should be 'sizeof(query_name)-1'
     * @return true if success
     */
    bool find(file_num_t* file_num, const char query_name[8], const uint8_t size_query_name);

    /**
     * @brief read from file
     *
     * @param[in] file_num filenumber of file to be read
     * @param[out] buf output buffer
     * @param[in] size number of sizeof(uint8_t) to be read
     * @return true if success, false if fails
     */
    bool read(const file_num_t file_num, const void* buf, const uint32_t size);

    /**
     * @brief write to file, replacing exists datas
     * @detail each PERIOD_SAVE_FS time written, write calls save_fs_force() to save filesystem
     *
     * @param[in] file_num filenumber of file to be written
     * @param[in] buf input buffer
     * @param[in] size number of sizeof(uint8_t) to be written
     * @return true if success, false if fails
     */
    bool write(const file_num_t file_num, const void* buf, const uint32_t size);

    /**
     * @brief append to file, write to its tail
     *
     * @param[in] file_num filenumber of file to be written
     * @param[in] buf input buffer
     * @param[in] size number of sizeof(uint8_t) to be written
     * @return true if success, false if fails
     */
    bool append(const file_num_t file_num, const void* buf, const uint32_t size);

    /**
     * @brief erase file (does not touch in flash)
     *
     * @param[in] file_num filenumber of file to be erase
     */
    void erase(const file_num_t file_num);

    /**
     * @brief erase all files & filesystem permanently
     * @detail clear all bytes in flash (to 0xff)
     *
     * @return true if success false if fails
     */
    bool nuke();

    /**
     * @brief save existing filesystem to flash
     * @detail each PERIOD_SAVE_FS time written, write() and append() call this internally
     * @attention we have to call this before poweroff
     *
     * @return true if success, false if fails
     */
    bool save_fs_force();

    /**
     * @brief helper function to get n-th file's name pointer
     *
     * @param[in] file_num filenumber of file 
     */
    uint8_t* fsdata_get_name(file_num_t file_num);

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
    bool save_fs_periodically();
    void show_files(uint8_t from, uint8_t size);

    SemaphoreHandle_t flashMutex;
    file_num_t fs_n = 0; // file number of the file system
    file_num_t file_num_head;
    // ew stands for erase, write
    struct flash_ew_t {
    	uint32_t offset;
    	const uint8_t* buf;
        uint32_t size;
    };
};
