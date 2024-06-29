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
#include "spiffs.h"
#include "spiffs_config.h"

static spiffs fs;

#define LOG_PAGE_SIZE       256

static uint8_t spiffs_work_buf[LOG_PAGE_SIZE*2];
static uint8_t spiffs_fds[32*4];
static uint8_t spiffs_cache_buf[(LOG_PAGE_SIZE+32)*4];

void my_spiffs_mount();
bool flashMutex_init();
bool flashMutex_lock();
bool flashMutex_unlock();
void erase_sector_unsafe(void* p);
void write_page_unsafe(void *p);
void read(const uint32_t offset, uint32_t size, uint8_t* buf);
bool erase_sector_safe(const uint32_t offset, uint32_t size);
bool write_page_safe(const uint32_t offset, uint32_t size, uint8_t* buf);

static s32_t my_spiffs_read(u32_t addr, u32_t size, u8_t *dst);
static s32_t my_spiffs_write(u32_t addr, u32_t size, u8_t *src);
static s32_t my_spiffs_erase(u32_t addr, u32_t size);

static void test_spiffs() {
    char buf[12];

    // Surely, I've mounted spiffs before entering here
    printf("test_spiffs:\n");
    spiffs_file fd = SPIFFS_open(&fs, "my_file", SPIFFS_CREAT | SPIFFS_TRUNC | SPIFFS_RDWR, 0);
    if (SPIFFS_write(&fs, fd, (u8_t *)"Hello world", 12) < 0) printf("errno %i\n", SPIFFS_errno(&fs));
    SPIFFS_close(&fs, fd); 

    fd = SPIFFS_open(&fs, "my_file", SPIFFS_RDWR, 0);
    if (SPIFFS_read(&fs, fd, (u8_t *)buf, 12) < 0) printf("errno %i\n", SPIFFS_errno(&fs));
    SPIFFS_close(&fs, fd);

    printf("--> %s <--\n", buf);
}

static s32_t my_spiffs_read(u32_t addr, u32_t size, u8_t *dst) {
    read(addr, size, dst);
    return SPIFFS_OK;
}

static s32_t my_spiffs_write(u32_t addr, u32_t size, u8_t *src) {
    write_page_safe(addr, size, src);
    return SPIFFS_OK;
}

static s32_t my_spiffs_erase(u32_t addr, u32_t size) {
    erase_sector_safe(addr, size);
    return SPIFFS_OK;
}

void my_spiffs_mount() {
    spiffs_config cfg;
//    cfg.phys_size = 2*1024*1024; // use all spi flash
//    cfg.phys_addr = 0x200000; // start spiffs at start of spi flash
//    cfg.phys_erase_block = 4*1024; // according to datasheet
//    cfg.log_block_size = 64*1024; // let us not complicate things
//    cfg.log_page_size = LOG_PAGE_SIZE; // as we said

    cfg.hal_read_f = my_spiffs_read;
    cfg.hal_write_f = my_spiffs_write;
    cfg.hal_erase_f = my_spiffs_erase;

    int res = SPIFFS_mount(&fs,
                           &cfg,
                           spiffs_work_buf,
                           spiffs_fds,
                           sizeof(spiffs_fds),
                           spiffs_cache_buf,
                           sizeof(spiffs_cache_buf),
                           0);
    printf("mount res: %i\n", res);

    if (res == SPIFFS_ERR_NOT_A_FS) {
        res = SPIFFS_format(&fs);
        printf("format res: %i\n", res);
    } else {
        SPIFFS_unmount(&fs);
    }
    
    res = SPIFFS_mount(&fs,
                       &cfg,
                       spiffs_work_buf,
                       spiffs_fds,
                       sizeof(spiffs_fds),
                       spiffs_cache_buf,
                       sizeof(spiffs_cache_buf),
                       0);

    printf("mount(2) res: %i\n", res);
}
SemaphoreHandle_t flashMutex;

// ew stands for erase, write
struct flash_ew_t {
    uint32_t offset;
    uint32_t size;
    const uint8_t* buf;
};

bool flashMutex_init() {
    flashMutex = xSemaphoreCreateMutex();
    return (flashMutex != NULL);
}

bool flashMutex_lock() {
    return (xSemaphoreTake(flashMutex, (TickType_t)20) == pdTRUE);
}

bool flashMutex_unlock() {
    return (xSemaphoreGive(flashMutex) == pdTRUE);
}

void erase_sector_unsafe(void* p) {
    struct flash_ew_t* fp = (struct flash_ew_t*)p;
    flash_range_erase(fp->offset, fp->size); // FLASH_SECTOR_SIZE == 4096Byte defined in flash.h
}

void write_page_unsafe(void *p) {
    struct flash_ew_t* fp = (struct flash_ew_t*)p;
    flash_range_program(fp->offset, fp->buf, fp->size);
}

void read(const uint32_t offset, uint32_t size, uint8_t* buf) {
    const uint8_t *p = (const uint8_t *)(XIP_BASE + offset);
    memcpy(buf, p, size);
}

bool erase_sector_safe(const uint32_t offset, uint32_t size) {
    struct flash_ew_t f = {
        .offset = offset,
        .size   = size
    };
  //  if (flashMutex_lock()) {
        int8_t ret = flash_safe_execute(erase_sector_unsafe, (void*)&f, UINT32_MAX);
//        printf("locked, ret: %d ", ret);
   //    return (flashMutex_unlock() && (PICO_OK == ret));
   //}
    return false;
}

bool write_page_safe(const uint32_t offset, uint32_t size, uint8_t* buf) {
    struct flash_ew_t f = {
        .offset = offset,
        .size   = size,
        .buf    = buf
    };
//    if (flashMutex_lock()) {
        int8_t ret = flash_safe_execute(write_page_unsafe, (void*)&f, UINT32_MAX);
//        printf("locked, ret: %d ", ret);
 //       return (flashMutex_unlock() && (PICO_OK == ret));
  //  }
    return false;
}

void task_spiffs(void* p) {
    flashMutex_init();
    
    my_spiffs_mount();
    test_spiffs();
    while(1) {}
}

int main(void) {
    stdio_usb_init();
    sleep_ms(2000);
    printf("\n%s\n", __FILE_NAME__);

    xTaskCreate(task_spiffs, "task_spiffs", 512, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1) {}
}

