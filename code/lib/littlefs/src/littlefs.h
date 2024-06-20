#pragma once
#include <limits.h>
#include "hardware/flash.h"
#include "pico/flash.h"
#include "hardware/regs/addressmap.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include "hardware/sync.h"
#include "pico/time.h"
#include "./littlefs/lfs.h"

#define IKAFLY_FLASH_SIZE_BYTES (4*1024*1024) // 4MiB
#define FS_SIZE (1*1024*1024) // 1MiB
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
#define HW_FLASH_STORAGE_BYTES  FS_SIZE
#define HW_FLASH_STORAGE_BASE   (IKAFLY_FLASH_SIZE_BYTES - HW_FLASH_STORAGE_BYTES) // 655360

extern struct lfs_config lfs_pico_cfg;
extern SemaphoreHandle_t filesystemMutex;
