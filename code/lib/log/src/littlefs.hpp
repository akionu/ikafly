#pragma once
#include <limits.h>
#include "hardware/flash.h"
#include "pico/flash.h"
#include "hardware/regs/addressmap.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include "hardware/sync.h"
#include "pico/time.h"

#include "littlefs/lfs.h"

#define FS_SIZE (2*1024*1024)
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE) 
#define HW_FLASH_STORAGE_BYTES  FS_SIZE
#define HW_FLASH_STORAGE_BASE   (0x200000)

#ifndef LFS_THREADSAFE
#warning "LFS_THREADSAFE did not defined"
#endif

class LFS {
public:
    /// Filesystem functions
    int init();
    int format();
    int mount();
    int unmount();
    int remove(const char* path);
    int rename(const char* oldpath, const char* newpath);
    int stat(const char* path, struct lfs_info *info);
    lfs_ssize_t getattr(const char* path, uint8_t type, void* buffer, lfs_size_t size);
    int setattr(const char* path, uint8_t type, const void* buffer, lfs_size_t size);
    int removeattr(const char* path, uint8_t type);
    /// File operations
    int file_open(lfs_file_t* file, const char* path, int flags);
    int file_opencfg(lfs_file_t* file, const char* path, int flags, const struct lfs_file_config *config);
    int file_close(lfs_file_t* file);
    int file_sync(lfs_file_t* file);
    lfs_ssize_t file_read(lfs_file_t* file, void* buffer, lfs_size_t size);
    lfs_ssize_t file_write(lfs_file_t *file, const void *buffer, lfs_size_t size);
    lfs_soff_t file_seek(lfs_file_t *file, lfs_soff_t off, int whence);
    int file_truncate(lfs_file_t *file, lfs_off_t size);
    lfs_soff_t file_tell(lfs_file_t *file);
    int file_rewind(lfs_file_t *file);
    lfs_soff_t file_size(lfs_file_t *file);

    /// Directory operations
    int mkdir(const char *path);
    int dir_open(lfs_dir_t *dir, const char *path);
    int dir_close(lfs_dir_t *dir);
    int dir_read(lfs_dir_t *dir, struct lfs_info *info);
    int dir_seek(lfs_dir_t *dir, lfs_off_t off);
    lfs_soff_t dir_tell(lfs_dir_t *dir);
    int dir_rewind(lfs_dir_t *dir);

    /// Filesystem-level filesystem operations
    int fs_stat(struct lfs_fsinfo *fsinfo);
    lfs_ssize_t fs_size();
#if 0
    int fs_traverse(int (*cb)(void*, lfs_block_t), void *data);
#endif
    int fs_mkconsistent();
    int fs_gc();
    int fs_grow(size_t block_count);
#ifdef LFS_MIGRATE
    int migrate();
#endif

private:
    lfs_t lfs;
    static void flash_erase_safe(void *p);
    static void flash_prog_safe(void *p);

    static int flash_fs_read(const struct lfs_config* config, lfs_block_t block, lfs_off_t off, void* buf, lfs_size_t size);
    static int flash_fs_prog(const struct lfs_config* config, lfs_block_t block, lfs_off_t off, const void* buf, lfs_size_t size);
    static int flash_fs_erase(const struct lfs_config* config, lfs_block_t block);
    static int flash_fs_sync(const struct lfs_config* config);
#ifdef LFS_THREADSAFE
    static SemaphoreHandle_t flashMutex;
    static int flash_fs_unlock(const struct lfs_config* config);
    static int flash_fs_lock(const struct lfs_config* config);
#endif

    static struct lfs_config pico_cfg;
//    {
//        // user provided value
//        .context = flashMutex, // SemaphoreHandle_t == QueueDefinition*
//        // block device operations
//        .read = &flash_fs_read,
//        .prog = &flash_fs_prog,
//        .erase = &flash_fs_erase,
//        .sync = &flash_fs_sync,
//        .lock = &flash_fs_lock,
//        .unlock = &flash_fs_unlock,
//
//        // block device configuration
//        .read_size = 1,
//        .prog_size = FLASH_PAGE_SIZE,
//        .block_size = FLASH_SECTOR_SIZE,
//        .block_count = FS_SIZE / FLASH_SECTOR_SIZE,
//        .block_cycles = (int32_t)500,
//        .cache_size = 256,
//        .lookahead_size = 32
//    };
    
    struct flash_ew_t {
        uint32_t offset;
        uint8_t* buf;
        size_t size;
    };
};
