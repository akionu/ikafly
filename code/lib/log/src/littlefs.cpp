/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A little flash file system for the Raspberry Pico
 *
 * Modified: Akio M, for simple c++ wrapper
 *
 */

#include "littlefs.hpp"
#include <cstring>

#ifdef LFS_THREADSAFE
SemaphoreHandle_t LFS::flashMutex;
#endif
struct lfs_config LFS::pico_cfg;

// public functions
int LFS::init() {

    pico_cfg.read = &flash_fs_read;
    pico_cfg.prog = &flash_fs_prog;
    pico_cfg.erase = &flash_fs_erase;
    pico_cfg.sync = &flash_fs_sync;
#ifdef LFS_THREADSAFE
    LFS::flashMutex = xSemaphoreCreateMutex();
    pico_cfg.lock = &flash_fs_lock;
    pico_cfg.unlock = &flash_fs_unlock;
#endif

    // block device configuration
    pico_cfg.read_size = 1;
    pico_cfg.prog_size = FLASH_PAGE_SIZE;
    pico_cfg.block_size = FLASH_SECTOR_SIZE;
    pico_cfg.block_count = FS_SIZE / FLASH_SECTOR_SIZE;
    pico_cfg.block_cycles = (int32_t)500;
    pico_cfg.cache_size = 256;
    pico_cfg.lookahead_size = 32;

#ifdef LFS_THREADSAFE
    if (LFS::flashMutex != NULL) return LFS_ERR_OK;
    else return (-1);
#else
    return true;
#endif
}

// public LFS functions
int LFS::format() {
    return lfs_format(&lfs, &pico_cfg);
}

int LFS::mount() {
    return lfs_mount(&lfs, &pico_cfg);
}

int LFS::unmount() {
    return lfs_unmount(&lfs);
}

int LFS::remove(const char* path) {
    return lfs_remove(&lfs, path);
}

int LFS::rename(const char* oldpath, const char* newpath) {
    return lfs_rename(&lfs, oldpath, newpath);
}

int LFS::stat(const char* path, struct lfs_info *info) {
    return lfs_stat(&lfs, path, info);
}

lfs_ssize_t LFS::getattr(const char* path, uint8_t type, void* buffer, lfs_size_t size) {
    return lfs_getattr(&lfs, path, type, buffer, size);
}

int LFS::setattr(const char* path, uint8_t type, const void* buffer, lfs_size_t size) {
    return lfs_setattr(&lfs, path, type, buffer, size);
}

int LFS::removeattr(const char* path, uint8_t type) {
    return lfs_removeattr(&lfs, path, type);
}

// file operations
int LFS::file_open(lfs_file_t* file, const char* path, int flags) {
    return lfs_file_open(&lfs, file, path, flags);
}

int LFS::file_opencfg(lfs_file_t* file, const char* path, int flags, const struct lfs_file_config *config) {
    return lfs_file_opencfg(&lfs, file, path, flags, config);
}

int LFS::file_close(lfs_file_t* file) {
    return lfs_file_close(&lfs, file);
}

int LFS::file_sync(lfs_file_t* file) {
    return lfs_file_sync(&lfs, file);
}

lfs_ssize_t LFS::file_read(lfs_file_t* file, void* buffer, lfs_size_t size) {
    return lfs_file_read(&lfs, file, buffer, size);
}

lfs_ssize_t LFS::file_write(lfs_file_t *file, const void *buffer, lfs_size_t size) {
    return lfs_file_write(&lfs, file, buffer, size);
}

lfs_soff_t LFS::file_seek(lfs_file_t *file, lfs_soff_t off, int whence) {
    return lfs_file_seek(&lfs, file, off, whence);
}

int LFS::file_truncate(lfs_file_t *file, lfs_off_t size) {
    return lfs_file_truncate(&lfs, file, size);
}

lfs_soff_t LFS::file_tell(lfs_file_t *file) {
    return lfs_file_tell(&lfs, file);
}

int LFS::file_rewind(lfs_file_t *file) {
    return lfs_file_rewind(&lfs, file);
}

lfs_soff_t LFS::file_size(lfs_file_t *file) {
    return lfs_file_size(&lfs, file);
}

/// Directory operations ///
int LFS::mkdir(const char *path) {
    return lfs_mkdir(&lfs, path);
}

int LFS::dir_open(lfs_dir_t *dir, const char *path) {
    return lfs_dir_open(&lfs, dir, path);
}

int LFS::dir_close(lfs_dir_t *dir) {
    return lfs_dir_close(&lfs, dir);
}

int LFS::dir_read(lfs_dir_t *dir, struct lfs_info *info) {
    return lfs_dir_read(&lfs, dir, info);
}

int LFS::dir_seek(lfs_dir_t *dir, lfs_off_t off) {
    return lfs_dir_seek(&lfs, dir, off);
}

lfs_soff_t LFS::dir_tell(lfs_dir_t *dir) {
    return lfs_dir_tell(&lfs, dir);
}

int LFS::dir_rewind(lfs_dir_t *dir) {
    return lfs_dir_rewind(&lfs, dir);
}

/// Filesystem-level filesystem operations
int LFS::fs_stat(struct lfs_fsinfo *fsinfo) {
    return lfs_fs_stat(&lfs, fsinfo);
}

lfs_ssize_t LFS::fs_size() {
    return lfs_fs_size(&lfs);
}

#if 0
int LFS::fs_traverse(int (*cb)(void*, lfs_block_t), void *data) {
    return lfs_fs_traverse((*cb)(void*, lfs_block_t), data);
}
#endif

int LFS::fs_mkconsistent() {
    return lfs_fs_mkconsistent(&lfs);
}

int LFS::fs_gc() {
    return lfs_fs_gc(&lfs);
}

int LFS::fs_grow(size_t block_count) {
    return lfs_fs_grow(&lfs, block_count);
}

#ifdef LFS_MIGRATE
int LFS::migrate() {
    return lfs_migrate(&lfs);
}
#endif

// private:
int LFS::flash_fs_read(const struct lfs_config* config, lfs_block_t block, lfs_off_t off, void* buf, lfs_size_t size) {
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * config->block_size) + off;

    memcpy(buf, (unsigned char*)addr, size);
    return LFS_ERR_OK;
}

void LFS::flash_prog_safe(void *p) {
    struct flash_ew_t *f = (struct flash_ew_t*)p;
    flash_range_program(f->offset, f->buf, f->size);
}

void LFS::flash_erase_safe(void *p) {
    struct flash_ew_t *f = (struct flash_ew_t*)p;
    flash_range_erase(f->offset, f->size);
}

int LFS::flash_fs_prog(const struct lfs_config* config, lfs_block_t block, lfs_off_t off, const void* buf, lfs_size_t size) {
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * config->block_size) + off;

    // printf("[FS] WRITE: %p, %d\n", addr, size);
    struct flash_ew_t fp = {
        .offset = addr,
        .buf = (uint8_t*)buf,
        .size = size
    };

    flash_safe_execute(flash_prog_safe, &fp, UINT32_MAX);

    return LFS_ERR_OK;
}

int LFS::flash_fs_erase(const struct lfs_config* config, lfs_block_t block) {
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * config->block_size);

    // printf("[FS] ERASE: %p\n", offset);
    struct flash_ew_t fe = {
        .offset = offset,
        .size = config->block_size
    };

    flash_safe_execute(flash_erase_safe, &fe, UINT32_MAX);

    return LFS_ERR_OK;
}

int LFS::flash_fs_sync(const struct lfs_config* config) {
    return LFS_ERR_OK;
}

#ifdef LFS_THREADSAFE
int LFS::flash_fs_lock(const struct lfs_config* config) {
    if (xSemaphoreTake(LFS::flashMutex, (TickType_t)20) == pdTRUE) {
        return LFS_ERR_OK;
    } else {
        return -1;
    }
}

int LFS::flash_fs_unlock(const struct lfs_config* config) {
    if (xSemaphoreGive(LFS::flashMutex) == pdTRUE) {
        return LFS_ERR_OK;
    } else {
        return -1;
    }
}
#endif
