#include "log.hpp"
Logging::Logging() {
}

bool Logging::init() {
    bool ret = false;
    ret = flashMutex_init();
    return ret;
}

bool Logging::mount() {
    bool ret = false;
    // restore fs data from flash

    read_page(offset_base, fsdata, filesys_size);
    //printf("mount: ");
//  for (uint16_t i = 0; i < filesys_size; i++) printf("%02x", fsdata[i]);
    //printf("\n");
    show_files(0, MAX_FILE);
    if (find(&fs_n, "filesys", 7)) {
        //printf("mount: OK!\n");
        ret = true;
    } else {
        //printf("mount: err\n");
        unmount();
        ret = false;
    }
    return ret;
}

bool Logging::unmount() {
    //printf("unmount\n");
    file_num_head = 0;
    memset(fsdata, 0, filesys_size);
    return true;
}

bool Logging::make(file_num_t* file_num, const char name[8], const uint32_t max_size) {
    const uint8_t fnum = file_num_head;
    // offset
    if (fnum == 0) fsdata_offset[fnum] = offset_base;
    else           fsdata_offset[fnum] = fsdata_offset[fnum-1] + fsdata_max_size[fnum-1];
    // head
    fsdata_head[fnum] = 0;
    // max_size
    fsdata_max_size[fnum] = max_size;
    // name
    memcpy((void*)fsdata_get_name(fnum), name, 8);
    
    *file_num = fnum;
    file_num_head++;

    show_files(fnum, 1);

    return true; 
}

bool Logging::find(file_num_t* file_num, const char query_name[8], const uint8_t size_query_name) {
    bool ret = false;
    for (uint8_t i = 0; i < MAX_FILE; i++) {
        if (std::memcmp((void*)fsdata_get_name(i), query_name, size_query_name) == 0) {
            *file_num = i;
            ret = true;
        }
    }
    return ret;
}

bool Logging::read(const file_num_t file_num, const void* buf, const uint32_t size) {
    bool ret = false;
    if (fsdata_head[file_num] >= size) {
        ret = read_page(fsdata_offset[file_num], (uint8_t*)buf, size);
    }
    return ret;
}

bool Logging::write(const file_num_t file_num, const void* buf, const uint32_t size) {
    bool ret = false;
    printf("write: offset: 0x%02x, size: %d\n", fsdata_offset[file_num], size);
    if (size <= fsdata_max_size[file_num]) {
        ret = write_page_internal_safe(fsdata_offset[file_num], (uint8_t*)buf, size);
    } else {
        printf("size > fsdata_max_size[file_num]\n");
    }

    if (ret) {
        fsdata_head[file_num] = size;
        save_fs_periodically();
    }
    return ret;
}

bool Logging::append(const file_num_t file_num, const void* buf, const uint32_t size) {
    bool ret = false;
    if ((fsdata_head[file_num]+size)<=fsdata_max_size[file_num]) {
        ret = write_page_internal_safe(fsdata_offset[file_num]+fsdata_head[file_num], (uint8_t*)buf, size);
    }

    if (ret) {
        fsdata_head[file_num] += size;
        save_fs_periodically();
    }
    return ret;
}

void Logging::erase(const file_num_t file_num) {
    fsdata_head[file_num] = 0;
}

bool Logging::nuke() {
    bool ret = false;
    ret = erase_sector_safe(offset_base, FLASH_SECTOR_SIZE);
    make(&fs_n, "filesys", FS_SIZE);
    return ret;
}

bool Logging::save_fs_force() {
    bool ret = false;
    ret = write(fs_n, fsdata, filesys_size);
    return ret;
}

uint8_t* Logging::fsdata_get_name(file_num_t file_num) {
    return (&fsdata_name[MAX_NAME_LEN*file_num]);
}

//private:
bool Logging::write_less_than_page_safe(const uint32_t offset, uint8_t* buf, uint32_t size) {
    bool ret = false;
    static uint8_t tmp[FLASH_PAGE_SIZE] = {0};
    memset((void*)tmp, 0, FLASH_PAGE_SIZE);
    uint32_t diff = 0;

    uint32_t begin = offset&0xffff00;
    uint32_t base = offset&0x0000ff;
    // read first then write
    read_page(begin, tmp, FLASH_PAGE_SIZE);
    for (uint16_t i = 0; i < size; i++) {
        if (tmp[base+i] != buf[i]) diff++;
        tmp[base+i] = buf[i];
    }

    ret = write_page_safe(begin, tmp);
    //printf("wltps: offset: 0x%x, size: %d\n", offset, size);
    //printf("wltps: ret: %s\n", ret?"ok":"ng");
    return ret;
}

bool Logging::write_page_internal_safe(const uint32_t offset, uint8_t* buf, uint32_t size) {
    bool ret = false;
    // is size over a page?
    if ((offset&0xff)+size <= 0xff) {
        // size is under a page
//      printf("write_page_internal_safe: size<=page\n");
        ret = write_less_than_page_safe(offset, buf, size);
    } else {
        // size over a page
        // first page
        uint32_t first_page_begin = offset&0xffff00;
//        uint16_t size_first = 0xff-(offset&0xff);
        uint32_t size_first = 0xff-((offset-first_page_begin)&0xff);
        uint32_t last_page_begin = (offset+size)&0xffff00;
        uint16_t size_last = (offset+size-last_page_begin)&0xff;
        if (size_first != 0) ret = write_less_than_page_safe(offset, buf, size_first);
        // middle page(s)
        uint16_t middle_page_num = (last_page_begin-first_page_begin)/0xff-1;
        for (uint16_t i = 0; i < middle_page_num; i++) {
            uint32_t now = size_first + i*0xff + 1;
//          printf("write_page_internal_safe: middle: now: 0x%x\n", now);
            ret = write_page_safe(offset+now, &buf[now]);
        }
        // last page
        if (size_last != 0) ret = write_less_than_page_safe(last_page_begin, &buf[size-size_last], size_last);
        printf("write_page_internal_safe: middle_page_num: %d, ret: %s\n",
                 middle_page_num, ret?"ok":"ng");
    }
    return ret;
}

bool Logging::flashMutex_init() {
    flashMutex = xSemaphoreCreateMutex();
    return (flashMutex != NULL);
}

bool Logging::flashMutex_lock() {
    return (xSemaphoreTake(flashMutex, (TickType_t)20) == pdTRUE);
}

bool Logging::flashMutex_unlock() {
    return (xSemaphoreGive(flashMutex) == pdTRUE);
}

void Logging::erase_sector_unsafe(void *p) {
    struct flash_ew_t* fp = (struct flash_ew_t*)p;
    flash_range_erase(fp->offset, fp->size); // FLASH_SECTOR_SIZE == 4096Byte defined in flash.h
}

void Logging::write_page_unsafe(void *p) {
    struct flash_ew_t* fp = (struct flash_ew_t*)p;
    flash_range_program(fp->offset, fp->buf, FLASH_PAGE_SIZE);
}

bool Logging::erase_sector_safe(const uint32_t offset, uint32_t size) {
    //printf("erase_sector_safe: offset: 0x%02x, size: 0x%02x\n", offset, size);
    struct flash_ew_t f = {
        .offset = offset,
        .size = size
    };
    if (flashMutex_lock()) {
        int8_t ret = flash_safe_execute(Logging::erase_sector_unsafe, (void*)&f, UINT32_MAX);
//      printf("locked, ret: %d ", ret);
        return (flashMutex_unlock() && (PICO_OK == ret));
    }
    return false;
}

bool Logging::write_page_safe(const uint32_t offset, uint8_t* buf) {
    struct flash_ew_t f = {
        .offset = offset,
        .buf    = buf
    };
    printf("write_page_safe: offset: 0x%x\n", f.offset);
    printf("0x");
    for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) printf("%02x", buf[i]);
    printf("\n");
    if (flashMutex_lock()) {
        int8_t ret = flash_safe_execute(Logging::write_page_unsafe, (void*)&f, UINT32_MAX);
        //printf("locked, ret: %d ", ret);
        return (flashMutex_unlock() && (PICO_OK == ret));
    }
    return false;
}

bool Logging::read_page(const uint32_t offset, uint8_t* buf, uint32_t size) {
    bool ret = false;
    printf("read_page: offset: 0x%x, size: %d\n", offset, size);
    if (flashMutex_lock()) {
        const uint8_t *p = (const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + offset);
        memcpy((void*)buf, (void*)p, size);
        ret = flashMutex_unlock();
    }
    printf("0x");
    for (uint16_t i = 0; i < size; i++) printf("%02x", buf[i]);
    printf("\n");
    return ret;
}

bool Logging::save_fs_periodically() {
    bool ret = false;
    static uint8_t write_cnt_from_last_save_fs = 0;
    if (write_cnt_from_last_save_fs > PERIOD_SAVE_FS) {
        write_cnt_from_last_save_fs = 0;
        ret = save_fs_force();
    } else {
        write_cnt_from_last_save_fs++;
    }
    return ret;
}

void Logging::show_files(uint8_t from, uint8_t size) {
    for (uint8_t i = from; i < from+size; i++) {
        printf("file: i: %d, name: %s, head: %d, size: %d, offset: 0x%02x\n",
               i, fsdata_get_name(i),
               fsdata_head[i],
               fsdata_max_size[i],
               fsdata_offset[i]
               );
    }
}
