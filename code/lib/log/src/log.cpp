#include "log.hpp"

// public:
bool Logging::init() {
    this->file_list_head = 0;
    this->unmount();
    return (this->flashMutex_init());
}

bool Logging::mount() {
    read_page(offset_base_internal, (void*)tmp, 4);
    printf("mount: %c%c%c%c\n", tmp[0], tmp[1], tmp[2], tmp[3]);
    if (tmp[0] == 'p' && tmp[1] == 'p' && tmp[2] == 'f' && tmp[3] == 's') {
        read_page(offset_base_internal+4, (void*)file_list, sizeof(file_list));
    }
    else return false;
    return true;
}

void Logging::unmount() {
    for (uint8_t i = 0; i < MAX_FILE; i++) {
        memcpy(file_list[i].name, 0, 8);
        file_list[i].offset = 0;
        file_list[i].head = 0;
        file_list[i].size = 0;
    }
}

bool Logging::make(uint8_t* file_num, const char name[8], const uint32_t max_size) {
    if (this->file_list_head + 1 > MAX_FILE) return false; // no more file
    memcpy(this->file_list[this->file_list_head].name, name, strlen(name));
    this->file_list[this->file_list_head].head = 0;
    this->file_list[this->file_list_head].size = max_size;
    if (this->file_list_head > 0) {
        this->file_list[this->file_list_head].offset = 
            this->file_list[this->file_list_head-1].offset
            + this->file_list[this->file_list_head-1].size;
        if (this->file_list[this->file_list_head].offset > PICO_FLASH_SIZE_BYTES) return false;
    } else {
        // this->file_list_head == 0
        this->file_list[this->file_list_head].offset = offset_base;
    }
    *file_num = this->file_list_head;
    printf("make: this->file_list_head: %d, name: %s, head: %d, size: %d, offset: 0x%x==%d, file_num: %d\n",
           this->file_list_head, this->file_list[this->file_list_head].name,
           this->file_list[this->file_list_head].head,
           this->file_list[this->file_list_head].size,
           this->file_list[this->file_list_head].offset,
           this->file_list[this->file_list_head].offset,
           *file_num
           );
    this->file_list_head++;
    write_fs_data();
    return true;
}

bool Logging::find(uint8_t* file_num, const char query_name[8], const uint8_t size_query_name) {
    bool ret = false;
    for (uint8_t i = 0; i < MAX_FILE; i++) {
        if (memcmp(file_list[i].name, query_name, size_query_name)) {
            *file_num = i;
            ret = true;
            // find head
            uint8_t head = 0;
            for (uint8_t j = 0; j < MAX_FILE; j++) {
                if (file_list[j].size != 0) head++;
            }
            file_list_head = head;
            printf("file: i: %d, name: %s, head: %d, size: %d, offset: 0x%x==%d, file_num: %d\n",
               i, this->file_list[i].name,
               this->file_list[i].head,
               this->file_list[i].size,
               this->file_list[i].offset,
               this->file_list[i].offset,
               *file_num
            );
        }
    }
    return ret;
}

bool Logging::read(const uint8_t file_num, const void* buf, const uint32_t size) {
    uint8_t* pbuf = (uint8_t*)buf;
    if ((this->file_list[file_num].head < size)
        || (this->file_list[file_num].size < size)) {
        //        for (uint32_t i = 0; i < size; i++) pbuf[i] = 0;
        printf("read: err: head: 0x%x==%d, size: 0x%x==%x\n", 
               this->file_list[file_num].head, this->file_list[file_num].head,
               size, size);
        return false;
    } else {
        this->read_page(this->file_list[file_num].offset, pbuf, size);
    }
    return true;
}

bool Logging::write(const uint8_t file_num, const void* buf, const uint32_t size) {
    uint8_t* pbuf = (uint8_t*)buf;
    uint32_t offset = this->file_list[file_num].offset;
    bool ret = false;
    if ((this->file_list[file_num].head) < this->file_list[file_num].size) {
        ret = write_page_internal_safe(offset, pbuf, size);
        if (ret) this->file_list[file_num].head += size;
    } else {
        printf("write: err: head: 0x%x==%d, size: 0x%x==%x\n", 
               this->file_list[file_num].head, this->file_list[file_num].head,
               size, size);
    }
    return ret;
}

bool Logging::append(const uint8_t file_num, const void* buf, const uint32_t size){
    uint8_t* pbuf = (uint8_t*)buf;
    uint32_t offset = this->file_list[file_num].offset+this->file_list[file_num].head;
    bool ret = false;
    if ((this->file_list[file_num].head) < this->file_list[file_num].size) {
        ret = write_page_internal_safe(offset, pbuf, size);
        if (ret) this->file_list[file_num].head += size;
    }
    return ret;
}

void Logging::erase(const uint8_t file_num, const uint32_t size) {
    file_list[file_num].head = 0;
}

bool Logging::nuke() {
    uint32_t size = (PICO_FLASH_SIZE_BYTES - offset_base_internal)/(0x1000);
    printf("nuke: size: 0x%x==%d\n", size, size);
    bool ret = false;
    ret = erase_sector_safe(offset_base_internal, /*size**/FLASH_SECTOR_SIZE);
    return ret;
}

// private:
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
    //    printf("erase_sector_safe: offset: 0x%x, size: 0x%x\n", offset, size);
    struct flash_ew_t f = {
        .offset = offset,
        .size = size
    };
    if (flashMutex_lock()) {
        int8_t ret = flash_safe_execute(this->erase_sector_unsafe, (void*)&f, UINT32_MAX);
        printf("locked, ret: %d ", ret);
        return (flashMutex_unlock() && (PICO_OK == ret));
    }
    return false;
}

bool Logging::write_page_safe(const uint32_t offset, uint8_t* buf) {
    struct flash_ew_t f = {
        .offset = offset,
        .buf    = buf
    };
    if (this->flashMutex_lock()) {
        int8_t ret = flash_safe_execute(write_page_unsafe, (void*)&f, UINT32_MAX);
        //		printf("locked, ret: %d ", ret);
        return (flashMutex_unlock() && (PICO_OK == ret));
    }
    return false;
}

bool Logging::write_less_than_page_safe(const uint32_t offset, uint8_t* buf, uint32_t size){
    bool ret = false;
    uint8_t* pbuf = (uint8_t*)buf;
    uint32_t begin = offset&0xffff00; // the beginning of the PAGE (FLASH_PAGE_SIZE should be 0xff)
    uint32_t size_from_page_begin = (offset-begin+size); // size from the beginning of the PAGE
    printf("wltps: begin: %x, size_from_page_begin: 0x%x==%d, offset-begin: %d\n", begin, size_from_page_begin, 
           size_from_page_begin, offset-begin);
    if (size_from_page_begin <= FLASH_PAGE_SIZE) { // buf size is within one PAGE 
        read_page(begin, this->tmp, FLASH_PAGE_SIZE); // read first not to lose data
        for (uint16_t i = 0; i <= size; i++) this->tmp[offset-begin+i] = pbuf[i];
        for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) printf("%x", tmp[i]);
        printf("\n");
        ret = this->write_page_safe(begin, this->tmp);
        printf("wltps: if: ret: %s\n", ret?"ok":"fail");
    }
    return ret;
}

bool Logging::write_page_internal_safe(const uint32_t offset, uint8_t* buf, uint32_t size) {
    uint32_t begin = offset&0xffff00; // the beginning of the PAGE (FLASH_PAGE_SIZE should be 0xff)
    uint32_t size_from_page_begin = (offset-begin+size); // size from the beginning of the PAGE
    printf("offset: 0x%x==%d, begin: 0x%x, size_from_page_begin: 0x%x==%d\n", offset, offset, begin, 
           size_from_page_begin, size_from_page_begin);
    if (size_from_page_begin <= FLASH_PAGE_SIZE) {
        write_less_than_page_safe(offset, buf, size);
    } else {
        // size_from_begin > FLASH_PAGE_SIZE
        uint32_t begin_last_page = (offset+size)&0xffff00;
        uint16_t middle_page_num = (begin_last_page - begin)/0xff -1;
        uint32_t s1 = (0xff-((offset-begin)&0xff));
        uint32_t s2 = (((offset+size-begin_last_page)&0xff));
        printf("begin_last_page: 0x%x==%d, middle_page_num: %d\n", begin_last_page, begin_last_page, middle_page_num);
        printf("s1: 0x%x==%d, s2: 0x%x==%d\n", s1, s1, s2, s2);
        // first page
        if (s1 != 0) this->write_less_than_page_safe(offset, buf, s1);
        // middle pages
        for (uint8_t i = 0; i < middle_page_num; i++) {
            uint32_t now = s1 + 0xff * i + 1; 
            uint32_t offset_now = offset + now;
            printf("offset_now: 0x%x==%d, offset_now-begin: 0x%x==%d\n", offset_now, offset_now, offset_now-begin, offset_now-begin);
            this->write_page_safe(offset_now, &buf[now]);
        }
        // last page
        if (s2 != 0) this->write_less_than_page_safe(begin_last_page, &buf[size-s2], s2);
    }
    return true;
}

void Logging::read_page(const uint32_t offset, void* buf, uint32_t size) {
    const uint8_t *p = (const uint8_t *)(XIP_BASE + offset);
    memcpy(buf, p, size);
}

bool Logging::write_fs_data() {
    bool ret = false;
    if (is_first_write_fs) {
        uint8_t wbuf[] = "ppfs";
        printf("write_fs_data: write: %c%c%c%c\n", wbuf[0], wbuf[1], wbuf[2], wbuf[3]);
        ret = this->write_page_internal_safe(offset_base_internal, wbuf, 4);
        is_first_write_fs = false;
        // check written "ppfs"
        read_page(offset_base_internal, (void*)wbuf, 4);
        printf("write_fs_data: read: %c%c%c%c\n", wbuf[0], wbuf[1], wbuf[2], wbuf[3]);
    }
    ret = this->write_page_internal_safe(offset_base_internal+4, (uint8_t*)file_list, sizeof(file_list));
    printf("write_fs_data: %s\n", ret?"ok":"fail");
    return ret; 
}
