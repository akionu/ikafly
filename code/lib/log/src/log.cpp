#include "log.hpp"

// public:
bool Logging::init() {
    this->file_num_now = 0;
    this->file_list_head = 0;
    return (this->flashMutex_init());
}

bool Logging::make(uint8_t* file_num, const char name[8], const uint32_t max_size) {
    if (this->file_list_head + 1 > MAX_FILE) return false; // no more file
    memcpy(this->file_list[this->file_list_head].name, name, strlen(name));
    this->file_list[this->file_list_head].head = 0;
    this->file_list[this->file_list_head].size = max_size;
    this->file_list[this->file_list_head].erase = false;
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
    printf("make: this->file_list_head: %d, name: %s, head: %d, size: %d, erase: %d, offset: 0x%x, file_num: %d\n",
           this->file_list_head, this->file_list[this->file_list_head].name,
           this->file_list[this->file_list_head].head,
           this->file_list[this->file_list_head].size,
           this->file_list[this->file_list_head].erase,
           this->file_list[this->file_list_head].offset,
           *file_num
           );
    this->file_list_head += 1;
//    this->file_num_now = this->file_list_head;
    return true;
}

bool Logging::open(const uint8_t file_num){
    if (file_num > MAX_FILE || this->file_list[file_num].size == 0) return false;
    printf("open file: %d, %s\n", file_num, this->file_list[file_num].name);
    this->file_num_now = file_num;
    return true;
}

bool Logging::read(const uint8_t file_num, const void* buf, const uint32_t size) {
    uint8_t* pbuf = (uint8_t*)buf;
    if (this->file_list[file_num].erase) {
        for (uint32_t i = 0; i < size; i++) pbuf[i] = 0;
        return false;
    } else {
        this->read_page(this->file_list[file_num].offset, pbuf, size);
    }
    return true;
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
        ret = this->write_page_safe(begin, this->tmp);
    }
    return ret;
}

bool Logging::write(const uint8_t file_num, const void* buf, const uint32_t size) {
    uint8_t* pbuf = (uint8_t*)buf;
    uint32_t offset = this->file_list[file_num].offset;
    uint32_t begin = offset&0xffff00; // the beginning of the PAGE (FLASH_PAGE_SIZE should be 0xff)
    uint32_t size_from_page_begin = (offset-begin+size); // size from the beginning of the PAGE
    printf("offset: 0x%x==%d, begin: 0x%x, size_from_page_begin: 0x%x==%d\n", offset, offset, begin, 
           size_from_page_begin, size_from_page_begin);
    if (size_from_page_begin <= FLASH_PAGE_SIZE) {
        write_less_than_page_safe(offset, pbuf, size);
    } else {
        // size_from_begin > FLASH_PAGE_SIZE
        uint32_t begin_last_page = (offset+size)&0xffff00;
        uint16_t middle_page_num = (begin_last_page - begin)/0xff -1;
        uint32_t s1 = (0xff-((offset-begin)&0xff));
        uint32_t s2 = (((offset+size-begin_last_page)&0xff));
        printf("begin_last_page: 0x%x==%d, middle_page_num: %d\n", begin_last_page, begin_last_page, middle_page_num);
        printf("s1: 0x%x==%d, s2: 0x%x==%d\n", s1, s1, s2, s2);
        // first page
        this->write_less_than_page_safe(offset, pbuf, s1);
        // middle pages
        for (uint8_t i = 0; i < middle_page_num; i++) {
            uint32_t now = s1 + 0xff * i + 1; 
            uint32_t offset_now = offset + now;
            printf("offset_now: 0x%x==%d, offset_now-begin: 0x%x==%d\n", offset_now, offset_now, offset_now-begin, offset_now-begin);
            this->write_page_safe(offset_now, &pbuf[now]);
        }
        // last page
        this->write_less_than_page_safe(begin_last_page, &pbuf[size-s2], s2);
    }
    return true;
}

bool Logging::append(const uint8_t file_num, const void* buf, const uint32_t size){
    return true; 
}

bool Logging::erase(const uint8_t file_num, const uint32_t size) {
    return true;
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
	uint32_t* offset = (uint32_t*)p;
	flash_range_erase(*offset, FLASH_SECTOR_SIZE); // FLASH_SECTOR_SIZE == 4096Byte defined in flash.h
}

void Logging::write_page_unsafe(void *p) {
	struct flash_ew_t* fp = (struct flash_ew_t*)p;
	flash_range_program(fp->offset, fp->buf, FLASH_PAGE_SIZE);
}

bool Logging::erase_sector_safe(const uint32_t offset) {
	if (flashMutex_lock()) {
		int8_t ret = flash_safe_execute(this->erase_sector_unsafe, (void*)&offset, UINT32_MAX);
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

void Logging::read_page(const uint32_t offset, uint8_t* buf, uint32_t size) {
	const uint8_t *p = (const uint8_t *)(XIP_BASE + offset);
	memcpy(buf, p, size);
}
