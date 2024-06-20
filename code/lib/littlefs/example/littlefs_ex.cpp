#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "littlefs.h"

lfs_t lfs;
lfs_file_t file;

void task_lfs(void *) {
	printf("task_lfs\n");
	int8_t err = lfs_mount(&lfs, &lfs_pico_cfg);

	if (err) {
		printf("err: lfs_mount");
		lfs_format(&lfs, &lfs_pico_cfg);
	}
	uint32_t boot_count = 0;
	lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

	printf("ok: lfs_file_read");
    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);

    // release any resources we were using
    lfs_unmount(&lfs);

    // print the boot count
    printf("boot_count: %d\n", boot_count);

	while (1) vTaskDelay(1000);
}

int main(void) {
	stdio_init_all();
	sleep_ms(2000);
	printf("%s\n", __FILE_NAME__);
	
	xTaskCreate(task_lfs,"task_lfs",256,NULL,1,NULL);
    vTaskStartScheduler();
	while (1) {}
}
