#include "image.h"
#include "bmp.c"

unsigned char image_in[Y_SIZE][X_SIZE][3];	/* 入力カラー画像配列 */
//unsigned char image_out[Y_SIZE][X_SIZE][3];	/* 出力カラー画像配列 */

void main(void) {
	char input[100], output[100];

	printf("入力RGB生画像ファイル名(input.bin)："); scanf("%s", input);
	printf("出力画像ファイル名(output.bmp)："); scanf("%s", output);

	/* 画像の入力 */
	readBmpHeader("test.bmp", image_in);
    readBmpData(input, image_in);

	/* 画像の出力 */
	writeBmp(image_in, output);
}
