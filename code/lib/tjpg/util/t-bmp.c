// 渡部広一『画像情報処理』より
// https://www.kyoritsu-pub.co.jp/book/b10008104.html
#include "image.h"
#include "bmp.c"

unsigned char image_in[Y_SIZE][X_SIZE][3];	/* 入力カラー画像配列 */
unsigned char image_out[Y_SIZE][X_SIZE][3];	/* 出力カラー画像配列 */

/* 鏡像を作る(左右逆) */
void mirror(
	unsigned char in[Y_SIZE][X_SIZE][3],
	unsigned char out[Y_SIZE][X_SIZE][3])
{
	int i,j,k;
	for (i=0; i<biHeight; i++)
	for (j=0; j<biWidth; j++)
	for (k=0; k<3; k++)
		out[i][j][k] = in[i][biWidth-1-j][k];
}

void main(void)
{
	char input[100], output[100];

	printf("入力画像ファイル名(input.bmp)："); scanf("%s", input);
	printf("出力画像ファイル名(output.bmp)："); scanf("%s", output);

	/* 画像の入力 */
	readBmp(input, image_in);  /* RGB24ビットカラーBMP画像を配列に格納 */

	/* 画像処理 */
	mirror(image_in, image_out);	/* 鏡像を作る */

	/* 画像の出力 */
	writeBmp(image_out, output);  /* RGB24ビット画像をファイルに出力 */
}
