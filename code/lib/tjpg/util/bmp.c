// 渡部広一『画像情報処理』より改変
// https://www.kyoritsu-pub.co.jp/book/b10008104.html
#ifndef BMP_C
#define BMP_C
/* 
unsigned char型配列に格納された
24ビット-ビットマップデータをBMPファイルに出力する
[注意] 画像の横幅は４の倍数であること！
引数：
 unsigned char image[Y_SIZE][X_SIZE][3] 24ビットRGB画像配列
 char *filename BMPファイル名
*/
void writeBmp(unsigned char image[Y_SIZE][X_SIZE][3], char *filename)
{
	FILE *fp;
	int i, j, k;
	
	/* ファイルオープン */
	if ((fp = fopen(filename, "wb"))==NULL) {
		printf("writeBmp: Open error!\n");
		exit(1);
	}
	printf("output file : %s\n", filename);

	/* ヘッダー情報 */
	fwrite(&bfType, sizeof(bfType), 1, fp);
	fwrite(&bfSize, sizeof(bfSize), 1, fp);
	fwrite(&bfReserved1, sizeof(bfReserved1), 1, fp);
	fwrite(&bfReserved2, sizeof(bfReserved2), 1, fp);
	fwrite(&bfOffBits, sizeof(bfOffBits), 1, fp);

	fwrite(&biSize, sizeof(biSize), 1, fp);
	fwrite(&biWidth, sizeof(biWidth), 1, fp);
	fwrite(&biHeight, sizeof(biHeight), 1, fp);
	fwrite(&biPlanes, sizeof(biPlanes), 1, fp);
	fwrite(&biBitCount, sizeof(biBitCount), 1, fp);
	fwrite(&biCompression, sizeof(biCompression), 1, fp);
	fwrite(&biSizeImage, sizeof(biSizeImage), 1, fp);
	fwrite(&biXPelsPerMeter, sizeof(biXPelsPerMeter), 1, fp);
	fwrite(&biYPelsPerMeter, sizeof(biYPelsPerMeter), 1, fp);
	fwrite(&biClrUsed, sizeof(biClrUsed), 1, fp);
	fwrite(&biClrImportant, sizeof(biClrImportant), 1, fp);

	/* ビットマップデータ */
	for (i=0; i<biHeight; i++)
	for (j=0; j<biWidth; j++) {
		for (k=0; k<3; k++) {
			fwrite(&image[biHeight-1-i][j][2-k], 1, 1, fp);
		}
	}
	
	fclose(fp);
}



/* 
24ビット-ビットマップデータ(BMPファイル)を
読み込み unsigned char型配列に格納する．
[注意] 画像の横幅は４の倍数であること！
引数：
 char *filename BMPファイル名
 unsigned char image[Y_SIZE][X_SIZE][3] 24ビットRGB画像配列
*/
void readBmpHeader(char *filename, unsigned char image[Y_SIZE][X_SIZE][3])
{
	FILE *fp;
	int i, j, k;
	
	/* ファイルオープン */
	if ((fp = fopen(filename, "rb"))==NULL) {
		printf("readBmpHeader: Open error!\n");
		exit(1);
	}
	printf("input file : %s\n", filename);

	/* ヘッダー情報 */
	fread(&bfType, sizeof(bfType), 1, fp);
	fread(&bfSize, sizeof(bfSize), 1, fp);
	fread(&bfReserved1, sizeof(bfReserved1), 1, fp);
	fread(&bfReserved2, sizeof(bfReserved2), 1, fp);
	fread(&bfOffBits, sizeof(bfOffBits), 1, fp);

	fread(&biSize, sizeof(biSize), 1, fp);
	fread(&biWidth, sizeof(biWidth), 1, fp);
	fread(&biHeight, sizeof(biHeight), 1, fp);
	fread(&biPlanes, sizeof(biPlanes), 1, fp);
	fread(&biBitCount, sizeof(biBitCount), 1, fp);
	fread(&biCompression, sizeof(biCompression), 1, fp);
	fread(&biSizeImage, sizeof(biSizeImage), 1, fp);
	fread(&biXPelsPerMeter, sizeof(biXPelsPerMeter), 1, fp);
	fread(&biYPelsPerMeter, sizeof(biYPelsPerMeter), 1, fp);
	fread(&biClrUsed, sizeof(biClrUsed), 1, fp);
	fread(&biClrImportant, sizeof(biClrImportant), 1, fp);

	/* ビットマップデータ */
//	for (i=0; i<biHeight; i++)
//	for (j=0; j<biWidth; j++) {
//		for (k=0; k<3; k++) {
//			fread(&image[biHeight-1-i][j][2-k], 1, 1, fp);
//		}
//	}
	
	fclose(fp);
}


void readBmpData(char *filename, unsigned char image[Y_SIZE][X_SIZE][3]) {
	FILE *fp;
	int i, j, k;
	
	/* ファイルオープン */
	if ((fp = fopen(filename, "rb"))==NULL) {
		printf("readBmpData: Open error!\n");
		exit(1);
	}

	printf("input file : %s\n", filename);

	for (i=0; i<biHeight; i++)
	for (j=0; j<biWidth; j++) {
		for (k=0; k<3; k++) {
			fread(&image[biHeight-1-i][j][2-k], 1, 1, fp);
		}
	}
    fclose(fp);
}

/* 256諧調白黒濃淡画像へ変換 */
void to256BW(
		unsigned char image[Y_SIZE][X_SIZE][3],
		unsigned char image_bw[Y_SIZE][X_SIZE]  )
{
	int y, x, a;
	for (y=0; y<biHeight; y++)
	for (x=0; x<biWidth; x++) {
		a = (int)(0.3*image[y][x][0] + 0.59*image[y][x][1] + 0.11*image[y][x][2]);
		if (a<LOW) a = LOW;
		if (a>HIGH) a = HIGH;
		image_bw[y][x] = a;
	}
}

/* 24ビットBMP形式に変換 */
void to24BMP(
		unsigned char image_bw[Y_SIZE][X_SIZE],
		unsigned char image[Y_SIZE][X_SIZE][3] )
{
	int y, x, a;
	for (y=0; y<biHeight; y++)
	for (x=0; x<biWidth; x++) {
		a = image_bw[y][x];
		image[y][x][0] = a;
		image[y][x][1] = a;
		image[y][x][2] = a;
	}
}

/* 白黒濃淡画像配列のコピー */
void imageCopyBW(
		unsigned char image1[Y_SIZE][X_SIZE],
		unsigned char image2[Y_SIZE][X_SIZE] )
{
	int x, y;
	for (y=0; y<biHeight; y++)
	for (x=0; x<biWidth; x++)
		image2[y][x] = image1[y][x];
}
#endif // BMP_C
