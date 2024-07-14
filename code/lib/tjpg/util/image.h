// 渡部広一『画像情報処理』より改変
// https://www.kyoritsu-pub.co.jp/book/b10008104.html
#ifndef IMAGE_H
#define IMAGE_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* 最大画像サイズ */
#define Y_SIZE 1280
#define X_SIZE 1280

/* BMPファイル用 */
typedef int16_t INT2;
typedef int32_t INT4;

INT2 bfType;
INT4 bfSize;
INT2 bfReserved1, 
     bfReserved2;
INT4 bfOffBits;
INT4 biSize, 
     biWidth, biHeight;
INT2 biPlanes,
     biBitCount;
INT4 biCompression, 
     biSizeImage,
     biXPelsPerMeter,
     biYPelsPerMeter,
     biClrUsed, 
     biClrImportant;

#define HIGH   255	/* ２値画像の白 */
#define LOW      0	/* ２値画像の黒 */
#define LEVEL  256	/* 濃度レベル数 */

#endif
