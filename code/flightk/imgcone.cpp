
// 'Red' in HSV
// Hue
#define H_MIN_1 0 // 固定
#define H_MAX_1 60
#define H_MIN_2 300
#define H_MAX_2 360 // 固定
// Satuation
#define S_MIN 10
#define S_MAX 100
// Value
#define V_MIN 15
#define V_MAX 90

// util funcs
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

uint32_t red_area[15] = {0};

int IMGCONE::my_out_func(JDEC *jdec, void *bitmap, JRECT *rect)
{
    // ref:https://qiita.com/yosihisa/items/c326e59ca5d65f35a181
    uint8_t *src;
    uint16_t x, y; //, bws;
    uint8_t r, g, b;
    double h = 0, s, v;
    uint32_t bitshift = 0;


    //    printf("\n");
    src = (uint8_t *)bitmap;
    //    bws = N_BPP * (rect->right - rect->left + 1);
    for (y = 0; y < (rect->bottom - rect->top + 1); y++)
    {
        for (x = 0; x < (rect->right - rect->left + 1); x += 1)
        {
            //            printf("y: %d\n", y);
            r = *(src + 3 * (y * (rect->right - rect->left + 1) + x));
            g = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 1);
            b = *(src + 3 * (y * (rect->right - rect->left + 1) + x) + 2);
            //            printf("(%03d,%03d,%03d),", r,g,b);
            double MAX = max((max(r, g)), b);
            double MIN = min((min(r, g)), b);
            v = MAX / 256 * 100;

            if (MAX == MIN)
            {
                h = 0;
                s = 0;
            }
            else
            {
                if (MAX == r)
                    h = 60.0 * (g - b) / (MAX - MIN) + 0;
                else if (MAX == g)
                    h = 60.0 * (b - r) / (MAX - MIN) + 120.0;
                else if (MAX == b)
                    h = 60.0 * (r - g) / (MAX - MIN) + 240.0;

                if (h > 360.0)
                    h = h - 360.0;
                else if (h < 0)
                    h = h + 360.0;
                s = (MAX - MIN) / MAX * 100.0;
            }

            if (h > 360.0)
                h -= 360;

            // 赤色の判定
            if ((h >= H_MIN_1 && h <= H_MAX_1) || (h >= H_MIN_2 && h <= H_MAX_2))
            {
                if ((s >= S_MIN && s <= S_MAX) && (v >= V_MIN && v <= V_MAX))
                {
                    //                    printf("red!: (rx,ry)=(%d,%d)\n", rect->left+x, rect->top+y);
                    bitshift = (rect->left + x);
                    red_area[rect->top + y] |= (1 << bitshift);
                }
            }
        }
    }

    return 1; // Continue to decompress
}

bool IMGCONE::init() {
    Cam::init();
    Cam::enableJpeg();

}

IMGCONE_RES_T IMGCONE::capture() {
    Cam::capture();
    sleep_ms(20);
    uint32_t size = cam::getJpegSize();
    memset(red_area, 0, sizeof(red_area));
    JRESULT res = TJPGD:::prepare(cam::image_buf, size);
    if (res == JDR_OK)
    {
        printf("Image size is %u x %u.\n%u bytes of work area is free.\n", TJPGD::jdec.width, TJPGD::jdec.height, TJPGD::jdec.sz_pool);
        res = TJPGD:::decomp(IMGCONE::my_out_func, 3);
        // 160x120 -> (1/2^3) -> 20x15
        if (res == JDR_OK)
        {
            printf("\rDecompression succeeded.\n");
            // print red_area (20x15)
            uint8_t bit;
            for (uint8_t y = 0; y < 15; y++)
            {
                //                   std::cout << std::bitset<32>(red_area[y]) << std::endl;
                for (int8_t i = 20 - 1; i >= 0; i--)
                {
                    bit = (red_area[y] >> i) & 1;
                    printf("%u", bit);
                }
                printf("\n");
            }

            count_vert(vert, red_area);
            printf("count: %d %d %d %d %d\n", vert[4], vert[3], vert[2], vert[1], vert[0]);
        }
        else
    {
            printf("jd_decomp() failed (rc=%d)\n", res);
        }
    }
    else
{
        printf("jd_prepare() failed (rc=%d)\n", res);
    }

    int8_t dire = 0;
    int8_t most = vert[0];

    for (int8_t q = 0; q < 5; q++)
    {
        if (vert[q] > most)
        {
            most=vert[q];
            dire = q;
        }
    }

    switch (dire)
    {
        case 0:
            if (vert[0] == 0) {
                return IC_NO_GOAL;
            } else {
                return IC_RIGHT;
            }
            break;
        case 1:
            return IC_SRIGHT;
            break;
        case 2:
            return IC_FORWARD;
            break;
        case 3:
            return IC_SLEFT;
            break;
        case 4:
            return IC_LEFT;
            break;
    }
}

void IMGCONE::count_vert(uint8_t vert[5], uint32_t area[15])
{
    // count ones in vertically devided area
    // [4] [3] [2] [1] [0]
    // max count in each area: 4*15=60
    // 左右・上下反転に注意
    memset(vert, 0, 5);
    uint8_t bit;
    for (uint8_t y = 0; y < 15; y++)
    {
        for (int8_t x = 20 - 1; x >= 0; x--)
        {
            bit = (area[y] >> x) & 1;
            if (bit == 1)
            {
                if ((x >= 0) && (x < 4))
                    vert[0]++;
                else if (x < 8)
                    vert[1]++;
                else if (x < 12)
                    vert[2]++;
                else if (x < 16)
                    vert[3]++;
                else
                    vert[4]++;
            }
        }
    }
}
