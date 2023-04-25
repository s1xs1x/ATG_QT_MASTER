#ifndef PLATFORM_TYPEDEF_H
#define PLATFORM_TYPEDEF_H


typedef unsigned char       uint8;

typedef unsigned short      uint16;

typedef unsigned long       uint32;

typedef signed char         int8;                                               // 有符号  8 bits
typedef signed short int    int16;                                              // 有符号 16 bits
typedef signed int          int32;                                              // 有符号 32 bits
typedef signed long long    int64;                                              // 有符号 64 bits

#define MT9V03X_W                (188)
#define MT9V03X_H                (120)

#define AT                  AT_IMAGE
#define AT_IMAGE(img, x, y)          ((img)->data[(y)*(img)->step+(x)]) /**< ����ͼƬ��X,Yλ�õ�Ԫ��*/
#define AT_IMAGE_CLIP(img, x, y)     AT_IMAGE(img, clip(x, 0, (img)->width-1), clip(y, 0, (img)->height-1))
#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w, .height=h, .step=w}
#define ROI_IMAGE(img, x1, y1, w, h) {.data=&AT_IMAGE(img, x1, y1), .width=w, .height=h, .step=img.width}

#define LINE_LENTH  (100)

#endif
