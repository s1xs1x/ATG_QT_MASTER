#ifndef ATG_IMAGE_H
#define ATG_IMAGE_H

#include "math.h"
#include "Platform_typedef.h"

//class ATG_Image : public QWidget
//{
//public:
//    explicit ATG_Image(QWidget *parent = nullptr);

//signals:

//};

typedef struct image {
    uint8 *data; /**< 图锟斤拷锟斤拷锟斤拷*/
    uint32 width;
    uint32 height;
    uint32 step;  /**< 锟斤拷锟斤拷之锟戒步锟斤拷*/
} image_t;
extern int ipts0[LINE_LENTH][2];/**< 原图左边线*/
extern int ipts1[LINE_LENTH][2];/**< 原图右边线*/
extern int ipts0_num, ipts1_num;

extern float rpts0[LINE_LENTH][2];/**< 透视变换后左边线*/
extern float rpts1[LINE_LENTH][2];/**< 透视变换后右边线*/
extern int rpts0_num, rpts1_num;

extern float inv_rptsc0[LINE_LENTH][2];/**< ������ 0x1y*/
extern float inv_rptsc1[LINE_LENTH][2];/**< ������ 0x1y*/

extern int16 line_blur_kernel;
extern float rpts0b[LINE_LENTH][2];/**< 左边线滤波*/
extern float rpts1b[LINE_LENTH][2];/**< 右边线滤波*/
extern int rpts0b_num, rpts1b_num;

extern float rpts0s[LINE_LENTH][2];/**< 左边线等距采样*/
extern float rpts1s[LINE_LENTH][2];/**< 右边线等距采样*/
extern int rpts0s_num, rpts1s_num;


extern float rpts0a[LINE_LENTH];/**< 左边线局部角度变化率*/
extern float rpts1a[LINE_LENTH];/**< 右边线局部角度变化率*/
extern int rpts0a_num, rpts1a_num;

extern float rpts0an[LINE_LENTH];/**< 非极大抑制后的左边线局部角度变化率*/
extern float rpts1an[LINE_LENTH];/**< 非极大抑制后的右边线局部角度变化率*/
extern int rpts0an_num, rpts1an_num;


extern float rptsc0[LINE_LENTH][2];/**< 透视变换后左中线*/
extern float rptsc1[LINE_LENTH][2];/**< 透视变换后右中线*/
extern int rptsc0_num, rptsc1_num;


void process_image();
void thres_hold_255(uint8 *img_data, uint8 *output_data, int width, int height, int thres);
void adaptiveThreshold(uint8* img_data, uint8* output_data, int width, int height, int block, uint8 clip_value);
uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height);
#endif
