#include "atg_image.h"
#include "utils.h"
#include "flash.h"
#include "mainWindow.h"
const int dir_front[4][2] = {{0,  -1},
                             {1,  0},
                             {0,  1},
                             {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1},
                                 {1,  -1},
                                 {1,  1},
                                 {-1, 1}};
const int dir_frontright[4][2] = {{1,  -1},
                                  {1,  1},
                                  {-1, 1},
                                  {-1, -1}};



float rot[3][3] =
    {{12.7746 , 0.0238,-633.3595},
     { 8.5788  , 1.4568,-461.3387},
     {  0.0913  ,0.0004  ,-3.5295}};

float inv_rot[3][3]=
    {{-0.2698  ,-0.0085 ,49.5329},
     {-0.6421  ,0.6904 ,24.9915},
     {-0.0070   ,-0.0001 ,1.0000}};

//int x0,y0,x1,y1;/**< 用于找边线的起始点*/
int16 line_blur_kernel  =11;
extern uint16 Ostu_Thres;

int pixel_per_meter     =100;
float ROAD_WIDTH        =0.45;
float sample_dist       =0.02;
float angle_dist        =0.20;

int ipts0[LINE_LENTH][2];/**< 原图左边线*/
int ipts1[LINE_LENTH][2];/**< 原图右边线*/
int ipts0_num, ipts1_num;

float rpts0[LINE_LENTH][2];/**< 透视变换后左边线*/
float rpts1[LINE_LENTH][2];/**< 透视变换后右边线*/
int rpts0_num, rpts1_num;

float rpts0b[LINE_LENTH][2];/**< 左边线滤波*/
float rpts1b[LINE_LENTH][2];/**< 右边线滤波*/
int rpts0b_num, rpts1b_num;

float rpts0s[LINE_LENTH][2];/**< 左边线等距采样*/
float rpts1s[LINE_LENTH][2];/**< 右边线等距采样*/
int rpts0s_num, rpts1s_num;

float rpts0a[LINE_LENTH];/**< 左边线局部角度变化率*/
float rpts1a[LINE_LENTH];/**< 右边线局部角度变化率*/
int rpts0a_num, rpts1a_num;

float rpts0an[LINE_LENTH];/**< 非极大抑制后的左边线局部角度变化率*/
float rpts1an[LINE_LENTH];/**< 非极大抑制后的边线局部角度变化率*/
int rpts0an_num, rpts1an_num;

float inv_rptsc0[LINE_LENTH][2];/**< 左中线*/
float inv_rptsc1[LINE_LENTH][2];/**< 右中线*/


float rptsc0[LINE_LENTH][2];/**< 透视变换后左中线*/
float rptsc1[LINE_LENTH][2];/**< 透视变换后右中线*/
int rptsc0_num, rptsc1_num;


// L角点
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
int inv_Lpt0_rpts0s_id,inv_Lpt1_rpts1s_id;
int Lpt0_found, Lpt1_found;

// 长直道
int is_straight0, is_straight1;

float conf0,conf1,conf0_max,conf1_max;/**< max用来显示在屏幕*/

void thres_hold_255(uint8 *img_data, uint8 *output_data, int width, int height, int thres)
{
    for(int y=0; y<height; y++){
        for(int x=0; x<width; x++){
            output_data[x+y*width] = img_data[x+y*width]>thres ? 255 : 0;
        }
    }
}

uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height)
{
#define GrayScale 256
    int pixelCount[GrayScale] = {0};//每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};//每个灰度值所占总像素比例
    int i,j;
    int Sumpix = width * height;   //总像素点
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针


    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
            //   pixelCount[(int)image[i][j]]++;    若不用指针用这个
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;   //计算每个像素在整幅图像中的比例
        u += i * pixelPro[i];  //总平均灰度
    }

    float maxVariance=0.0;  //最大类间方差
    float w0 = 0, avgValue  = 0;  //w0 前景比例 ，avgValue 前景平均灰度
    for(int i = 0; i < 256; i++)     //每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
    {
        w0 += pixelPro[i];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue  += i * pixelPro[i];

        float variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);    //类间方差
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = i;
        }
    }
    return threshold;

}

void adaptiveThreshold(uint8* img_data, uint8* output_data, int width, int height, int block, uint8 clip_value)
{
    //assert(block % 2 == 1); // block必须为奇数
    int half_block = block / 2;
    // 先遍历y后遍历x比较cache-friendly
    for(int y=half_block; y<height-half_block; y++){
        for(int x=half_block; x<width-half_block; x++){
            // 计算局部阈值
            int thres = 0;
            for(int dy=-half_block; dy<=half_block; dy++){
                for(int dx=-half_block; dx<=half_block; dx++){
                    thres += img_data[(x+dx)+(y+dy)*width];
                }
            }
            thres = thres / (block * block) - clip_value;
            // 进行二值化
            output_data[x+y*width] = img_data[x+y*width]>thres ? 255 : 0;
        }
    }
}

void findline_lefthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) {
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < *num && half < x && x < img->width - half - 1 && 0 < y && y < img->height - half - 1 && turn < 4) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value = AT(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value = AT(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}


void findline_righthand_adaptive(image_t *img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) {
    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < *num && 0 < x && x < img->width -half- 1 && 0 < y && y < img->height - 1 && turn < 4) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += AT(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value = AT(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = AT(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }

    *num = step;
}

void rot_img_process()
{
    for(int i=0;i<ipts0_num;i++)
    {
        rpts0[i][0] = (rot[1][0]*ipts0[i][1]+rot[1][1]*ipts0[i][0]+rot[1][2])/(rot[2][0]*ipts0[i][1]+rot[2][1]*ipts0[i][0]+rot[2][2]);
        rpts0[i][1] = (rot[0][0]*ipts0[i][1]+rot[0][1]*ipts0[i][0]+rot[0][2])/(rot[2][0]*ipts0[i][1]+rot[2][1]*ipts0[i][0]+rot[2][2]);
    }
    for(int i=0;i<ipts1_num;i++)
    {
        rpts1[i][0] = (rot[1][0]*ipts1[i][1]+rot[1][1]*ipts1[i][0]+rot[1][2])/(rot[2][0]*ipts1[i][1]+rot[2][1]*ipts1[i][0]+rot[2][2]);
        rpts1[i][1] = (rot[0][0]*ipts1[i][1]+rot[0][1]*ipts1[i][0]+rot[0][2])/(rot[2][0]*ipts1[i][1]+rot[2][1]*ipts1[i][0]+rot[2][2]);
    }
    rpts0_num = ipts0_num;
    rpts1_num = ipts1_num;

}


void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel){
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - fabs(j));
            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - fabs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}

void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist){
    int remain = 0, len = 0;
    for(int i=0; i<num1-1 && len < *num2; i++){
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        float dn = sqrt(dx*dx+dy*dy);
        dx /= dn;
        dy /= dn;

        while(remain < dn && len < *num2){
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}

void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist){
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

void nms_angle(float angle_in[], int num, float angle_out[], int kernel){
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) {
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;


    }
}

void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist){
    for (int i = 0; i < num; i++) {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}

/******************************************************************************
* FunctionName   : find_boudry()
* Description    : 找第一个跳变点+自适应寻线
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void find_boudry()
{
    int half_adaptive_Block=(adaptive_Block-1)/2;
    int x0,y0,x1,y1;

    //左边线
    for(y0=start_y;y0>end_y;y0--)
    {
        for(x0=MT9V03X_W/2-start_x;x0>half_adaptive_Block+end_x;x0--)
        {
            if(AT_IMAGE(&img_raw, x0-1, y0) < Ostu_Thres)
            {
                goto break1_for_flag;
            }
        }
    }
break1_for_flag:

    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    if (AT_IMAGE(&img_raw, x0, y0) >= Ostu_Thres)
        findline_lefthand_adaptive(&img_raw, adaptive_Block, clip_value, x0, y0, ipts0, &ipts0_num);//只有此处使用了左手巡线新版
    else ipts0_num = 0;

    //右边线
    for(y1=start_y;y1>end_y;y1--)
    {
        for(x1=MT9V03X_W/2+start_x;x1<MT9V03X_W-half_adaptive_Block-end_x;x1++)
        {
            if(AT_IMAGE(&img_raw, x1+1, y1) < Ostu_Thres)
            {
                goto break2_for_flag;
            }
        }
    }

break2_for_flag:

    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    if (AT_IMAGE(&img_raw, x1, y1) >= Ostu_Thres)
        findline_righthand_adaptive(&img_raw, adaptive_Block, clip_value, x1, y1, ipts1, &ipts1_num);//只有此处使用了右手巡线新版
    else ipts1_num = 0;



}
void process_image()
{
    find_boudry();
    //    int x0 = img_raw.width / 2 - begin_x, y0 = begin_y;
    //    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    //    for (; x0 > 0; x0--) if (AT_IMAGE(&img_raw, x0 - 1, y0) < Ostu_Thres) break;
    //    if (AT_IMAGE(&img_raw, x0, y0) >= Ostu_Thres)
    //        findline_lefthand_adaptive(&img_raw, adaptive_Block, clip_value, x0, y0, ipts0, &ipts0_num);
    //    else ipts0_num = 0;

    //    int x1 = img_raw.width / 2 + begin_x, y1 = begin_y;
    //    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    //    for (; x1 < img_raw.width - 1; x1++) if (AT_IMAGE(&img_raw, x1 + 1, y1) < Ostu_Thres) break;
    //    if (AT_IMAGE(&img_raw, x1, y1) >= Ostu_Thres)
    //        findline_righthand_adaptive(&img_raw, adaptive_Block, clip_value, x1, y1, ipts1, &ipts1_num);
    //    else ipts1_num = 0;

    //透视变换
    rot_img_process();

    // 边线滤波
    blur_points(rpts0, rpts0_num, rpts0b, (int) round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int) round(line_blur_kernel));
    rpts1b_num = rpts1_num;

    // 边线等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);

    // 边线局部角度变化率，采用我们的算法可以得到稳定且等距的边线前提下，直接取每个点前后20cm构成一个三点，
    //利用向量的角度计算公式算实际角度即可。为方便判断，
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int) round(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int) round(angle_dist / sample_dist));//angle_dist / sample_dist
    rpts1a_num = rpts1s_num;

    // 角度变化率非极大抑制
    //我们又进行了角度的非极大抑制，保证只留下角度最大的点，即角点。这样算出来的角度很稳定，跟实际也很接近。
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int) round(angle_dist / sample_dist) * 2 + 1);//angle_dist / sample_dist
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int) round(angle_dist / sample_dist) * 2 + 1);//angle_dist / sample_dist
    rpts1an_num = rpts1a_num;

    // 左右中线跟踪
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int) round(10.0), pixel_per_meter * ROAD_WIDTH / 2);//
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int) round(10.0), pixel_per_meter * ROAD_WIDTH / 2);//
    rptsc1_num = rpts1s_num;

}

void find_corners()
{

}





