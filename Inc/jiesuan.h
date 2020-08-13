/*
 *jiesuan.h - The C head file of the jiesuan.c
 *
 *用途：规定了1、三角函数表的内容；2、磁力计 陀螺仪 加速度计各三个方向的量、差及误差累计；
 *3、kp ki；4、解算的核心函数；5、自己定义的用于计算的函数
 *
 *Note:此算法源自     基于互补滤波器的四旋翼飞行器姿态解算___梁延德，程敏，何福本，李航
 *
 *
 *Change Logs:
 *Date         Author    Notes            mail  
 *2020-5.27    ts-199    first version    1428381962@qq.com
 *2020-5.29    ts-199    修补             1428381962@qq.com
 */
 





#ifndef _hubu_h
#define _hubu_h

/*private defines-------------------------------------------*/
#define  delta  0.0349f               //三角函数表步长（数值是弧度）
#define  T      0.001                 //采样周期（？
#define  g      9.8                     //重力加速度（不知此处准确值，先以9.8）

/* 别名(我习惯这个） */   //ValueX这样的标识是另一个人写的，主要是最开始得到的原始数据，一开始我就用的mx之类，难得改了
#define mx ValueX
#define my ValueY
#define mz ValueZ

#define gx ValueXGyro
#define gy ValueYGyro
#define gz ValueZGyro

#define ax ValueXAccel
#define ay ValueYAccel
#define az ValueZAccel

/*private variables----------------------------------------*/
float exx=0,exy=0,exz=0,eyx=0,eyy=0,eyz=0,ezx=0,ezy=0,ezz=0;
float rxx=1,rxy=1,rxz=1,ryx=1,ryy=1,ryz=1,rzx=1,rzy=1,rzz=1;
float ex1=0,ey1=0,ez1=0,ex2=0,ey2=0,ez2=0;

float p_a=0,r_a=0,y_m=0;     //三个旋转角   加计或磁力计得到的，不是融合

/* 三角函数表*/
float s_sin[46]={0.00f,0.0349f,0.0697f,0.104f,0.139f,0.173f,0.208f,0.242f,0.275f,0.309f,
	0.342f,0.374f,0.407f,0.438f,0.470f,0.500f,0.530f,0.560f,0.587f,0.615f,0.642f,0.670f,0.694f,
	0.720f,0.743f,0.766f,0.788f,0.809f,0.829f,0.848f,0.866f,0.883f,0.898f,0.913f,0.927f,0.939f,0.951f,
	0.961f,0.970f,0.978f,0.984f,0.990f,0.994f,0.997f,0.999f,1.0f};
float s_cos[46]={1.0f,0.999f,0.997f,0.994f,0.990f,0.985f,0.978f,0.970f,0.961f,0.951f,0.940f,0.927f,0.913f,
	0.899f,0.883f,0.866f,0.848f,0.829f,0.809f,0.788f,0.766f,0.743f,0.720f,0.695f,0.670f,0.643f,0.616f,0.588f,
	0.560f,0.530f,0.500f,0.470f,0.438f,0.407f,0.375f,0.342f,0.309f,0.276f,0.242f,0.208f,0.174f,0.105f,0.070f,
	0.035f,0.0f};


float fro_fai=0;   //中间量

float kp1=0.5,kp2=0.5,ki1=0.05,ki2=0.05;   //pi

float norm1=0,norm2=0,norm3=0;      //标准化（加计、磁力计、陀螺仪）   

float integral_ex1=0,integral_ey1=0,integral_ez1=0;         //加速度误差积累
float integral_ex2=0,integral_ey2=0,integral_ez2=0;         //磁力计误差积累

/*private function----------------------------------------*/

/*核心解算的函数*/	
void update(float ax,float ay,float az, float gx,float gy,float gz,float mx,float my,float mz);    //方向余弦更新
void accel_jiesuan_fai(float ax,float ay,float az,float mx,float my,float mz);                     //由磁力计达到航向角


/* 计算要用到的*/	
float inv_sqrt(float x);     //平方根倒数
float self_sin(float x);     //自己的sin函数
float self_cos(float x);     //自己的cos函数


#endif
