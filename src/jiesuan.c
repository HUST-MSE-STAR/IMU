/*
 *jiesuan.c
 *核心文件
 *
 *Change Logs:
 *Date           Author        Notes                 mail
 *2020.5.29      ts-199        first version         1428381962@qq.com
 *
 *
 */

#include "jiesuan.h"
#include "math.h"
/*****解所需要的航向角fai*****/
void accel_jiesuan_fai(float ax,float ay,float az,float mx,float my,float mz)
{
r_a=atan(ax/az);
p_a=-asin(ay/g);
y_m=atan((mx*self_sin(r_a)*self_sin(p_a)+mz*self_cos(r_a)*self_sin(p_a)+my*self_cos(p_a))/(mx*self_cos(r_a)-mz*self_sin(r_a)));

}


/***自己的sin函数    利用表存储起来，减少实际运行时的计算时间***/
float self_sin(float x)
{
int m=0;
float k=0,yu=0;
m=x/delta;
k=(s_sin[m+1]-s_sin[m])/T;
yu=x-m*delta;	
return s_sin[m]+k*yu;

}

/***自己的cos函数 ***/
float self_cos(float x)
{
int m=0;
float k=0,yu=0;
m=x/delta;
k=(s_cos[m+1]-s_cos[m])/T;
yu=x-m*delta;	
return s_cos[m]+k*yu;

}

/***核心解算的函数  具体理解参照那篇论文***/
void update(float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz)
{
norm1=inv_sqrt(ax*ax+ay*ay+az*az);
norm2=inv_sqrt(gx*gx+gy*gy+gz*gz);
norm3=inv_sqrt(mx*mx+my*my+mz*mz);

ax=ax/norm1;
ay=ay/norm1;
az=az/norm1;
gx=gx/norm2;
gy=gy/norm2;
gz=gz/norm2;
mx=mx/norm3;
my=my/norm3;
mz=mz/norm3;             //标准化

ex1=rzy*az-rzz*ay;
ey1=rzz*ax-rzx*az;
ez1=rzx*ay-rzy*ax;     //加速度计的叉积

fro_fai=inv_sqrt(1.0/(rxx*rxx+ryx*ryx))*(rxx*self_sin(y_m)-ryx*self_cos(y_m));   //磁力计偏差 前面系数

ex2=fro_fai*rzx;
ey2=fro_fai*rzy;
ey2=fro_fai*rzz;      //

gx+=kp1*ex1+ki1*integral_ex1*T+kp2*ex2+ki2*integral_ex2*T;
gy+=kp1*ey1+ki1*integral_ey1*T+kp2*ey2+ki2*integral_ey2*T;
gz+=kp1*ez1+ki1*integral_ez1*T+kp2*ez2+ki2*integral_ez2*T;    //矫正角加速度

exx=rxy*gz*T-gy*T*rxz;
exy=rxz*gx*T-rxx*gz*T;
exz=rxx*gy*T-rxy*gx*T;

eyx=ryy*gz*T-ryz*gy*T;
eyy=ryz*gx*T-ryx*gz*T;
eyz=ryx*gy*T-ryy*gx*T;

ezx=rzy*gz*T-rzz*gy*T;
ezy=rzz*gx*T-rzx*gz*T;
ezz=rzx*gy*T-rzy*gx*T;          //方向余弦矩阵增量

rxx+=exx;
rxy+=exy;
rxz+=exz;

ryx+=eyx;
ryy+=eyy;
ryz+=eyz;

rzx+=ezx;
rzy+=ezy;
rzz+=ezz;                       //更新方向余弦矩阵

}

/***平方根倒数函数***/
float inv_sqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

