#include "jiesuan.h"
                                         //����һ��fai������
void update(uint ax,uint ay,uint az, uint gx,uint gy,uint gz,uint mx,uint my,uint mz,uint fai)
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
mz=mz/norm3;             //��׼��

ex1=rzy*az-rzz*ay;
ey1=rzz*ax-rzx*az;
ez1=rzx*ay-rzy*ax;     //���ٶȼƵĲ��

fro_fai=inv_sqrt(1.0/(rxx*rxx+ryx*ryx))*(rxx*sin(fai)-ryx*cos(fai));   //������ƫ�� ǰ��ϵ��

ex2=fro_fai*rzx;
ey2=fro_fai*rzy;
ey2=fro_fai*rzz;      //

gx+=kp1*ex1+ki1*integral_ex1*T+kp2*ex2+ki2*integral_ex2*T;
gy+=kp1*ey1+ki1*integral_ey1*T+kp2*ey2+ki2*integral_ey2*T;
gz+=kp1*ez1+ki1*integral_ez1*T+kp2*ez2+ki2*integral_ez2*T;    //�����Ǽ��ٶ�

exx=rxy*gz*T-gy*T*rxz+rxx;
exy=rxz*gx*T-rxx*gz*T+rxy;
exz=rxx*gy*T-rxy*gx*T+rxz;

eyx=ryy*gz*T-ryz*gy*T+ryx;
eyy=ryz*gx*T-ryx*gz*T+ryy;
eyz=ryx*gy*T-ryy*gx*T+ryz;

ezx=rzy*gz*T-rzz*gy*T+rzx;
ezy=rzz*gx*T-rzx*gz*T+rzy;
ezz=rzx*gy*T-rzy*gx*T+rzz;    //�������Ҿ�������

rxx+=exx;
rxy+=exy;
rxz+=exz;

ryx+=eyx;
ryy+=eyy;
ryz+=eyz;

rzx+=ezx;
rzy+=ezy;
rzz+=ezz;              //���·������Ҿ���

}


float inv_sqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}