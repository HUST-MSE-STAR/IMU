/*
 *jiesuan.h - The C head file of the jiesuan.c
 *
 *��;���涨��1�����Ǻ���������ݣ�2�������� ������ ���ٶȼƸ�������������������ۼƣ�
 *3��kp ki��4������ĺ��ĺ�����5���Լ���������ڼ���ĺ���
 *
 *Note:���㷨Դ��     ���ڻ����˲������������������̬����___���ӵ£��������θ������
 *
 *
 *Change Logs:
 *Date         Author    Notes            mail  
 *2020-5.27    ts-199    first version    1428381962@qq.com
 *
 */
 





#ifndef _hubu_h
#define _hubu_h

/*private defines-------------------------------------------*/
#define  delta  0.0349f               //���Ǻ�����������ֵ�ǻ��ȣ�
#define  T      0.001                 //�������ڣ���
#define  g      9.8                     //�������ٶȣ���֪�˴�׼ȷֵ������9.8��

/* ����(��ϰ������� */
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

float p_a=0,r_a=0,y_m=0;     //������ת��   �Ӽƻ�����Ƶõ��ģ������ں�

/* ���Ǻ�����*/
float s_sin[46]={0.00f,0.0349f,0.0697f,0.104f,0.139f,0.173f,0.208f,0.242f,0.275f,0.309f,
	0.342f,0.374f,0.407f,0.438f,0.470f,0.500f,0.530f,0.560f,0.587f,0.615f,0.642f,0.670f,0.694f,
	0.720f,0.743f,0.766f,0.788f,0.809f,0.829f,0.848f,0.866f,0.883f,0.898f,0.913f,0.927f,0.939f,0.951f,
	0.961f,0.970f,0.978f,0.984f,0.990f,0.994f,0.997f,0.999f,1.0f};
float s_cos[46]={1.0f,0.999f,0.997f,0.994f,0.990f,0.985f,0.978f,0.970f,0.961f,0.951f,0.940f,0.927f,0.913f,
	0.899f,0.883f,0.866f,0.848f,0.829f,0.809f,0.788f,0.766f,0.743f,0.720f,0.695f,0.670f,0.643f,0.616f,0.588f,
	0.560f,0.530f,0.500f,0.470f,0.438f,0.407f,0.375f,0.342f,0.309f,0.276f,0.242f,0.208f,0.174f,0.105f,0.070f,
	0.035f,0.0f};


float fro_fai=0;   //�м���

float kp1=0.5,kp2=0.5,ki1=0.05,ki2=0.05;   //pi

float norm1=0,norm2=0,norm3=0;      //��׼�����Ӽơ������ơ������ǣ�   

float integral_ex1=0,integral_ey1=0,integral_ez1=0;         //���ٶ�������
float integral_ex2=0,integral_ey2=0,integral_ez2=0;         //������������

/*private function----------------------------------------*/

/*���Ľ���ĺ���*/	
void update(float ax,float ay,float az, float gx,float gy,float gz,float mx,float my,float mz);    //�������Ҹ���
void accel_jiesuan_fai(float ax,float ay,float az,float mx,float my,float mz);                     //�ɴ����ƴﵽ�����


/* ����Ҫ�õ���*/	
float inv_sqrt(float x);     //
float self_sin(float x);     //�Լ���sin����
float self_cos(float x);     //�Լ���cos����


#endif