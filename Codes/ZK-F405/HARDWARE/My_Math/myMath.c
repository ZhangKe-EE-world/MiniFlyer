/*******************************************************************
 *MPU6050
 *@brief 
 *@brief 
 *@time  2016.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 ******************************************************************/
 
#include "myMath.h"
#include <math.h>
#include "sys.h"

const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
const float Gyro_G = 0.03051756f*2;	  	//陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2		
const float Gyro_Gr = 0.0005326f*2;     //面计算度每秒,转换弧度每秒则 2*0.03051756	 * 0.0174533f = 0.0005326*2
////in  -+500
////out exp -+500
//int16_t Math_AngelEXP(int16_t in){
//	int16_t tmp2 ,tmp,value;
//	tmp = Math_min(Math_abs(in),500); //[0 , +500]
//	tmp2 = tmp/100;
//	value =	Angel_EXP[tmp2] + (tmp-tmp2*100) * (Angel_EXP[tmp2+1]-Angel_EXP[tmp2]) / 100;
//	if(in<0)value = -value;
//	return value;
//}

////油门输入转换。
//int16_t Math_ThrEXP(int16_t RCThr){
//	int16_t tmp2 ,tmp,value;
//	if(RCThr <1000)return RCThr; //低于最小值，直接输出，不做指数变换
//	tmp = Math_Constrain(RCThr,1000,2000);
//  	tmp = (unsigned int)(tmp-1000)*1000/(2000-1000); // [1000;2000] -> [0;1000]
//  	tmp2 = tmp/100;
//  	value = ThrottleEXP[tmp2] + (tmp-tmp2*100) * (ThrottleEXP[tmp2+1]-ThrottleEXP[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
//  	return value;
//}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Q_rsqrt
**功能 : 快速计算 三角函数
**输入 : number  
**输出 : 结果
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
//逼近法；线性拟合
//Q (4/M_PI x - 4/M_PI^2 x^2) + P (4/M_PI x - 4/M_PI^2 x^2)^2 
#ifndef TAPYOR
float sine(float x)          // (-M_PI , M_PI) ???? 0.0005
{
	const float Q = 0.775;
	const float P = 0.225;
	const float B =  4 / M_PI;  
	const float C = -4 /(M_PI*M_PI);
	float y = B * x + C * x * fabs(x); 
	return (Q * y + P * y * fabs(y));
}
#else 
//4级泰勒公式法 在PI出会出现0.7的最大误差
//sinx= x- x^3/3! + x^5/5! - x^7/7!+ x^9/9! . =?(-1)^n x^(2n+1)/(2n+1)!
float sine(float x)
{
	float t=x;
	float result = x;
	float X2 = x*x;
	uint8_t cnt = 1;

	do
	{
		t=-t;
		t *= X2;
		result += t/((cnt<<1)+1);
		cnt++;
	}	while(cnt<5);//6阶

	return result;
} 
#endif
//http://wenku.baidu.com/link?url=jUswZ3G2z26IUS72IkeZrizc5V9VdR1sTF8xGCOHPFW0P70bGjjm5zhNxvRT36X31TMoFf6S-9lMoIkK4pPwExAaEZGtRpWggdQAzpg3Fsu
//cos(x)=sin(M_PI/2+x)=sin(M_PI/2-x)
//cos(x-M_PI/2)=sin(x)
float cosine(float x)
{
	return sine(x+M_PI/2);//奇变偶不变，符号看象限
}

//反正切麦克劳林展开式 阶数越高，值越准确   70°以内是准确的
//http://www.zybang.com/question/246f9997776f7d5cc636b10aff27a1cb.html
float arctan(float x)  //  (-1 , +1)    6? ?? 0.002958 
{
	float t = x;
	float result = 0;
	float X2 = x * x;
	unsigned char cnt = 1;
	do
	{
		result += t / ((cnt << 1) - 1);
		t = -t;
		t *= X2;
		cnt++;
	}while(cnt <= 6);//5??
	return result;
}

//反正弦麦克劳林展开式 -1<x<+1     42°以内是准确的
//http://xuxzmail.blog.163.com/blog/static/25131916200971794014536/
const float PI_2 = 1.570796f;
float arcsin(float x)   //(-1 , +1)  ? 0 ????  6? ??0.005
{
	float d=1;
	float t=x;
	unsigned char cnt = 1;
	float result = 0;	
	float X2 = x*x;
	
	if (x >= 1.0f) 
	{
		return PI_2;
	}
	if (x <= -1.0f) 
	{
		return -PI_2;
	}
	do
	{
		result += t / (d * ((cnt << 1) - 1));
		t *= X2 * ((cnt << 1) - 1);//
		d *= (cnt << 1);//2 4 6 8 10 ...
		cnt++;
	}while(cnt <= 6);

	return result;
}

//保证输入值是有效的
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}




/*====================================================================================================*/
/*====================================================================================================*
**函数 : Q_rsqrt
**功能 : 快速计算 1/Sqrt(x) 
**输入 : number  
**输出 : 结果
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 

/**************************实现函数********************************************
*函数原型:    array_astrict_lower(int16_t *array,int16_t value)
*功　　能:    对数组下限限制
输入参数：    *array   目标数组指针
*             value      
输出参数：    无
*******************************************************************************/
void array_astrict(int16_t *array,int16_t lower,int16_t upper)
{
   int16_t length = sizeof(array); 
	 uint16_t i = 0;
   for(i=0;i<length;i++)
   {
     if(*(array+i)<lower)  *(array+i) = lower;
     else if(*(array+i)>upper)  *(array+i) = upper;
   } 
}

/**************************实现函数********************************************
*函数原型:    array_assign(int16_t *array,int16_t value)
*功　　能:    对数组赋值
输入参数：    *array   目标数组指针 
*             value      
输出参数：    无
*******************************************************************************/
void array_assign(int16_t *array,int16_t value)
{
   uint16_t length = sizeof(array); 
	 uint16_t i=0;
   for(i=0;i<length;i++)
   {
     *(array+i) = value;
   } 
}

/**************************实现函数********************************************
*函数原型:    data_limit(float data,flaot toplimit,float lowerlimit)
*功　　能:    数据限幅
输入参数：    data       要操作的数据 
*             toplimit   上限
*             lowerlimit 下限
输出参数：    无
*******************************************************************************/
float data_limit(float data,float toplimit,float lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
	return data;
}


/***********************************************
  * @brief  可变增益自适应参数
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.6f)
	{
	   error = 0.6f;
	}
	result = 1 - 1.667f * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}


float middle_3(float input) //3个数取中间的数
{ 

  int a,b,c,t; 


  if(a<b)

  { 

     t=a;a=b;b=t; 

  } 

 if(b<c)//9 8 7 

 { 

  t=b;b=c;c=t;      

 } 

 if(a<b)//9 8 7 

 { 

  t=a;a=b;b=t; 

 } 

 return b; 

}



/**************************实现函数********************************************
*函数原型:    rad(double angle)
*功　　能:    角度转化为弧度
输入参数：    角度
输出参数：    弧度
*******************************************************************************/
//float Rad(float angle)
//{
//    return angle * AtR ;
//}
/**************************实现函数********************************************
*函数原型:    degree(double rad)
*功　　能:    弧度转化为角度	
输入参数：    弧度
输出参数：    角度
*******************************************************************************/


float my_deathzoom_2(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}
  return (t);
}


float my_deathzoom(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
