#ifndef __MY_MATH_H
#define	__MY_MATH_H

extern const float M_PI;
extern const float AtR;
extern const float RtA;
extern const float Gyro_G;
extern const float Gyro_Gr;
#define PI 3.1415926f
#define squa( Sq )        (((float)Sq)*((float)Sq))
#define absu16( Math_X )  ((Math_X)<0? -(Math_X):(Math_X))
#define absFloat( Math_X )((Math_X)<0? -(Math_X):(Math_X))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

extern float safe_asin(float v);
extern float arcsin(float x);
extern float arctan(float x);
extern float sine(float x);
extern float cosine(float x);
extern float Q_rsqrt(float number);
extern float VariableParameter(float error);
#endif /* __Algorithm_math_H */
