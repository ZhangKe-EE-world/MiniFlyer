 
#include "kalman.h"
#include <math.h>
#include "sys.h"
/*********************************************************************************************************************
**卡尔曼滤波
**@brief: 线性最优评估滤波
**@param[in]  InputData 滤波前的数据，QR误差
**@param[out] None
**@return 滤波后的数据
**@remark: 通过修改过程噪声和测量噪声R,Q值优化输出值
X(k)=A X(k-1)+B U(k)+W(k)  
Z(k)=H X(k)+V(k)
AB是系统参数
XK时刻的系统状态
H：测量系统的参数
Z：K时刻的测量值
WV：过程和测量噪声

X(k|k-1)=X(k-1|k-1) 预测下一状态的系统
P(k|k-1)=P(k-1|k-1) +Q  
Kg(k)= P(k|k-1) / (P(k|k-1) + R)   计算Kg卡尔曼增益
X(k|k)= X(k|k-1)+Kg(k) (Z(k)-X(k|k-1))   根据预测值结合估算值得出当前状态值
P(k|k)=(1-Kg(k))P(k|k-1)  更新协方差


(k-1|k-1)上一个状态的最优评估
(k|k-1) 由上一个状态的最优评估预测当前状态的最优评估
(k|k)  由预测本状态的评估具体实现最优评估

Q:系统过程的协方差
R：测量的协方差
高斯白 = Q+R
Kg：卡尔曼增益
P：协方差

注:本卡尔曼滤波器争对单模型，单测量H,I均为1，没有控制量U=0，通常对A,B初始值取1
注：X会逐渐收敛，X(0|0)初始测量值状态根据，测量前的数值而定。
注 :   P (0|0)一般不取0，取0意味在0时候的方差为0，系统认为0时刻的值是最优的。然而在实际系统中往往0时刻不是最优的
 **********************************************************************************************************************/
 		//		const float Q = 0.018;//0.01; 
		//		const float R = 0.543;//0.9;			
//float kalman_1(float InputData,float Q,float R)  //一维卡尔曼
//{
//	struct Kalman{
//		float K_1_K_1;//(k-1|k-1)
//		float K_K_1;  //(k|k-1)
//		float K_K;    //(k|k)
//	};
//	static struct Kalman P={0};
//	static struct Kalman X={0};
//	float Kg;
//	
//	X.K_K_1 = X.K_1_K_1;
//	P.K_K_1 = P.K_1_K_1 + Q;
//	Kg = P.K_K_1 / (P.K_K_1 + R);
//	X.K_K = X.K_K_1 + Kg * (InputData - X.K_K_1);
//	P.K_K = (1-Kg) * P.K_K_1 ;
//	
//	X.K_1_K_1 = X.K_K;
//	P.K_1_K_1 = P.K_K;
//	return X.K_K;
//}

void kalman_1(struct _1_ekf_filter *ekf,float input)  //一维卡尔曼
{
	ekf->Now_P = ekf->LastP + ekf->Q;
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
	ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
}


/*****************************************************
*多维卡尔曼是一维卡尔曼的多维噪声扩展。
*利用角速度估计角度的预测值，同时角度更新角速度的偏置补偿值。
*http://wapwenku.baidu.com/view/0ac55e8cf01dc281e53af0ff?pn=5&pu=
*/
float kalman_2_Update(float InputAngle,float InputGyro,float dt)  //二维卡尔曼
{
	struct Kalman{
		float k_1_k_1;//(k-1|k-1)
		float k_k_1;  //(k|k-1)
		float k_k;    //(k|k)
	};
	static struct Kalman X={0};
		
	const float R_angle=0.5; //角度计算的过程噪声		
	const float h_0=1; //二维卡尔曼矩阵系数
	const float Q_angle=0.001;  
	const float Q_gyro=0.001;	
		
	static float P[2][2];
	float k_0,k_1;
	float t0,t1;
	float PHt_0;
	float PHt_1;
	float Pdot[4];	
	float E;
//矩阵	
//A=[1,dt]
//	[0,1 ];
//B=[dt,0];
//H=[1,0];
//P=[a,b]
	//[c,d];
//事实上X输入的也是二维矩阵X[InputAngle,InputGyro]，而初始值即为X.k_1_k_1和Q_bias，可以取0
//--------------------------------		
//X(k|k-1)=A X(k-1|k-1)+B U(k) 	
	static float Q_bias; //角速度偏置补偿值
	X.k_k_1 = X.k_1_k_1 + (InputGyro - Q_bias) * dt; //用角速度积分预测角度新值
//---------------------------------		
// P(k|k-1)=A P(k-1|k-1) A'

	Pdot[0]=Q_angle - P[0][1] - P[1][0]; 

	Pdot[1]= - P[1][1];

	Pdot[2]= - P[1][1];

	Pdot[3]= Q_gyro;

	P[0][0] += Pdot[0] * dt;

	P[0][1] += Pdot[1] * dt;

	P[1][0] += Pdot[2] * dt;

	P[1][1] += Pdot[3] * dt;
	//以上各式由矩阵运算得来
//---------------------------------	
/*
* Update covariance matrix:
* (equation 21-3)
*
* P = P - K H P
* Let
* Y = H P

*/
//Kg(k)= P(k|k-1) H'/ (H P(k|k-1) H' + R) 
	PHt_0 = h_0 * P[0][0];

	PHt_1 = h_0 * P[1][0];

	E = R_angle + h_0 * PHt_0;

	k_0 = PHt_0 / E;

	k_1 = PHt_1 / E;
//---------------------------------	
//P(k|k)=(I-Kg(k) H)P(k|k-1)	

	t0 = PHt_0;

	t1 = h_0 * P[0][1];

	P[0][0] -= k_0 * t0;

	P[0][1] -= k_0 * t1;

	P[1][0] -= k_1 * t0;

	P[1][1] -= k_1 * t1;

//---------------------------------	
	
/*
* Update our state estimate: *

* Xnew = X + K * error

* X(k|k)= X(k|k-1)+Kg(k) (Z(k)-X(k|k-1)) 
*/

	X.k_k = X.k_k_1 + k_0 * (InputAngle - X.k_k_1);

	Q_bias = Q_bias + k_1 * (InputAngle - X.k_k_1);
	

	X.k_1_k_1 = X.k_k; //为下一次计算做准备
	
	return X.k_k;
}
//对比一维二维的性质，可以扩展N维卡尔曼
/*********************************************END OF FILE ***********************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*******************************二维卡尔曼范例************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
//http://blog.csdn.net/xiahouzuoxin/article/details/39582483
typedef struct{
	float x;
	float p;
	float A;
	float H;
	float q;
	float r;
	float gain;
}kalman1_state;

typedef struct{
	float x[2];
	float p[2][2];
	float A[2][2];
	float H[2];
	float q[2];
	float r;
	float gain[2];
}kalman2_state;
/*********************************************************************************************************
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1; 
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 *   state - Klaman filter structure
 *   init_x - initial x state value   
 *   init_p - initial estimated error convariance
 * @outputs 
 * @retval  
 */
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 2e2;//10e-6;  /* predict noise convariance */
    state->r = 5e2;//10e-5;  /* measure error convariance */
}
/*********************************************************************************************************
 * @brief   
 *   1 Dimension Kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 * @retval  
 *   Estimated result
 */
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

/*********************************************************************************************************
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}

/*********************************************************************************************************
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - Updated state value, Such as angle,velocity
 *   state->x[1] - Updated state value, Such as diffrence angle, acceleration
 *   state->p    - Updated estimated error convatiance matrix
 * @retval  
 *   Return value is equals to state->x[0], so maybe angle or velocity.
 */
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0;
    float temp1;
    float temp;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*******************************三维卡尔曼范例************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
//https://www.embbnux.com/2015/01/30/9_dof_imu_with_kalman_filter_on_avr/
//算法实现
//kalman.c
float dtTimer   = 0.008;
float xk[9] = {0,0,0,0,0,0,0,0,0};
float pk[9] = {1,0,0,0,1,0,0,0,0};
float I[9]  = {1,0,0,0,1,0,0,0,1};
float R[9]  = {0.5,0,0,0,0.5,0,0,0,0.01};
float Q[9] = {0.005,0,0,0,0.005,0,0,0,0.001};
 
void matrix_add(float* mata,float* matb,float* matc){
    uint8_t i,j;
    for (i=0; i<3; i++){
       for (j=0; j<3; j++){
          matc[i*3+j] = mata[i*3+j] + matb[i*3+j];
       }
    }
}
 
void matrix_sub(float* mata,float* matb,float* matc){
    uint8_t i,j;
    for (i=0; i<3; i++){
       for (j=0; j<3; j++){
          matc[i*3+j] = mata[i*3+j] - matb[i*3+j];
       }
    }
}
 
void matrix_multi(float* mata,float* matb,float* matc){
  uint8_t i,j,m;
  for (i=0; i<3; i++)
  {
    for (j=0; j<3; j++)
   {
      matc[i*3+j]=0.0;
      for (m=0; m<3; m++)
     {
        matc[i*3+j] += mata[i*3+m] * matb[m*3+j];
      }
    }
 }
}
 
void KalmanFilter(float* am_angle_mat,float* gyro_angle_mat)
{
		uint8_t i,j;
		float yk[9];
		float pk_new[9];
		float K[9];
		float KxYk[9];
		float I_K[9];
		float S[9];
		float S_invert[9];
		float sdet;
		 
		//xk = xk + uk
		matrix_add(xk,gyro_angle_mat,xk);
		//pk = pk + Q
		matrix_add(pk,Q,pk);
		//yk =  xnew - xk
		matrix_sub(am_angle_mat,xk,yk);
		//S=Pk + R
		matrix_add(pk,R,S);
		//S??invert
		sdet = S[0] * S[4] * S[8]
							+ S[1] * S[5] * S[6]
							+ S[2] * S[3] * S[7]
							- S[2] * S[4] * S[6]
							- S[5] * S[7] * S[0]
							- S[8] * S[1] * S[3];
		 
		S_invert[0] = (S[4] * S[8] - S[5] * S[7])/sdet;
		S_invert[1] = (S[2] * S[7] - S[1] * S[8])/sdet;
		S_invert[2] = (S[1] * S[7] - S[4] * S[6])/sdet;
		 
		S_invert[3] = (S[5] * S[6] - S[3] * S[8])/sdet;
		S_invert[4] = (S[0] * S[8] - S[2] * S[6])/sdet;
		S_invert[5] = (S[2] * S[3] - S[0] * S[5])/sdet;
		 
		S_invert[6] = (S[3] * S[7] - S[4] * S[6])/sdet;
		S_invert[7] = (S[1] * S[6] - S[0] * S[7])/sdet;
		S_invert[8] = (S[0] * S[4] - S[1] * S[3])/sdet;
		//K = Pk * S_invert
		matrix_multi(pk,S_invert,K);
		//KxYk = K * Yk
		matrix_multi(K,yk,KxYk);
		//xk = xk + K * yk
		matrix_add(xk,KxYk,xk);
		//pk = (I - K)*(pk)
		matrix_sub(I,K,I_K);
		matrix_multi(I_K,pk,pk_new);
		//update pk
		//pk = pk_new;
		for (i=0; i<3; i++)
		{
				for (j=0; j<3; j++)
				{
						pk[i*3+j] = pk_new[i*3+j];
				}
		}
}
//应用范例
//isr.c
//#include "kalman.h"
//float mpu_9dof_values[9]={0};
//float am_angle[3];
//float gyro_angle[3];
//float am_angle_mat[9]={0,0,0,0,0,0,0,0,0};
//float gyro_angle_mat[9]={0,0,0,0,0,0,0,0,0};
// 
////8MS
//ISR(TIMER0_OVF_vect){
////设置计数开始的初值
//TCNT0 = 130 ;  //8ms
//sei();
////采集9轴数据
////mpu_9dof_values 单位g与度/s
////加速度计和陀螺仪
//mpu_getValue6(&mpu_9dof_values[0],&mpu_9dof_values[1],&mpu_9dof_values[2],&mpu_9dof_values[3],&mpu_hmc_values[4],&mpu_hmc_values[5]);
////磁场传感器
//compass_mgetValues(&mpu_9dof_values[6],&mpu_9dof_values[7],&mpu_9dof_values[8]);
// 
//accel_compass2angle(&mpu_9dof_values[0],&mpu_9dof_values[6],am_angle);
//gyro2angle(&mpu_9dof_values[3],gyro_angle);
// 
//am_angle_mat[0] = am_angle[0];
//am_angle_mat[4] = am_angle[1];
//am_angle_mat[8] = am_angle[2];
// 
//gyro_angle_mat[0] = gyro_angle[1];
//gyro_angle_mat[4] = - gyro_angle[0];
//gyro_angle_mat[8] = - gyro_angle[2];
// 
////卡尔曼
//KalmanFilter(am_angle_mat,gyro_angle_mat);
// 
////输出三轴角度
////xk[0] xk[4] xk[8]
//}





