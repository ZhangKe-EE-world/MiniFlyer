#ifndef _KALMAN_H
#define _KALMAN_H




struct _1_ekf_filter
{
	float LastP;
	float	Now_P;
	float out;
	float Kg;
	float Q;
	float R;	
};

//void ekf_1(struct EKF *ekf,void *input);  //Ò»Î¬¿¨¶ûÂü
extern void kalman_1(struct _1_ekf_filter *ekf,float input);  //Ò»Î¬¿¨¶ûÂü

#endif


