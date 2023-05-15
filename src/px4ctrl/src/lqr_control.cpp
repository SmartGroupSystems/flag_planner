/*************************************************************/
/* Author: ruochengli */
/* 1598863621@qq.com */
/*************************************************************/

#include <iostream>
#include "lqr_control.h"

using namespace std;

void LQR::initial(const Parameter_t &param, 
                  const Odom_Data_t &odom,
                  const Desired_State_t &des)
{
    dt_ = param.lqrgain.dt;
    double q0 = param.lqrgain.Q0;
    double q1 = param.lqrgain.Q1;
    double q2 = param.lqrgain.Q2;
    double q3 = param.lqrgain.Q3;
    double q4 = param.lqrgain.Q4;
    double q5 = param.lqrgain.Q5;
    double r0 = param.lqrgain.R0;
    double r1 = param.lqrgain.R1;
    double r2 = param.lqrgain.R2;

    A << 1.0,   0.0,    0.0,    dt_,    0.0,    0.0,
         0.0,   1.0,    0.0,    0.0,    dt_,    0.0,
         0.0,   0.0,    1.0,    0.0,    0.0,    dt_,
         0.0,   0.0,    0.0,    1.0,    0.0,    0.0,   
         0.0,   0.0,    0.0,    0.0,    1.0,    0.0,
         0.0,   0.0,    0.0,    0.0,    0.0,    1.0;   
    // cout<< A <<endl;
    B << 0.0,   0.0,    0.0,
         0.0,   0.0,    0.0,
         0.0,   0.0,    0.0,
         dt_,   0.0,    0.0,
         0.0,   dt_,    0.0,
         0.0,   0.0,    dt_;

    Q << q0,    0.0,    0.0,    0.0,    0.0,    0.0,
         0.0,    q1,    0.0,    0.0,    0.0,    0.0,
         0.0,   0.0,     q2,    0.0,    0.0,    0.0,
         0.0,   0.0,    0.0,     q3,    0.0,    0.0,   
         0.0,   0.0,    0.0,    0.0,     q4,    0.0,
         0.0,   0.0,    0.0,    0.0,    0.0,     q5;  
    // cout<< Q <<endl;
    R <<  r0,   0.0,    0.0,
         0.0,    r1,    0.0,
         0.0,   0.0,     r2; 
    
    x_des << des.p, des.v;

    x_now << odom.p, odom.v;
    // cout<< x_now.transpose() <<endl;
}

void LQR::cal_Riccati() {
	int N = 150;//迭代终止次数
	double err = 100;//误差值
	double err_tolerance = 0.005;//误差收敛阈值
	Matrix6x6 Qf = Q;
	Matrix6x6 P = Qf;//迭代初始值
	//cout << "P初始矩阵为\n" << P << endl;
	Matrix6x6 Pn;//计算的最新P矩阵
	for (int iter_num = 0; iter_num < N; iter_num++) {
		Pn = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;//迭代公式
		//cout << "收敛误差为" << (Pn - P).array().abs().maxCoeff() << endl;
		//err = (Pn - P).array().abs().maxCoeff();//
		err = (Pn - P).lpNorm<Eigen::Infinity>();
		if(err < err_tolerance)//
		{
			P = Pn;
			cout << "迭代次数" << iter_num << endl;
			break;
		}
		P = Pn;			
	}

	Matrix3x6 K = -(R + B.transpose() * P * B).inverse() * B.transpose() * P * A;//反馈率K
	K_ = K;
    // cout<< K_<< endl;
}
void LQR::calcacc()
{
    u = K_*( x_now - x_des );
    // cout<<"xdes is"<< x_des.transpose()<<endl;
}