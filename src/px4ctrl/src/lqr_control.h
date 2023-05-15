/*************************************************************/
/* Author: ruochengli */
/* 1598863621@qq.com */
/*************************************************************/
#ifndef __LQR_CONTROL_H
#define __LQR_CONTROL_H

#include <iostream>
#include <Eigen/Dense>
#include "input.h"
#include "controller.h"

using namespace std;

typedef Eigen::Matrix<double, 6, 6> Matrix6x6;
typedef Eigen::Matrix<double, 6, 3> Matrix6x3;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3;
typedef Eigen::Matrix<double, 3, 1> Matrix3x1;
typedef Eigen::Matrix<double, 6, 1> Matrix6x1;
typedef Eigen::Matrix<double, 3, 6> Matrix3x6;

//state: x = [p_x p_y p_z v_x v_y v_z]^T
//input: u = [a_x a_y a_z]^T
//cost function J = /int_{0}^{inf}( x^TQx + u^TRu) dt
//control law : a = -K[x(k) - x_{des}(k)] + a_{des}

class LQR
{
    private:
    Matrix6x6 A;
    Matrix6x3 B;    
    Matrix6x6 Q;
    Matrix3x3 R;
    Matrix6x1 x_des;
    Matrix6x1 x_now;
    Matrix3x6 K_;
    Parameter_t param_; 
    Odom_Data_t odom_;
    Desired_State_t des_;

    double dt_;//采样间隔

    public:
    Eigen::Vector3d u;
    void initial(const Parameter_t &param,
                 const Odom_Data_t &odom,
                 const Desired_State_t &des);//初始化
    void cal_Riccati();//黎卡提方程求解
    void calcacc();
};


#endif