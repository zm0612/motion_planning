#include "trajectory_generator_waypoint.h"
#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

/*!
 * 计算x的阶乘
 * @param x
 * @return x!
 */
int TrajectoryGeneratorWaypoint::Factorial(int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
        const int d_order,                    // 导数的阶数, 其中d对应PPT中转换之后的p
        const Eigen::MatrixXd &Path,          // 航点坐标 (3d)
        const Eigen::MatrixXd &Vel,           // 边界速度
        const Eigen::MatrixXd &Acc,           // 边界加速度
        const Eigen::VectorXd &Time)          // 每一段轨迹的时间
{ //给定起始点和终点的速度加速度，更高阶的导数设置为0

    const int p_order = 2 * d_order - 1;              //多项式的最高次数 p^(p_order)t^(p_order) + ...
    const int p_num1d = p_order + 1;                  //每一段轨迹的变量个数，对于五阶多项式为：p5, p4, ... p0

    const int number_segments = Time.size();
    //每一段都有x,y,z三个方向，每一段多项式的系数的个数有3*p_num1d
    MatrixXd PolyCoeff = MatrixXd::Zero(number_segments, 3 * p_num1d);
    //整条轨迹在ｘ,y,z方向上共多少个未知系数
    const int number_coefficients = p_num1d * number_segments;
    VectorXd Px(number_coefficients), Py(number_coefficients), Pz(number_coefficients);

    /*   Produce Mapping Matrix M to the entire trajectory, M is a mapping matrix that maps polynomial coefficients to derivatives.   */
    const int M_block_rows = d_order * 2;
    const int M_block_cols = p_num1d;
    MatrixXd M = MatrixXd::Zero(number_segments * M_block_rows, number_segments * M_block_cols);
    for (int i = 0; i < number_segments; ++i) {
        int row = i * M_block_rows, col = i * M_block_cols;
        MatrixXd sub_M = MatrixXd::Zero(M_block_rows, M_block_cols);

        for (int j = 0; j < d_order; ++j) {
            for (int k = 0; k < p_num1d; ++k) {
                if (k < j)
                    continue;

                sub_M(j, p_num1d - 1 - k) = Factorial(k) / Factorial(k - j) * pow(0, k - j);
                sub_M(j + d_order, p_num1d - 1 - k) = Factorial(k) / Factorial(k - j) * pow(Time(i), k - j);
            }
        }

        M.block(row, col, M_block_rows, M_block_cols) = sub_M;
    }

    std::cout << "    M   " << std::endl;
    std::cout << M << std::endl << std::endl;

    //构造选择矩阵C的过程非常复杂，但是只要多花点时间探索一些规律，举几个例子，应该是能写出来的!!
    const int number_valid_variables = (number_segments + 1) * d_order;
    const int number_fixed_variables = 2 * d_order + (number_segments - 1);
    MatrixXd C_T = MatrixXd::Zero(number_coefficients, number_valid_variables);
    for (int i = 0; i < number_coefficients; ++i) {
        if (i < d_order) {
            C_T(i, i) = 1;
            continue;
        }

        if (i >= number_coefficients - d_order) {
            const int delta_index = i - (number_coefficients - d_order);
            C_T(i, number_fixed_variables - d_order + delta_index) = 1;
            continue;
        }

        if ((i % d_order == 0) && (i / d_order % 2 == 1)) {
            const int index = i / (2 * d_order) + d_order;
            C_T(i, index) = 1;
            continue;
        }

        if ((i % d_order == 0) && (i / d_order % 2 == 0)) {
            const int index = i / (2 * d_order) + d_order - 1;
            C_T(i, index) = 1;
            continue;
        }

        if ((i % d_order != 0) && (i / d_order % 2 == 1)) {
            const int temp_index_0 = i / (2 * d_order) * (2 * d_order) + d_order;
            const int temp_index_1 = i / (2 * d_order) * (d_order - 1) + i - temp_index_0 - 1;
            C_T(i, number_fixed_variables + temp_index_1) = 1;
            continue;
        }

        if ((i % d_order != 0) && (i / d_order % 2 == 0)) {
            const int temp_index_0 = (i - d_order) / (2 * d_order) * (2 * d_order) + d_order;
            const int temp_index_1 = (i - d_order) / (2 * d_order) * (d_order - 1) + (i - d_order) - temp_index_0 - 1;
            C_T(i, number_fixed_variables + temp_index_1) = 1;
            continue;
        }
    }

    std::cout << "    C_T   " << std::endl;
    std::cout << C_T << std::endl << std::endl;

    MatrixXd Q = MatrixXd::Zero(number_coefficients, number_coefficients);
    for (int k = 0; k < number_segments; ++k) {
        MatrixXd sub_Q = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = 0; i <= p_order; ++i) {
            for (int l = 0; l <= p_order; ++l) {
                if (p_num1d - i <= d_order || p_num1d - l <= d_order)
                    continue;

                sub_Q(i, l) = Factorial(p_order - i) / Factorial(p_order - d_order - i) *
                              Factorial(p_order - l) / Factorial(p_order - d_order - l) /
                              (p_order - i + p_order - l - (2 * d_order - 1)) *
                              pow(Time(k), p_order - i + p_order - l - (2 * d_order - 1));
            }
        }

        const int row = k * p_num1d;
        Q.block(row, row, p_num1d, p_num1d) = sub_Q;
    }

    std::cout << "    Q   " << std::endl;
    std::cout << Q << std::endl << std::endl;

    MatrixXd R = C_T.transpose() * M.transpose().inverse() * Q * M.inverse() * C_T;

    for (int axis = 0; axis < 3; ++axis) {
        VectorXd d_selected = VectorXd::Zero(number_valid_variables);
        for (int i = 0; i < number_coefficients; ++i) {
            if (i == 0) {
                d_selected(i) = Path(0, axis);//第0个点的x坐标值
                continue;
            }

            if (i == 1) {
                d_selected(i) = Vel(0, axis);
                continue;
            }

            if (i == 2) {
                d_selected(i) = Acc(0, axis);
                continue;
            }

            if (i == number_coefficients - 1) {
                d_selected(number_fixed_variables - 1) = Acc(1, axis);
                continue;
            }

            if (i == number_coefficients - 2) {
                d_selected(number_fixed_variables - 2) = Vel(1, axis);
                continue;
            }

            if (i == number_coefficients - 3) {
                d_selected(number_fixed_variables - 3) = Path(number_segments, axis);
                continue;
            }

            if ((i % d_order == 0) && (i / d_order % 2 == 0)) {
                const int index = i / (2 * d_order) + d_order - 1;
                d_selected(index) = Path(i / (2 * d_order), axis);
                continue;
            }
        }

        if (axis == 0) {
            std::cout << "    d_selected   " << std::endl;
            std::cout << d_selected << std::endl << std::endl;
        }

        MatrixXd R_PP = R.block(number_fixed_variables, number_fixed_variables,
                                number_valid_variables - number_fixed_variables,
                                number_valid_variables - number_fixed_variables);
        VectorXd d_F = d_selected.head(number_fixed_variables);
        MatrixXd R_FP = R.block(0, number_fixed_variables, number_fixed_variables,
                                number_valid_variables - number_fixed_variables);

        MatrixXd d_optimal = -R_PP.inverse() * R_FP.transpose() * d_F;

        VectorXd d = C_T * d_selected;

        if (axis == 0)
            Px = d;

        if (axis == 1)
            Py = d;

        if (axis == 2)
            Pz = d;
    }

    for (int i = 0; i < number_segments; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (j == 0) {
                PolyCoeff.block(i, j * p_num1d, 1, p_num1d) =
                        Px.block(p_num1d * i, 0, p_num1d, 1).transpose();
                continue;
            }

            if (j == 1) {
                PolyCoeff.block(i, j * p_num1d, 1, p_num1d) =
                        Py.block(p_num1d * i, 0, p_num1d, 1).transpose();
                continue;
            }

            if (j == 2) {
                PolyCoeff.block(i, j * p_num1d, 1, p_num1d) =
                        Pz.block(p_num1d * i, 0, p_num1d, 1).transpose();
                continue;
            }
        }
    }

    return PolyCoeff;
}
