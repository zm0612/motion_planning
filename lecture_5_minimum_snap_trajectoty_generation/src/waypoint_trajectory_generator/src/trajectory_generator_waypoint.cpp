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

/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
        const int d_order,                    // 导数的阶数, 其中d对应PPT中转换之后的p
        const Eigen::MatrixXd &Path,          // 航点坐标 (3d)
        const Eigen::MatrixXd &Vel,           // 边界速度
        const Eigen::MatrixXd &Acc,           // 边界加速度
        const Eigen::VectorXd &Time)          // 每一段轨迹的时间
{ //给定起始点和终点的速度加速度，更高阶的导数设置为0

    int p_order = 2 * d_order - 1;              //多项式的最高次数 p^(p_order)t^(p_order) + ...
    int p_num1d = p_order + 1;                  //每一段轨迹的变量个数，对于五阶多项式为：p5, p4, ... p0

    int number_segments = Time.size();
    //每一段都有x,y,z三个方向，每一段多项式的系数的个数有3*p_num1d
    MatrixXd PolyCoeff = MatrixXd::Zero(number_segments, 3 * p_num1d);
    //整条轨迹在ｘ,y,z方向上共多少个未知系数
    VectorXd Px(p_num1d * number_segments), Py(p_num1d * number_segments), Pz(p_num1d * number_segments);

    /*   Produce Mapping Matrix M to the entire trajectory, M is a mapping matrix that maps polynomial coefficients to derivatives.   */
    const int M_block_rows = d_order * 2;
    const int M_block_cols = p_num1d;
    MatrixXd M = MatrixXd::Zero(number_segments * M_block_rows, number_segments * M_block_cols);
    for (int i = 0; i < number_segments; ++i) {
        int row = i * M_block_rows, col = i * M_block_cols;
        MatrixXd sum_M = MatrixXd::Zero(M_block_rows, M_block_cols);

        for (int j = 0; j < d_order; ++j) {
            for (int k = 0; k < p_num1d; ++k) {
                if (k < j)
                    continue;

                sum_M(j, k) = Factorial(k) / Factorial(k - j) * pow(0, k - j);
                sum_M(j + d_order, k) = Factorial(k) / Factorial(k - j) * pow(Time(i), k - j);
            }
        }

        M.block(row, col, M_block_rows, M_block_cols) = sum_M;
    }

    /*   Produce the dereivatives in X, Y and Z axis directly.  */

    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */

    return PolyCoeff;
}
