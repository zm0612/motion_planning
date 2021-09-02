//
// Created by meng on 2021/9/3.
//
#include "Hybrid_A_star.h"
#include <cmath>

using namespace std;

void HybridAStar::initGridMap(double resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u,
                              int max_x_id, int max_y_id, int max_z_id) {
    gl_xl_ = global_xyz_l(0);
    gl_yl_ = global_xyz_l(1);
    gl_zl_ = global_xyz_l(2);

    gl_xu_ = global_xyz_u(0);
    gl_yu_ = global_xyz_u(1);
    gl_zu_ = global_xyz_u(2);

    GLX_SIZE_ = max_x_id;
    GLY_SIZE_ = max_y_id;
    GLZ_SIZE_ = max_z_id;
    GLYZ_SIZE_ = GLY_SIZE_ * GLZ_SIZE_;
    GLXYZ_SIZE_ = GLX_SIZE_ * GLYZ_SIZE_;

    resolution_ = resolution;
    inv_resolution_ = 1.0 / resolution;

    data = new uint8_t[GLXYZ_SIZE_];
    memset(data, 0, GLXYZ_SIZE_ * sizeof(uint8_t));
}

void HybridAStar::setObs(const double coord_x, const double coord_y, const double coord_z) {
    if (coord_x < gl_xl_ || coord_y < gl_yl_ || coord_z < gl_zl_ ||
        coord_x >= gl_xu_ || coord_y >= gl_yu_ || coord_z >= gl_zu_)
        return;

    int idx_x = static_cast<int>((coord_x - gl_xl_) * inv_resolution_);
    int idx_y = static_cast<int>((coord_y - gl_yl_) * inv_resolution_);
    int idx_z = static_cast<int>((coord_z - gl_zl_) * inv_resolution_);

    data[idx_x * GLYZ_SIZE_ + idx_y * GLZ_SIZE_ + idx_z] = 1;
}

bool HybridAStar::isObsFree(const double coord_x, const double coord_y, const double coord_z) {
    Vector3d pt;
    Vector3i idx;

    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE_ && idx_y >= 0 && idx_y < GLY_SIZE_ && idx_z >= 0 && idx_z < GLZ_SIZE_ &&
            (data[idx_x * GLYZ_SIZE_ + idx_y * GLZ_SIZE_ + idx_z] < 1));
}

Eigen::Vector3d HybridAStar::gridIndex2coord(const Eigen::Vector3i &index) {
    Vector3d pt;

    pt(0) = ((double) index(0) + 0.5) * resolution_ + gl_xl_;
    pt(1) = ((double) index(1) + 0.5) * resolution_ + gl_yl_;
    pt(2) = ((double) index(2) + 0.5) * resolution_ + gl_zl_;

    return pt;
}

Eigen::Vector3i HybridAStar::coord2gridIndex(const Eigen::Vector3d &pt) {
    Vector3i idx;
    idx << min(max(int((pt(0) - gl_xl_) * inv_resolution_), 0), GLX_SIZE_ - 1),
            min(max(int((pt(1) - gl_yl_) * inv_resolution_), 0), GLY_SIZE_ - 1),
            min(max(int((pt(2) - gl_zl_) * inv_resolution_), 0), GLZ_SIZE_ - 1);

    return idx;
}

Eigen::Vector3d HybridAStar::coordRounding(const Eigen::Vector3d &coord) {
    return gridIndex2coord(coord2gridIndex(coord));
}

double HybridAStar::OptimalBVP(Eigen::Vector3d start_position, Eigen::Vector3d start_velocity,
                               Eigen::Vector3d target_position) {
    double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it
    double T = 0.0;

    //初始位置和初始速度已知，末位置已知，末速度设置为0，求解OBVP问题
    double alpha_1, alpha_2, alpha_3, beta_1, beta_2, beta_3;
    const double px_0 = start_position.x();
    const double py_0 = start_position.y();
    const double pz_0 = start_position.z();
    const double vx_0 = start_velocity.x();
    const double vy_0 = start_velocity.y();
    const double vz_0 = start_velocity.z();

    const double px_T = target_position.x();
    const double py_T = target_position.y();
    const double pz_T = target_position.z();
    const double vx_T = 0.0;
    const double vy_T = 0.0;
    const double vz_T = 0.0;

    double a, b, c, d;

    //对于J到最优值求解，可以通过matlab得到解析解
    a = 0.0;
    b = -4 * (pow(vx_0, 2) + pow(vy_0, 2) + pow(vz_0, 2));
    c = -24 * (px_0 * vx_0 - px_T * vx_0 + py_0 * vy_0
               - py_T * vy_0 + pz_0 * vz_0 - pz_T * vz_0);
    d = 72 * (pz_0 * pz_T + py_0 * py_T + px_0 * px_T)
        - 36 * (pow(pz_T, 2) + pow(pz_0, 2) + pow(py_T, 2)
                + pow(py_0, 2) + pow(px_T, 2) + pow(px_0, 2));

    Matrix4d A;
    A << 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -d, -c, -b, -a;

    EigenSolver<MatrixXd> eigen_solver(A);
    MatrixXd r;
    r = eigen_solver.eigenvalues().real();
    for (int i = 0; i < 4; ++i) {
        if (r(i) > T) {
            T = r(i);
        }
    }

    alpha_1 = -12.0 * (px_T - vx_0 * T - px_0) / pow(T, 3)
              + 6.0 * (vx_T - vx_0) / pow(T, 2);
    alpha_2 = -12.0 * (py_T - vy_0 * T - py_0) / pow(T, 3)
              + 6.0 * (vy_T - vy_0) / pow(T, 2);
    alpha_3 = -12.0 * (pz_T - vz_0 * T - pz_0) / pow(T, 3)
              + 6.0 * (vz_T - vz_0) / pow(T, 2);

    beta_1 = 6.0 * (px_T - vx_0 * T - px_0) / pow(T, 2)
             - 2.0 * (vx_T - vx_0) / T;

    beta_2 = 6.0 * (py_T - vy_0 * T - py_0) / pow(T, 2)
             - 2.0 * (vy_T - vy_0) / T;

    beta_3 = 6.0 * (pz_T - vz_0 * T - pz_0) / pow(T, 2)
             - 2.0 * (vz_T - vz_0) / T;

    optimal_cost = T + 1.0 / 3.0 * pow(alpha_1, 2) * pow(T, 3) + alpha_1 * beta_1 * pow(T, 2) + pow(beta_1, 2) * T
                   + 1.0 / 3.0 * pow(alpha_2, 2) * pow(T, 3) + alpha_2 * beta_2 * pow(T, 2) + pow(beta_2, 2) * T
                   + 1.0 / 3.0 * pow(alpha_3, 2) * pow(T, 3) + alpha_3 * beta_3 * pow(T, 2) + pow(beta_3, 2) * T;

    return optimal_cost;
}