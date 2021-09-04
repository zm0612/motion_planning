//
// Created by meng on 2021/9/3.
//
#include "Hybrid_A_star.h"
#include <cmath>
#include <unordered_set>

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

    GridNodeMap_ = new GridNodePtr **[GLX_SIZE_];
    for (int i = 0; i < GLX_SIZE_; ++i) {
        GridNodeMap_[i] = new GridNodePtr *[GLY_SIZE_];
        for (int j = 0; j < GLY_SIZE_; ++j) {
            GridNodeMap_[i][j] = new GridNodePtr[GLZ_SIZE_];
            for (int k = 0; k < GLZ_SIZE_; ++k) {
                Vector3i tempIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tempIdx);
                GridNodeMap_[i][j][k] = new GridNode(tempIdx, pos);
            }
        }
    }
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
    double optimal_cost = std::numeric_limits<double>::max();
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

void HybridAStar::SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
    ros::Time start_time = ros::Time::now();
    Eigen::Vector3i start_idx = coord2gridIndex(start_pt);
    Eigen::Vector3d start_velocity(0.0, 0.0, 0.0);

    GridNodePtr current_node_ptr = nullptr;
    GridNodePtr neighbor_node_ptr = nullptr;
    open_set_.clear();

    GridNodePtr start_node_ptr = new GridNode(start_idx, start_pt);
    start_node_ptr->g_score_ = 0.0;
    start_node_ptr->f_score_ = OptimalBVP(start_pt, start_velocity, end_pt);
    start_node_ptr->id_ = 1;
    open_set_.insert(std::make_pair(start_node_ptr->f_score_, start_node_ptr));

    std::vector<GridNodePtr> neighbors_ptr;
    std::vector<TrajectoryStatePtr> neighbors_traj_state;

    ///TODO: 当固定每次搜索的路径长度时，会出现遍历openset到空也不能找到路径
    while (!open_set_.empty()) {
        current_node_ptr = open_set_.begin()->second;
        current_node_ptr->id_ = -1;
        open_set_.erase(open_set_.begin());

        double dist = (current_node_ptr->coord_ - end_pt).norm();
        if (dist < resolution_) {
            terminate_ptr_ = current_node_ptr;
            ros::Duration use_time = ros::Time::now() - start_time;
            ROS_INFO("\033[1;32m --> hybrid A* use time: %f (ms)\033[0m", use_time.toSec() * 1000);
            return;
        }

        TrajectoryStatePtr ***trajectory_state_ptr;
        if (current_node_ptr->trajectory_ == nullptr) {
            trajectory_state_ptr = trajectoryLibrary(current_node_ptr->coord_,
                                                     start_velocity, end_pt);
        } else {
            trajectory_state_ptr = trajectoryLibrary(current_node_ptr->coord_,
                                                     current_node_ptr->trajectory_->Velocity.back(), end_pt);
        }

        GetNeighbors(trajectory_state_ptr, neighbors_ptr, neighbors_traj_state);

        for (unsigned int i = 0; i < neighbors_ptr.size(); ++i) {
            neighbor_node_ptr = neighbors_ptr[i];

            ///TODO: G score用的是到起点的欧氏距离，这很明显是不合理的，更合理的方式应该是选用类似最低能量代价等
            double delta_score = (neighbor_node_ptr->coord_ - current_node_ptr->coord_).norm();
            if (neighbor_node_ptr->id_ == 0) {
                neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + delta_score;
                neighbor_node_ptr->f_score_ = neighbor_node_ptr->g_score_ + neighbors_traj_state[i]->Trajctory_Cost;
                neighbor_node_ptr->parent_ = current_node_ptr;
                neighbor_node_ptr->trajectory_ = neighbors_traj_state[i];
                open_set_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));

                neighbor_node_ptr->id_ = 1;
                continue;
            } else if (neighbor_node_ptr->id_ == 1) {
                if (neighbor_node_ptr->g_score_ > current_node_ptr->g_score_ + delta_score) {
                    neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + delta_score;
                    neighbor_node_ptr->f_score_ = neighbor_node_ptr->g_score_ + neighbors_traj_state[i]->Trajctory_Cost;
                    neighbor_node_ptr->parent_ = current_node_ptr;
                    delete neighbor_node_ptr->trajectory_;
                    neighbor_node_ptr->trajectory_ = neighbors_traj_state[i];

                    std::multimap<double, GridNodePtr>::iterator map_iter = open_set_.begin();
                    for (; map_iter != open_set_.end(); map_iter++) {
                        if (map_iter->second->index_ == neighbor_node_ptr->index_) {
                            open_set_.erase(map_iter);
                            open_set_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                            break;
                        }
                    }
                }
                continue;
            } else {
                continue;
            }
        }
    }
    ROS_WARN_STREAM("Hybrid A* failed to search path!");
}

TrajectoryStatePtr ***HybridAStar::trajectoryLibrary(const Eigen::Vector3d &start_pt,
                                                     const Eigen::Vector3d &start_velocity,
                                                     const Eigen::Vector3d &target_pt) {
    Vector3d acc_input;
    Vector3d pos, vel;
    int a = 0;
    int b = 0;
    int c = 0;

    double min_Cost = 100000.0;
    double Trajctory_Cost;
    TrajectoryStatePtr ***TraLibrary;
    TraLibrary = new TrajectoryStatePtr **[discretize_step_ + 1];//recored all trajectories after input

    //如果想要加快以下过程，可以采用for循环并行加速
    for (int i = 0; i <= discretize_step_; i++) {//acc_input_ax
        TraLibrary[i] = new TrajectoryStatePtr *[discretize_step_ + 1];
        for (int j = 0; j <= discretize_step_; j++) {//acc_input_ay
            TraLibrary[i][j] = new TrajectoryStatePtr[discretize_step_ + 1];
            for (int k = 0; k <= discretize_step_; k++) {//acc_input_az
                vector<Vector3d> Position;
                vector<Vector3d> Velocity;
                acc_input(0) = double(-max_input_acc_ + i * (2 * max_input_acc_ / double(discretize_step_)));
                acc_input(1) = double(-max_input_acc_ + j * (2 * max_input_acc_ / double(discretize_step_)));
                ///TODO: 当Z轴加速度取负轨迹会出现异常，规划出来的轨迹会跑到地图之外
                acc_input(2) = double(k * (2 * max_input_acc_ / double(discretize_step_)));

                pos(0) = start_pt(0);
                pos(1) = start_pt(1);
                pos(2) = start_pt(2);
                vel(0) = start_velocity(0);
                vel(1) = start_velocity(1);
                vel(2) = start_velocity(2);
                Position.push_back(pos);
                Velocity.push_back(vel);

                bool collision = false;
                double delta_time;
                delta_time = time_interval_ / double(time_step_);

                ///TODO: 增加动态规划轨迹长度的功能，当轨迹在目标附近时，应该减小轨迹长度
                for (int step = 0; step <= time_step_; step++) {
                    pos = pos + vel * delta_time + 0.5 * acc_input * delta_time * delta_time;
                    vel = vel + acc_input * delta_time;

                    Position.push_back(pos);
                    Velocity.push_back(vel);
                    double coord_x = pos(0);
                    double coord_y = pos(1);
                    double coord_z = pos(2);
                    if (isObsFree(coord_x, coord_y, coord_z) != 1) {
                        collision = true;
                    }
                }

                Trajctory_Cost = OptimalBVP(pos, vel, target_pt);

                TraLibrary[i][j][k] = new TrajectoryState(Position, Velocity, Trajctory_Cost);

                if (collision)
                    TraLibrary[i][j][k]->setCollisionfree();

                if (Trajctory_Cost < min_Cost && !TraLibrary[i][j][k]->collision_check) {
                    a = i;
                    b = j;
                    c = k;
                    min_Cost = Trajctory_Cost;
                }
            }
        }
    }
    TraLibrary[a][b][c]->setOptimal();

    return TraLibrary;
}

void HybridAStar::GetNeighbors(TrajectoryStatePtr ***trajectory_state_ptr,
                               std::vector<GridNodePtr> &neighbors,
                               vector<TrajectoryStatePtr> &neighbors_traj_state) {
    neighbors.clear();
    neighbors_traj_state.clear();
    std::unordered_map<int, std::pair<GridNodePtr, TrajectoryStatePtr>> candidates_grid_node_traj;

    for (int i = 0; i < discretize_step_; ++i) {
        for (int j = 0; j < discretize_step_; ++j) {
            for (int k = 0; k < discretize_step_; ++k) {
                auto current_trajectory_state_ptr = trajectory_state_ptr[i][j][k];

                if (current_trajectory_state_ptr->collision_check) {
                    delete current_trajectory_state_ptr;
                    continue;
                }

                Eigen::Vector3d coord_end = current_trajectory_state_ptr->Position.back();
                Eigen::Vector3i index_end = coord2gridIndex(coord_end);

                const int index = k * i * j + j * i + i;
                if (candidates_grid_node_traj.count(index) == 0) {
                    GridNodePtr current_grid_node_ptr;
                    current_grid_node_ptr = GridNodeMap_[index_end.x()][index_end.y()][index_end.z()];
                    current_grid_node_ptr->coord_ = coord_end;
                    candidates_grid_node_traj[index] = std::make_pair(current_grid_node_ptr,
                                                                      current_trajectory_state_ptr);
                } else {
                    if (current_trajectory_state_ptr->Trajctory_Cost <
                        candidates_grid_node_traj[index].second->Trajctory_Cost) {
                        GridNodePtr current_grid_node_ptr;
                        current_grid_node_ptr = GridNodeMap_[index_end.x()][index_end.y()][index_end.z()];
                        current_grid_node_ptr->coord_ = coord_end;
                        candidates_grid_node_traj[index] = std::make_pair(current_grid_node_ptr,
                                                                          current_trajectory_state_ptr);
                    }
                }
            }
        }
    }

    auto iter = candidates_grid_node_traj.begin();
    for (; iter != candidates_grid_node_traj.end(); ++iter) {
        neighbors.emplace_back(iter->second.first);
        neighbors_traj_state.emplace_back(iter->second.second);
    }
}

std::vector<Eigen::Vector3d> HybridAStar::GetPath() {
    std::vector<Eigen::Vector3d> path;
    std::vector<GridNodePtr> grid_path;

    GridNodePtr grid_node_ptr = terminate_ptr_;
    while (grid_node_ptr != nullptr) {
        grid_path.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->parent_;
    }

    for (auto &ptr: grid_path) {
        if (ptr->trajectory_ == nullptr) {
            continue;
        }
        for (auto iter = ptr->trajectory_->Position.end() - 1; iter != ptr->trajectory_->Position.begin(); iter--) {
            path.emplace_back(*iter);
        }
    }

    reverse(path.begin(), path.end());

    return path;
}

void HybridAStar::Reset() {
    for (int i = 0; i < GLX_SIZE_; i++)
        for (int j = 0; j < GLY_SIZE_; j++)
            for (int k = 0; k < GLZ_SIZE_; k++) {
                if (GridNodeMap_[i][j][k] != nullptr) {
                    GridNodeMap_[i][j][k]->id_ = 0;
                    GridNodeMap_[i][j][k]->g_score_ = std::numeric_limits<double>::max();
                    GridNodeMap_[i][j][k]->f_score_ = std::numeric_limits<double>::max();
                    GridNodeMap_[i][j][k]->parent_ = nullptr;
                    delete GridNodeMap_[i][j][k]->trajectory_;
                    GridNodeMap_[i][j][k]->trajectory_ = nullptr;
                }
            }
}