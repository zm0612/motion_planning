//
// Created by meng on 2021/9/3.
//

#ifndef GRID_PATH_SEARCHER_HYBRID_A_STAR_H
#define GRID_PATH_SEARCHER_HYBRID_A_STAR_H

#include "node.h"
#include "State.h"
#include <Eigen/Dense>

using namespace Eigen;

class HybridAStar {
public:
    HybridAStar() = default;

    ~HybridAStar() = default;

    void initGridMap(double resolution, Eigen::Vector3d global_xyz_l,
                     Eigen::Vector3d global_xyz_u, int max_x_id,
                     int max_y_id, int max_z_id);

    void setObs(const double coord_x, const double coord_y, const double coord_z);

    bool isObsFree(const double coord_x, const double coord_y, const double coord_z);

    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    void SearchPath(const Eigen::Vector3d &start_pt,
                    const Eigen::Vector3d &end_pt);

    std::vector<Eigen::Vector3d> GetPath();

    void Reset();

private:
    GridNodePtr ***GridNodeMap_;

    uint8_t *data;
    int discretize_step_ = 2;
    double max_input_acc_ = 1.0;
    double time_interval_ = 1.25;
    int time_step_ = 50;

    int GLX_SIZE_, GLY_SIZE_, GLZ_SIZE_;
    int GLXYZ_SIZE_, GLYZ_SIZE_;

    double resolution_, inv_resolution_;
    double gl_xl_, gl_yl_, gl_zl_;
    double gl_xu_, gl_yu_, gl_zu_;

    GridNodePtr terminate_ptr_;
    std::multimap<double, GridNodePtr> open_set_;

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);

    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);

    void GetNeighbors(TrajectoryStatePtr ***trajectory_state_ptr,
                      std::vector<GridNodePtr> &neighbors_ptr,
                      std::vector<double> &neighbors_cost);

    double OptimalBVP(Eigen::Vector3d start_position,
                      Eigen::Vector3d start_velocity,
                      Eigen::Vector3d target_position);

    TrajectoryStatePtr ***trajectoryLibrary(const Eigen::Vector3d &start_pt,
                                            const Eigen::Vector3d &start_velocity,
                                            const Eigen::Vector3d &target_pt);
};

#endif //GRID_PATH_SEARCHER_HYBRID_A_STAR_H
