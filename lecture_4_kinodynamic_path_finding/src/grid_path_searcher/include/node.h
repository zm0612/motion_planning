//
// Created by meng on 2021/9/3.
//

#ifndef GRID_PATH_SEARCHER_NODE_H
#define GRID_PATH_SEARCHER_NODE_H

#include "State.h"
#include <Eigen/Dense>
#include <map>
#include <utility>

struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode {
    int id_;//1: open set   -1: closed set   0: not visited
    Eigen::Vector3d coord_;//当前这段轨迹终点坐标
    Eigen::Vector3i index_;//当前栅格的索引

    //g_score: 从起点到当前点的cost, 在控制空间下计算；
    //f_score: 从当前点到终点的cost, 在状态空间下使用OBVP，只考虑动力学和运动学约束，忽略障碍物
    double g_score_, f_score_;

    TrajectoryStatePtr trajectory_;//from parent to current
    GridNodePtr parent_;

    GridNode(Eigen::Vector3i index, Eigen::Vector3d coord) {
        id_ = 0;
        index_ = std::move(index);
        coord_ = std::move(coord);
        g_score_ = std::numeric_limits<double>::max();
        f_score_ = std::numeric_limits<double>::max();
        parent_ = nullptr;
        trajectory_ = nullptr;
    }
};

#endif //GRID_PATH_SEARCHER_NODE_H