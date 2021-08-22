//
// Created by meng on 2021/8/22.
//

#ifndef GRID_PATH_SEARCHER_RRT_NODE_H
#define GRID_PATH_SEARCHER_RRT_NODE_H

#include <Eigen/Dense>

struct RRTNode{
    Eigen::Vector3d coordinate_;
    Eigen::Vector3i index_;
    RRTNode* parent_;
};

#endif //GRID_PATH_SEARCHER_RRT_NODE_H
