//
// Created by meng on 2021/8/26.
//

#ifndef GRID_PATH_SEARCHER_RRT_STAR_H
#define GRID_PATH_SEARCHER_RRT_STAR_H

#include "RRT.h"

class RRTStar : public RRT {
public:
    RRTStar() = default;

    bool SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) override;

    ~RRTStar() override;

private:
    std::vector<RRTNode *> NearC(const Eigen::Vector3d &new_point, double search_radius);

    void ChooseParent(const std::vector<RRTNode *> &near_nodes, RRTNode *const &new_node);

    double GetPathLength(const RRTNode *start_node_ptr, const RRTNode *end_node_ptr);

    void Rewire(std::vector<RRTNode *> &near_nodes, const RRTNode *const &new_node);
};

#endif //GRID_PATH_SEARCHER_RRT_STAR_H
