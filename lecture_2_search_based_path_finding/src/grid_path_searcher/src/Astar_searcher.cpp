#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u,
                                  int max_x_id, int max_y_id, int max_z_id) {
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++) {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++) {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++) {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z) {
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++) {
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if (GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_INFO("\033[1;32m --> visited_nodes size : %d \033[0m", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
    Vector3d pt;

    pt(0) = ((double) index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double) index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double) index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

/*!
 * 将坐标值转换为栅格坐标
 * @param pt
 * @return
 */
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
    Vector3i idx;
    //max min是为了防止栅格点处在地图范围之外
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const {
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const {
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
    neighborPtrSets.clear();
    edgeCostSets.clear();

    //Step 4
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                Eigen::Vector3i idx = currentPtr->index + Eigen::Vector3i(dx, dy, dz);

                if (idx == currentPtr->index)
                    continue;

                if (idx.x() < 0 || idx.x() >= GLX_SIZE ||
                    idx.y() < 0 || idx.y() >= GLY_SIZE ||
                    idx.z() < 0 || idx.z() >= GLZ_SIZE)
                    continue;

                if (isOccupied(idx)){
                    continue;
                }

                GridNodePtr temp_grid_node = GridNodeMap[idx.x()][idx.y()][idx.z()];
                if (temp_grid_node->id == -1){
                    continue;
                }

                neighborPtrSets.emplace_back(temp_grid_node);

                double edge_cost = (temp_grid_node->index - currentPtr->index).norm();
                edgeCostSets.emplace_back(edge_cost);
            }
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
#define sqrt_3 1.7320508
#define sqrt_2 1.4142136

    double h = 0.0;

    if (heuristic_function_type == HeuristicFunctionType::Manhattan) {
        h = (node2->index - node1->index).lpNorm<1>();
    } else if (heuristic_function_type == HeuristicFunctionType::Euclidean) {
        h = (node2->index - node1->index).norm();
    } else if (heuristic_function_type == HeuristicFunctionType::Diagonal) {
        double dx = std::abs(node1->index.x() - node2->index.x());
        double dy = std::abs(node1->index.y() - node2->index.y());
        double dz = std::abs(node1->index.z() - node2->index.z());
        double min_xyz = std::min({dx, dy, dz});
        double max_xyz = std::max({dx, dy, dz});
        double mid_xyz = dx + dy + dz - min_xyz - max_xyz;
        h = (sqrt_3 - sqrt_2) * min_xyz + (sqrt_2 - 1) * mid_xyz + max_xyz;
    } else if (heuristic_function_type == HeuristicFunctionType::Dijkstra) {
        h = 0.0;
    }

    if (use_tie_breaker) {
        h = h * 1.01;
    }

    return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
    ros::Time time_1 = ros::Time::now();

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr->gScore = 0;
    //Step 1
    startPtr->fScore = getHeu(startPtr, endPtr);

    startPtr->id = 1;
    startPtr->coord = start_pt;
    openSet.insert(make_pair(startPtr->fScore, startPtr));

    // Step 2
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while (!openSet.empty()) {
        //Step 3
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1;
        openSet.erase(openSet.begin());

        // if the current node is the goal 
        if (currentPtr->index == goalIdx) {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_INFO("\033[1;32m --> Time in A star is %f ms, path cost %f m \033[0m",
                     (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution);

            ROS_INFO("\033[1;32m --> heuristic type: %d (Manhattan = 0, Euclidean=1, Diagonal=2, Dijkstra=3)\033[0m",
                     (int)heuristic_function_type);
            return;
        }

        //Step 4
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        //Step 5
        for (int i = 0; i < (int) neighborPtrSets.size(); i++) {
            neighborPtr = neighborPtrSets.at(i);

            if (neighborPtr->id == 0) { //discover a new node, which is not in the closed set and open set
                //Step 6
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets.at(i);
                neighborPtr->fScore = getHeu(neighborPtr, endPtr) + neighborPtr->gScore;
                neighborPtr->cameFrom = currentPtr;
                openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));
                neighborPtr->id = 1;

                continue;
            } else if (neighborPtr->id == 1) { //this node is in open set and need to judge if it needs to update,
                //Step 7
                if (neighborPtr->gScore > currentPtr->gScore + edgeCostSets.at(i)) {
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets.at(i);
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->cameFrom = currentPtr;
                }

                continue;
            } else {//this node is in closed set
                continue;
            }
        }
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_INFO("\033[1;32m --> Time consume in A star path finding is %f \033[0m", (time_2 - time_1).toSec());
}


vector<Vector3d> AstarPathFinder::getPath() {
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    //Step 8
    GridNodePtr grid_node_ptr = terminatePtr;
    while (grid_node_ptr != NULL) {
        gridPath.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->cameFrom;
    }

    for (auto ptr: gridPath){
        path.push_back(ptr->coord);
    }

    reverse(path.begin(), path.end());

    return path;
}

void AstarPathFinder::SetHeuristic(const ros::NodeHandle &nh) {
    std::string heu_type;
    nh.getParam("heuristic_type", heu_type);
    nh.getParam("use_tie_breaker", use_tie_breaker);

//    Manhattan = 0, Euclidean=1, Diagonal=2, Dijkstra=3
    if (heu_type == "Manhattan"){
        heuristic_function_type = HeuristicFunctionType::Manhattan;
    } else if (heu_type == "Euclidean"){
        heuristic_function_type = HeuristicFunctionType::Euclidean;
    } else if (heu_type == "Diagonal"){
        heuristic_function_type = HeuristicFunctionType::Diagonal;
    } else if (heu_type == "Dijkstra"){
        heuristic_function_type = HeuristicFunctionType::Dijkstra;
    } else {
        heuristic_function_type = HeuristicFunctionType::Manhattan;
    }
}