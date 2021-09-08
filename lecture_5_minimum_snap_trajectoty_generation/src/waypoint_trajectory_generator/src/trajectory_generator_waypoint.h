#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
private:
    int Factorial(int x);

public:
    TrajectoryGeneratorWaypoint() = default;

    ~TrajectoryGeneratorWaypoint() = default;

    Eigen::MatrixXd PolyQPGeneration(
            int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);
};

#endif
