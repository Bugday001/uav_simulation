#ifndef PARAM_H
#define PARAM_H
#define PARAM_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Params
{
public:
    //SE3 gains
    Eigen::Vector3d k_p;
    Eigen::Vector3d k_v;
    Eigen::Vector3d k_R;
    Eigen::Vector3d k_w;

    Params() {
        k_p << 9.6, 8, 11.2;
        k_v << 2.5, 2.5, 3.5;
        k_R << 1, 1, 1;
        k_w << 1, 1, 1;
    }
};

#endif