#ifndef VISION_MODEL_H
#define VISION_MODEL_H

#include <vector>
#include <cmath>

class VisionModel {
public:
    VisionModel();
    ~VisionModel() {};

    void setParams(double a0, double a1, double b0, double b1, double fov, int res);

    void compute(const std::vector<int>& vpf, double& out_dv, double& out_dpsi);

private:
    double m_alpha_0;
    double m_alpha_1;
    double m_beta_0;
    double m_beta_1;
    
    double m_fov_rad;
    int m_resolution;
};

#endif