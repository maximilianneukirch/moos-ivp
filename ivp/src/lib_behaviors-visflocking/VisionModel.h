#ifndef VISION_MODEL_H
#define VISION_MODEL_H

#include <vector>
#include <cmath>

class VisionModel {
public:
    VisionModel();
    ~VisionModel() {};

    void setParams(double a0, double a1, double b0, double b1, double v0, double gam, double fov);

    void compute(double vel_now, const std::vector<int>& V_now, double& dvel, double& dpsi);

private:
    std::vector<double> dPhi_V_of(const std::vector<int>& V);

    double m_a0;
    double m_a1;
    double m_b0;
    double m_b1;
    double m_v0;
    double m_gam;
    double m_fov;
};

#endif