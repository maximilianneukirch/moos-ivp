//---------------------------------------------------------------
// VisionModel.h
// This is strongly inspired by 
// Mezey, David & Bastien, Renaud & Zheng, Yating & McKee, Neal &
// Stoll, David & Hamann, Heiko & Romanczuk, Pawel. (2025). 
// Purely vision-based collective movement of robots. npj Robotics.
// 3. 10.1038/s44182-025-00027-2. 
//---------------------------------------------------------------

#include "VisionModel.h"

VisionModel::VisionModel() {
    m_alpha_0 = 1.25;
    m_alpha_1 = 1.25;
    m_beta_0  = 1.75;
    m_beta_1  = 1.75;
    m_fov_rad = 175.0 * (M_PI / 180.0);
    m_resolution = 320;
}

void VisionModel::setParams(double a0, double a1, double b0, double b1, double fov, int res) {
    m_alpha_0 = a0;
    m_alpha_1 = a1;
    m_beta_0  = b0;
    m_beta_1  = b1;
    m_fov_rad = fov * (M_PI / 180.0);
    m_resolution = res;
}

void VisionModel::compute(const std::vector<int>& vpf, double& out_dv, double& out_dpsi) {
    out_dv = 0.0;
    out_dpsi = 0.0;

    if (vpf.size() < 2) return;

    double dphi = m_fov_rad / (double)m_resolution; // degree per bin
    double start_phi = -m_fov_rad / 2.0;            // start at -FOV/2

    for (size_t i = 1; i < vpf.size(); ++i) {
        // Current angle of the bin (in rad)
        double phi_i = start_phi + i * dphi;

        // Discrete derivative: dV / dphi
        double dV_dphi = (vpf[i] - vpf[i-1]) / dphi;
        double dV_dphi_sq = dV_dphi * dV_dphi;

        // Eq 5: A(phi, t)
        double A_val = -vpf[i] + (m_alpha_1 * dV_dphi_sq);

        // Eq 7: B(phi, t)
        double B_val = -vpf[i] + (m_beta_1 * dV_dphi_sq);

        // Eq 4: Integration over fov (speed)
        out_dv += m_alpha_0 * A_val * std::cos(phi_i) * dphi;

        // Eq 6: Integration fov (rotation)
        out_dpsi += m_beta_0 * B_val * std::sin(phi_i) * dphi;
    }
}