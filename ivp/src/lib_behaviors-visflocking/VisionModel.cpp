//---------------------------------------------------------------
// VisionModel.cpp
// This is strongly inspired by 
// Mezey, David & Bastien, Renaud & Zheng, Yating & McKee, Neal &
// Stoll, David & Hamann, Heiko & Romanczuk, Pawel. (2025). 
// Purely vision-based collective movement of robots. npj Robotics.
// 3. 10.1038/s44182-025-00027-2. 
//---------------------------------------------------------------

#include "VisionModel.h"
#include <cmath>

VisionModel::VisionModel() {
    m_a0  = 1.25;
    m_a1  = 1.25;
    m_b0  = 1.75;
    m_b1  = 1.75;
    m_v0  = 1.0;
    m_gam = 0.1;
    m_fov = 175.0;
}

void VisionModel::setParams(double a0, double a1, double b0, double b1, double v0, double gam, double fov) {
    m_a0  = a0;
    m_a1  = a1;
    m_b0  = b0;
    m_b1  = b1;
    m_v0  = v0;
    m_gam = gam;
    m_fov = fov;
}

std::vector<double> VisionModel::dPhi_V_of(const std::vector<int>& V) {
    int n = V.size();
    if (n == 0) return std::vector<double>();

    std::vector<double> diff(n + 1);

    // Circular padding for edge cases
    diff[0] = V[0] - V[n - 1];
    for (int i = 1; i < n; i++) {
        diff[i] = V[i] - V[i - 1];
    }
    diff[n] = V[0] - V[n - 1];

    std::vector<double> result(n);
    if (diff[0] > 0.0 && diff[n] > 0.0) {
        for (int i = 0; i < n; i++) {
            result[i] = diff[i];
        }
    } else {
        for (int i = 0; i < n; i++) {
            result[i] = diff[i + 1];
        }
    }

    return result;
}

void VisionModel::compute(double vel_now, const std::vector<int>& V_now, double& dvel, double& dpsi) {
    int n = V_now.size();
    dvel = 0.0;
    dpsi = 0.0;

    if (n < 2) return;

    std::vector<double> Phi(n);
    double fov_rad = m_fov * M_PI / 180.0;
    double start_phi = -fov_rad / 2.0;            // start at -FOV/2
    double d_phi = fov_rad / (n - 1);             // resolution (deg/bin) set automatically,
                                                  // depending on the length of the generated VPF (therefor a parameter of pSimVisionServer)

    for (int i = 0; i < n; i++) {
        // Current angle of the bin
        Phi[i] = start_phi + i * d_phi;
    }

    // Deriving over Phi
    std::vector<double> dPhi_V = dPhi_V_of(V_now);

    // Calculating series expansion of functional G
    std::vector<double> G_vel(n);
    std::vector<double> G_psi(n);
    // Spikey parts shall be handled separately because of numerical integration
    std::vector<double> G_vel_spike(n);
    std::vector<double> G_psi_spike(n);
    for (int i = 0; i < n; i++) {
        G_vel[i] = -V_now[i];                   //EDIT for EVENT-BASED CAMERA (with actual dt_V)
        G_psi[i] = -V_now[i];                   //EDIT for EVENT-BASED CAMERA (with actual dt_V)

        //G_vel_spike[i] = dPhi_V[i] * dPhi_V[i];
        //G_psi_spike[i] = dPhi_V[i] * dPhi_V[i];
        G_vel_spike[i] = std::abs(dPhi_V[i]);    // Using std::abs instead of ² to handle desity-VPF (with values >1)
        G_psi_spike[i] = std::abs(dPhi_V[i]);
    }

    double trapz_psi = 0.0;
    double sum_psi_spike = 0.0;

    double trapz_vel = 0.0;
    double sum_vel_spike = 0.0;

    for (int i = 0; i < n; i++) {
        sum_psi_spike += std::sin(Phi[i]) * G_psi_spike[i];
        sum_vel_spike += std::cos(Phi[i]) * G_vel_spike[i];
    }

    for (int i = 0; i < n - 1; i++) {
        double y_psi_i      = std::sin(Phi[i]) * G_psi[i];
        double y_psi_next   = std::sin(Phi[i+1]) * G_psi[i+1];
        trapz_psi += ((y_psi_i + y_psi_next) / 2.0) * d_phi;

        double y_vel_i      = std::cos(Phi[i]) * G_vel[i];
        double y_vel_next   = std::cos(Phi[i+1]) * G_vel[i+1];
        trapz_vel += ((y_vel_i + y_vel_next) / 2.0) * d_phi;
    }

    dpsi = m_b0 * trapz_psi + m_b0 * m_b1 * sum_psi_spike;

    dvel = m_gam * (m_v0 - vel_now) + m_a0 * trapz_vel + m_a0 * m_a1 * sum_vel_spike;

}