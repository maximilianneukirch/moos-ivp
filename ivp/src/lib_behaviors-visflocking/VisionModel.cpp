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
    m_mask_sigmoid = false;   // paper's printed cos/sin masks unless told otherwise
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

    std::vector<double> result(n, 0.0);

    bool full_fov = (m_fov >= 359.9);
    
    // forward difference
    for (int i = 0; i < n; i++) {
        int val_current = V[i];
        int val_prev;

        if (i == 0) {
            if (full_fov) {
                val_prev = V[n - 1];
            } else {
                val_prev = V[0];
            }
        } else {
            val_prev = V[i - 1];
        }

        result[i] = (val_current - val_prev);
    }

    return result;
}

// Sigmoid building block used by the paper's simulator:
// sigmoid(x, s) = 2/(1 + exp(-s*x)) - 1, steepness s = 3*pi.
static double vm_sigmoid(double x, double s) {
    return 2.0 / (1.0 + std::exp(-s * x)) - 1.0;
}

// cos-like (front/back) mask: +1 ahead, -1 behind, switching at +/- pi/2.
double VisionModel::maskCos(double phi) const {
    if(!m_mask_sigmoid) return std::cos(phi);
    const double s = 3.0 * M_PI;
    if(phi < 0.0) return  vm_sigmoid(phi + M_PI/2.0, s);
    return -vm_sigmoid(phi - M_PI/2.0, s);
}

// sin-like (left/right) mask: +1 to starboard, -1 to port, switching at 0
// and at +/- pi.
double VisionModel::maskSin(double phi) const {
    if(!m_mask_sigmoid) return std::sin(phi);
    const double s = 3.0 * M_PI;
    if(phi > -M_PI/2.0 && phi < M_PI/2.0) return vm_sigmoid(phi, s);
    if(phi <= -M_PI/2.0) return -vm_sigmoid(phi + M_PI, s);
    return -vm_sigmoid(phi - M_PI, s);
}

void VisionModel::compute(double vel_now, const std::vector<int>& V_now, double& dvel, double& dpsi) {
    int n = V_now.size();
    dvel = 0.0;
    dpsi = 0.0;

    if (n < 2) return;

    // At FOV=360 the domain is periodic: phi=-pi and phi=+pi are the same
    // physical direction (straight behind the agent). pSimVisionServer lays
    // its VPF bins out accordingly -- n bins of width fov/n tiling
    // [-fov/2, +fov/2) once, with bin 0 and bin n-1 *adjacent* across the
    // wrap, never identical (see SimVisionServer.cpp's degrees_per_bin).
    // Use the same threshold as dPhi_V_of() below so both treat "full FOV"
    // consistently.
    bool full_fov = (m_fov >= 359.9);

    std::vector<double> Phi(n);
    double fov_rad = m_fov * M_PI / 180.0;
    double start_phi = -fov_rad / 2.0;            // start at -FOV/2
    // Partial FOV: n points fenceposting [-FOV/2, +FOV/2] inclusive (both
    // edges are genuine, distinct directions bounding a blind spot).
    // Full FOV: n points tiling the circle once, fov/n apart, *not*
    // reaching +FOV/2 (which would duplicate the -FOV/2 sample at index 0
    // and give the "behind" direction double weight in the integrals
    // below).
    double d_phi = full_fov ? (fov_rad / n) : (fov_rad / (n - 1));

    for (int i = 0; i < n; i++) {
        // Current angle of the bin
        Phi[i] = start_phi + i * d_phi;
    }

    // The model assumes a *binary* projection field: V is the silhouette, so
    // dV/dphi is a delta train whose square counts blob edges (paper Eqs. 5/7,
    // and ABM's np.square(dPhi_V) over a binarized field). pSimVisionServer
    // emits exactly that, but uVPFMerger can be configured to emit a "density"
    // VPF where overlapping detections stack above 1 -- binarize here so both
    // sources mean the same thing to the model rather than silently scaling
    // the edge term by how many agents happen to overlap.
    std::vector<int> V_bin(n);
    for (int i = 0; i < n; i++) V_bin[i] = (V_now[i] > 0) ? 1 : 0;
    const std::vector<int>& V_use = V_bin;

    // Deriving over Phi
    std::vector<double> dPhi_V = dPhi_V_of(V_use);

    // Calculating series expansion of functional G
    std::vector<double> G_vel(n);
    std::vector<double> G_psi(n);
    // Spikey parts shall be handled separately because of numerical integration
    std::vector<double> G_vel_spike(n);
    std::vector<double> G_psi_spike(n);
    for (int i = 0; i < n; i++) {
        G_vel[i] = -V_use[i];                   //EDIT for EVENT-BASED CAMERA (with actual dt_V)
        G_psi[i] = -V_use[i];                   //EDIT for EVENT-BASED CAMERA (with actual dt_V)

        // (dV/dphi)^2, as in the paper's Eqs. 5/7. pSimVisionServer now emits a
        // binary VPF (the union of silhouettes, i.e. the occluded view), so
        // squaring is identical to the |dV/dphi| this used to use as a
        // workaround for the old density VPF -- and it is what the paper's own
        // simulator computes (ABM vf_supcalc.py: np.square(dPhi_V)).
        G_vel_spike[i] = dPhi_V[i] * dPhi_V[i];
        G_psi_spike[i] = dPhi_V[i] * dPhi_V[i];
    }

    double trapz_psi = 0.0;
    double sum_psi_spike = 0.0;

    double trapz_vel = 0.0;
    double sum_vel_spike = 0.0;

    // NOTE: the spike sums are deliberately NOT weighted by d_phi, unlike the
    // trapezoid terms above. For a binary V, dV/dphi is a delta train and
    // integral((dV/dphi)^2)dphi formally diverges; the paper's simulator
    // regularizes it as a plain per-edge unit count on a fixed-resolution
    // retina (ABM vf_supcalc.py: np.sum(...G_spike), no dPhi factor), and
    // alpha1 = beta1 = 0.09 are tuned for exactly that convention at
    // 1.125 deg/bin. Adding d_phi here -- or varying the angular resolution --
    // would silently rescale them, which is why the mission now holds the bin
    // width fixed across FOV columns (see launch.sh's RES_VAL).
    for (int i = 0; i < n; i++) {
        sum_psi_spike += maskSin(Phi[i]) * G_psi_spike[i];
        sum_vel_spike += maskCos(Phi[i]) * G_vel_spike[i];
    }

    if (full_fov) {
        // Periodic domain: the trapezoidal rule over equally spaced samples
        // of a periodic function collapses exactly to a plain Riemann sum
        // with uniform weight d_phi per sample -- no endpoints to special-
        // case (there are none; see the d_phi/Phi construction above).
        for (int i = 0; i < n; i++) {
            trapz_psi += maskSin(Phi[i]) * G_psi[i] * d_phi;
            trapz_vel += maskCos(Phi[i]) * G_vel[i] * d_phi;
        }
    } else {
        for (int i = 0; i < n - 1; i++) {
            double y_psi_i      = maskSin(Phi[i]) * G_psi[i];
            double y_psi_next   = maskSin(Phi[i+1]) * G_psi[i+1];
            trapz_psi += ((y_psi_i + y_psi_next) / 2.0) * d_phi;

            double y_vel_i      = maskCos(Phi[i]) * G_vel[i];
            double y_vel_next   = maskCos(Phi[i+1]) * G_vel[i+1];
            trapz_vel += ((y_vel_i + y_vel_next) / 2.0) * d_phi;
        }
    }

    dpsi = m_b0 * trapz_psi + m_b0 * m_b1 * sum_psi_spike;

    dvel = m_gam * (m_v0 - vel_now) + m_a0 * trapz_vel + m_a0 * m_a1 * sum_vel_spike;

}