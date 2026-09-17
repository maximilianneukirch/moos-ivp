#ifndef VISION_MODEL_H
#define VISION_MODEL_H

#include <vector>
#include <cmath>

class VisionModel {
public:
    VisionModel();
    ~VisionModel() {};

    void setParams(double a0, double a1, double b0, double b1, double v0, double gam, double fov);

    // Angular masks applied to the visual field before integration.
    // "trig"    -- cos(phi) / sin(phi), as printed in the paper's Eqs. 4-7.
    // "sigmoid" -- the boxcar-like sigmoid masks the paper's own simulator
    //              actually runs (ABM vf_supcalc.cos_sigmoid/sin_sigmoid with
    //              steepness 3*pi, used unconditionally on vf_agent's default
    //              non-verbose path). Same symmetry, but peripheral blobs keep
    //              full weight instead of being tapered to zero at +/-90 deg.
    void setMaskSigmoid(bool v) {m_mask_sigmoid = v;}

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
    bool   m_mask_sigmoid;

    // Mask values at angle phi, per m_mask_sigmoid.
    double maskCos(double phi) const;
    double maskSin(double phi) const;
};

#endif