#ifndef VPF_MERGER_H
#define VPF_MERGER_H

#include "MOOS/libMOOS/MOOSLib.h"
#include <vector>
#include <string>

class VPFMerger : public CMOOSApp {
public:
    VPFMerger();
    ~VPFMerger() {};

protected:
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();
    void RegisterVariables();

private:
    std::vector<int> stringToVPF(const std::string& s);
    std::string vpfToString(const std::vector<int>& vpf);

    // Configuration parameters
    std::string m_vpf_real_var;
    std::string m_vpf_sim_var;
    std::string m_vpf_out_var;
    std::string m_merge_mode; // "binary" or "density"

    // State variables
    std::vector<int> m_latest_real_vpf;
    std::vector<int> m_latest_sim_vpf;
    
    bool m_real_updated;
    bool m_sim_updated;
};

#endif