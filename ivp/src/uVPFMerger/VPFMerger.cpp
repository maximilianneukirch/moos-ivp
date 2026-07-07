#include "VPFMerger.h"
#include "MBUtils.h"
#include <iostream>

using namespace std;

//---------------------------------------------------------
// Constructor
VPFMerger::VPFMerger() {
    // Default MOOS variables
    m_vpf_real_var = "VPF_REAL";
    m_vpf_sim_var  = "VPF_SIM";
    m_vpf_out_var  = "VPF";
    m_merge_mode   = "binary"; 
    
    m_real_updated = false;
    m_sim_updated  = false;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
bool VPFMerger::OnStartUp() {
    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    
    if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
        STRING_LIST::iterator p;
        for(p = sParams.begin(); p != sParams.end(); p++) {
            string line  = *p;
            string param = tolower(biteStringX(line, '='));
            string value = line;

            if(param == "vpf_real_var") {
                m_vpf_real_var = value;
            } else if(param == "vpf_sim_var") {
                m_vpf_sim_var = value;
            } else if(param == "vpf_out_var") {
                m_vpf_out_var = value;
            } else if(param == "merge_mode") {
                m_merge_mode = tolower(value);
            }
        }
    }
    
    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()
bool VPFMerger::OnConnectToServer() {
    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: RegisterVariables()
void VPFMerger::RegisterVariables() {
    m_Comms.Register(m_vpf_real_var, 0);
    m_Comms.Register(m_vpf_sim_var, 0);
}

//---------------------------------------------------------
// Helper: Convert comma-separated string to vector
std::vector<int> VPFMerger::stringToVPF(const std::string& s) {
    vector<string> s_tokens = parseString(s, ',');
    vector<int> vpf;
    for(const string& token : s_tokens) {
        vpf.push_back(atoi(token.c_str()));
    }
    return vpf;
}

//---------------------------------------------------------
// Helper: Convert vector back to comma-separated string
std::string VPFMerger::vpfToString(const std::vector<int>& vpf) {
    string s_vpf = "";
    for(size_t i = 0; i < vpf.size(); i++) {
        s_vpf += to_string(vpf[i]);
        if(i < vpf.size() - 1) {
            s_vpf += ",";
        }
    }
    return s_vpf;
}

//---------------------------------------------------------
// Procedure: OnNewMail()
bool VPFMerger::OnNewMail(MOOSMSG_LIST &NewMail) {
    MOOSMSG_LIST::iterator p;
    for(p = NewMail.begin(); p != NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();

        if(key == m_vpf_real_var && msg.IsString()) {
            m_latest_real_vpf = stringToVPF(msg.GetString());
            m_real_updated = true;
        } else if(key == m_vpf_sim_var && msg.IsString()) {
            m_latest_sim_vpf = stringToVPF(msg.GetString());
            m_sim_updated = true;
        }
    }
    return true;
}

//---------------------------------------------------------
// Procedure: Iterate()
bool VPFMerger::Iterate() {

    if(m_latest_real_vpf.empty() || m_latest_sim_vpf.empty()) {
        return true;
    }

    // At least one has to be being updated
    if(!m_real_updated && !m_sim_updated) {
        return true;
    }

    // resolution mismatches
    if(m_latest_real_vpf.size() != m_latest_sim_vpf.size()) {
        cout << "uVPFMerger WARNING: Real and Sim VPF resolutions do not match!" << endl;
        m_real_updated = false;
        m_sim_updated = false;
        return true;
    }

    size_t n = m_latest_real_vpf.size();
    std::vector<int> merged_vpf(n, 0);

    for(size_t i = 0; i < n; i++) {
        if(m_merge_mode == "density") {
            // Density VPF (entries also > 1): Add both bins
            merged_vpf[i] = m_latest_real_vpf[i] + m_latest_sim_vpf[i];
        } else {
            // Binary VPF (original)
            merged_vpf[i] = (m_latest_real_vpf[i] > 0 || m_latest_sim_vpf[i] > 0) ? 1 : 0;
        }
    }

    m_Comms.Notify(m_vpf_out_var, vpfToString(merged_vpf));

    m_real_updated = false;
    m_sim_updated = false;

    return true;
}

//ProcessConfig = uVPFMerger
//{
//  AppTick   = 10
//  CommsTick = 10
//
//  // The MOOS variables to subscribe to
//  vpf_real_var = VPF_CAMERA   // E.g., The raw python output
//  vpf_sim_var  = VPF_SIMULATED // E.g., The incoming pSimVisionServer feed
//
//  // The final MOOS variable to publish (The helm expects 'VPF*')
//  vpf_out_var  = VPF          
//
//  // Set to 'binary' or 'density'
//  merge_mode   = density
//}