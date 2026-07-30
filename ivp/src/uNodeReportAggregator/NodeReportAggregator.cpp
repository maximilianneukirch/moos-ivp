#include "NodeReportAggregator.h"
#include "MBUtils.h"
#include <vector>

using namespace std;

//---------------------------------------------------------
// Constructor
NodeReportAggregator::NodeReportAggregator() {
    m_target_vehicle_count = 1; 
    m_vname_prefix = "sim";
    m_in_var_name = "NODE_REPORT";
    m_out_var_name = "NODE_REPORTS_AGG";
}

//---------------------------------------------------------
// Procedure: OnStartUp()
bool NodeReportAggregator::OnStartUp() {
    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    
    if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
        STRING_LIST::iterator p;
        for(p = sParams.begin(); p != sParams.end(); p++) {
            string line  = *p;
            string param = tolower(biteStringX(line, '='));
            string value = line;

            if(param == "target_vehicle_count") {
                m_target_vehicle_count = atoi(value.c_str());
            } else if(param == "vname_prefix") {
                m_vname_prefix = value.c_str();
            } else if(param == "in_var_name") {
                m_in_var_name = value;
            } else if(param == "out_var_name") {
                m_out_var_name = value.c_str();
            }
        }
    }
    
    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()
bool NodeReportAggregator::OnConnectToServer() {
    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: RegisterVariables()
void NodeReportAggregator::RegisterVariables() {
    //m_Comms.Register("NODE_REPORT", 0);
    //m_Comms.Register("NODE_REPORT_LOCAL", 0);
    m_Comms.Register(m_in_var_name, 0);
}

//---------------------------------------------------------
// Procedure: OnNewMail()
bool NodeReportAggregator::OnNewMail(MOOSMSG_LIST &NewMail) {
    MOOSMSG_LIST::iterator p;
    for(p = NewMail.begin(); p != NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();

        //if(key == "NODE_REPORT" || key == "NODE_REPORT_LOCAL") {
        if(key == m_in_var_name) {
            string report = msg.GetString();
            
            string vname = "";
            string vx = "", vy = "", vhdg = "", vspd = "";
            
            vector<string> svector = parseString(report, ',');
            for(unsigned int i=0; i<svector.size(); i++) {
                string pair = svector[i];
                string param = biteStringX(pair, '=');
                string value = pair;
                
                if(param == "NAME")       vname = value;
                else if(param == "X")     vx = value;
                else if(param == "Y")     vy = value;
                else if(param == "HDG")   vhdg = value;
                else if(param == "SPD")   vspd = value;
            }

            // Filter for prefix
            if(vname.find(m_vname_prefix) == 0) {
                // Pack into small format: sim1,45.2,12.1,270.5
                string compact_state = vname + "," + vx + "," + vy + "," + vhdg + "," + vspd;
                
                m_vehicle_reports[vname] = compact_state;
            }
        }
    }
    return true;
}

//---------------------------------------------------------
// Procedure: Iterate()
bool NodeReportAggregator::Iterate() {
    if(m_vehicle_reports.size() >= m_target_vehicle_count) {
        
        string agg_string = "";
        
        // Concatenate all map values separated by a pipe delimiter
        for(auto const& [name, compact_data] : m_vehicle_reports) {
            agg_string += compact_data + "|";
        }
        
        // Strip the trailing pipe character
        if(!agg_string.empty()) {
            agg_string.pop_back(); 
        }

        m_Comms.Notify(m_out_var_name, agg_string);
        m_vehicle_reports.clear();
    }
    
    return true;
}

// result: sim1,45.2,12.1,270.5|sim2,45.2,12.1,270.5 ...

//------------------------------------------
// uNodeReportAggregator config block
//ProcessConfig = uNodeReportAggregator
//{
//  AppTick   = 4
//  CommsTick = 4
//  
//  target_vehicle_count = 10
//  vname_prefix = sim
//  in_var_name = NODE_REPORT
//  out_var_name = NODE_REPORTS_AGG
//}