#include "NodeReportDistributor.h"
#include "MBUtils.h"
#include <vector>

using namespace std;

//---------------------------------------------------------
// Constructor
NodeReportDistributor::NodeReportDistributor() {
    m_in_var_name = "BOAT_REPORT";
    m_out_var_name = "NODE_REPORT";
    m_extra_args = ""; 
}

//---------------------------------------------------------
// Procedure: OnStartUp()
bool NodeReportDistributor::OnStartUp() {
    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    
    if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
        STRING_LIST::iterator p;
        for(p = sParams.begin(); p != sParams.end(); p++) {
            string line  = *p;
            string param = tolower(biteStringX(line, '='));
            string value = line;

            if(param == "extra_args") {
                m_extra_args = value;
            } else if(param == "in_var_name") {
                m_in_var_name = value;
            } else if(param == "out_var_name") {
                m_out_var_name = value;
            }
        }
    }
    
    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()
bool NodeReportDistributor::OnConnectToServer() {
    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: RegisterVariables()
void NodeReportDistributor::RegisterVariables() {
    m_Comms.Register("NODE_REPORTS_AGG", 0);
}

//---------------------------------------------------------
// Procedure: OnNewMail()
bool NodeReportDistributor::OnNewMail(MOOSMSG_LIST &NewMail) {
    MOOSMSG_LIST::iterator p;
    for(p = NewMail.begin(); p != NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();

        if(key == m_in_var_name && msg.IsString()) {
            string agg_string = msg.GetString();
            
            // Split at '|'
            vector<string> boat_records = parseString(agg_string, '|');
            
            for(unsigned int i = 0; i < boat_records.size(); i++) {
                string record = boat_records[i];
                
                // vname,x,y,hdg
                vector<string> fields = parseString(record, ',');
                
                if(fields.size() == 4) {
                    string vname = fields[0];
                    string vx    = fields[1];
                    string vy    = fields[2];
                    string vhdg  = fields[3];
                    
                    // standard MOOS NODE_REPORT string
                    string node_report = "NAME=" + vname + 
                                         ",X=" + vx + 
                                         ",Y=" + vy + 
                                         ",HDG=" + vhdg + 
                                         ",TIME=" + doubleToStringX(MOOSTime());
                                         
                    if(!m_extra_args.empty()) {
                        node_report += "," + m_extra_args;
                    }

                    m_Comms.Notify(m_out_var_name, node_report);
                }
            }
        }
    }
    return true;
}

//---------------------------------------------------------
// Procedure: Iterate()
bool NodeReportDistributor::Iterate() {
    // Empty, since everything is handled in OnNewMail
    return true;
}

//------------------------------------------
// uNodeReportDistributor config block
//ProcessConfig = uNodeReportDistributor
//{
//  AppTick   = 4
//  CommsTick = 4
//  
//  extra_args = TYPE=kayak,LENGTH=1.1
//  in_var_name = NODE_REPORT_AGG
//  out_var_name = NODE_REPORT
//}