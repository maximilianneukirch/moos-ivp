#ifndef NODE_REPORT_AGGREGATOR_H
#define NODE_REPORT_AGGREGATOR_H

#include "MOOS/libMOOS/MOOSLib.h"
#include <map>
#include <string>

class NodeReportAggregator : public CMOOSApp {
public:
    NodeReportAggregator();
    ~NodeReportAggregator() {};

protected:
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();
    void RegisterVariables();

private:
    unsigned int m_target_vehicle_count;
    std::string m_vname_prefix;
    
    std::map<std::string, std::string> m_vehicle_reports;
};

#endif