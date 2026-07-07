#ifndef NODE_REPORT_DISTRIBUTOR_H
#define NODE_REPORT_DISTRIBUTOR_H

#include "MOOS/libMOOS/MOOSLib.h"
#include <string>

class NodeReportDistributor : public CMOOSApp {
public:
    NodeReportDistributor();
    ~NodeReportDistributor() {};

protected:
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();
    void RegisterVariables();

private:
    std::string m_nrd_in_var;
    std::string m_nrd_out_var;
    std::string m_extra_args;
};

#endif