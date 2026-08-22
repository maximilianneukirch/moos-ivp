/*#include "MOOS/libMOOS/Thirdparty/AppTickConfig/AppTickConfig.h"*/
/************************************************************/
/* NAME: Maximilian von Neukirch                            */
/* ORGN: Cyber-Physical Systems Group Universiät Konstanz   */
/* FILE: SimVisionServer.h                                  */
/* DATE: 28.04.2026                                         */
/************************************************************/

#ifndef SIM_VISION_SERVER_H
#define SIM_VISION_SERVER_H

#include "MOOS/libMOOS/MOOSLib.h"
#include "NodeRecord.h" // Standard MOOS-Utility for NODE_REPORTS
#include <map>
#include <string>
#include <vector>

// Current contact-info
struct ContactState {
    std::string name;
    double x;
    double y;
    double heading;
    double time;
    double speed;   // m/s -- lets us dead-reckon stale contacts (default 0)
};

class SimVisionServer : public CMOOSApp {
public:
    SimVisionServer();
    ~SimVisionServer() {};

protected:
    // Standard CMOOSApp overrides
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();

    void RegisterVariables();

    // Helpers for geometry
    double relAng(double x1, double y1, double x2, double y2);
    double angle360(double angle);
    double degToRad(double deg);
    double radToDeg(double rad);

private:
    /*// Own state
    double m_nav_x;
    double m_nav_y;
    double m_nav_heading;
    std::string m_vname;*/

    // Parameters for Simulation
    double m_fov;        // Field of View in degrees (e.g. 175.0)
    int    m_resolution; // Number of bins (e.g. 320)
    double m_len;        // Length of boat in meters
    double m_beam;       // Width of boat in meters

    std::string m_real_boat_prefix;

    // When non-empty, only synthesise the VPF for this observer (ownship on a
    // real boat). On a Pi this avoids re-computing every contact as an
    // observer (~N x wasted work). Empty (default) computes for all observers.
    std::string m_only_vname;

    // Store all contacts (sim and real)
    std::map<std::string, ContactState> m_contacts;
};

#endif