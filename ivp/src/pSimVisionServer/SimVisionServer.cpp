/************************************************************/
/* NAME: Maximilian von Neukirch                            */
/* ORGN: Cyber-Physical Systems Group Universiät Konstanz   */
/* FILE: SimVisionServer.cpp                                */
/* DATE: 28.04.2026                                         */
/************************************************************/

#include "SimVisionServer.h"
#include "MBUtils.h" // For String-Parsing (tokStringParse etc.)
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;

//---------------------------------------------------------
// Constructor
SimVisionServer::SimVisionServer()
{
    // Standard values (can be overwritten by .moos file)
    //m_nav_x = 0.0;
    //m_nav_y = 0.0;
    //m_nav_heading = 0.0;
    //m_vname = "unknown";

    m_fov = 175.0;  // Field of fiew of used camera
    m_resolution = 320; // resolution of camera image (width)
    m_len = 5.0;   // e.g. 5 meters length
    m_beam = 1.5;  // e.g. 1.5 meters width

    // Optional FOV-cone visualization (off by default)
    m_post_fov_cones = false;
    m_fov_cone_radius = 50.0;      // meters
    m_fov_cone_color = "yellow";
    m_fov_cone_transparency = 0.5; // 0=opaque .. 1=invisible
    m_real_boat_prefix = "real";
    m_only_vname = "";  // empty = synthesise every observer (default)
}

//---------------------------------------------------------
// Procedure: OnStartUp()
bool SimVisionServer::OnStartUp()
{
    list<string> sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
        list<string>::iterator p;
        for(p = sParams.begin(); p != sParams.end(); p++) {
            string line  = *p;
            string param = tolower(biteStringX(line, '='));
            string value = line;

            if(param == "fov") {
                m_fov = atof(value.c_str());
            }
            else if(param == "resolution") {
                m_resolution = atoi(value.c_str());
            }
            else if(param == "boat_length") {
                m_len = atof(value.c_str());
            }
            else if(param == "boat_beam") {
                m_beam = atof(value.c_str());
            }
            else if(param == "real_boat_prefix") {
                m_real_boat_prefix = value.c_str();
            }
            else if(param == "post_fov_cones") {
                setBooleanOnString(m_post_fov_cones, value);
            }
            else if(param == "fov_cone_radius") {
                m_fov_cone_radius = atof(value.c_str());
            }
            else if(param == "fov_cone_color") {
                m_fov_cone_color = value.c_str();
            }
            else if(param == "fov_cone_transparency") {
                m_fov_cone_transparency = atof(value.c_str());
            }
            else if(param == "only_vname") {
                m_only_vname = value.c_str();
            }
        }
    }

    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer
bool SimVisionServer::OnConnectToServer()
{
    RegisterVariables();
    return true;
}

//---------------------------------------------------------
// Procedure: RegisterVariables
void SimVisionServer::RegisterVariables()
{
    //Register("NAV_X", 0);
    //Register("NAV_Y", 0);
    //Register("NAV_HEADING", 0);
    Register("NODE_REPORT", 0);       // Other boats
    Register("NODE_REPORT_LOCAL", 0); // Ground Truth (own boat)
}

//---------------------------------------------------------
// Procedure: OnNewMail
bool SimVisionServer::OnNewMail(MOOSMSG_LIST &NewMail)
{
    MOOSMSG_LIST::iterator p;
    for(p = NewMail.begin(); p != NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();

        /*if(key == "NAV_X") {
            m_nav_x = msg.GetDouble();
        }
        else if(key == "NAV_Y") {
            m_nav_y = msg.GetDouble();
        }
        else if(key == "NAV_HEADING") {
            m_nav_heading = msg.GetDouble();
        }*/

        if(key == "NODE_REPORT" || key == "NODE_REPORT_LOCAL") {
            string report = msg.GetString();
            string vname = tokStringParse(report, "NAME", ',', '=');
            
            // Save or update all contacts, except excluded vname prefixes
            if(vname != "") {
                ContactState contact;
                contact.name = vname;
                contact.x = atof(tokStringParse(report, "X", ',', '=').c_str());
                contact.y = atof(tokStringParse(report, "Y", ',', '=').c_str());
                contact.heading = atof(tokStringParse(report, "HDG", ',', '=').c_str());
                contact.speed = atof(tokStringParse(report, "SPD", ',', '=').c_str()); // 0 if absent
                contact.time = MOOSTime();
                
                m_contacts[vname] = contact;
            }
        }
    }
    return true;
}

//---------------------------------------------------------
// Procedure: Iterate()
// Calculation of one (boat specific) 1D Visual Projection Field (VPF) for every boat
bool SimVisionServer::Iterate()
{
    double degrees_per_bin = m_fov / (double)m_resolution;
    double current_time = MOOSTime();
    
    // OUTER LOOP; loop observer's positions
    std::map<std::string, ContactState>::iterator obs_it;
    for(obs_it = m_contacts.begin(); obs_it != m_contacts.end(); obs_it++){
        
        ContactState &observer = obs_it->second;

        // only_vname: synthesise just this one observer (ownship on a real
        // boat) instead of every contact -- avoids ~N x wasted work on the Pi.
        if(!m_only_vname.empty() && observer.name != m_only_vname) {
            continue;
        }

        // Filter old contacts (e.g. older than 5 seconds)
        if(current_time - observer.time > 5.0) continue;
        
        // 1. Initialize VPF for this observer with zeros
        std::vector<int> vpf(m_resolution, 0);

        // INNER LOOP; Iterate over all boats (the observer can see)
        std::map<std::string, ContactState>::iterator tgt_it;
        for(tgt_it = m_contacts.begin(); tgt_it != m_contacts.end(); tgt_it++) {
            
            ContactState &target = tgt_it->second;

            // Filter for old contacts (e.g. older than 5 seconds) and own contact
            if(observer.name == target.name || current_time - target.time > 5.0) {
                continue; 
            }

            // If own boat is a real boat, also filter all real boats, as these are detected by the real boat's camera
            bool observer_is_real = (observer.name.find(m_real_boat_prefix) == 0);
            bool target_is_real = (target.name.find(m_real_boat_prefix) == 0);

            if (observer_is_real && target_is_real) {
                continue;
            }

            // Dead-reckon the target forward along its heading since its last
            // report, so the VPF tracks moving sim boats between the ~1 Hz
            // contact updates. The observer (ownship) is fresh (high-rate
            // NODE_REPORT_LOCAL), so only the target is extrapolated. Cap at
            // 3 s (the 5 s staleness filter above stays as the outer bound).
            double age = current_time - target.time;
            if(age < 0.0) age = 0.0;
            if(age > 3.0) age = 3.0;
            double thr = degToRad(target.heading);
            double tx = target.x + target.speed * std::sin(thr) * age;
            double ty = target.y + target.speed * std::cos(thr) * age;

            // Get distance and bearing from observer to (extrapolated) target
            double dx = tx - observer.x;
            double dy = ty - observer.y;
            double dist = std::max(0.1, std::sqrt(dx*dx + dy*dy));

            double bearing = relAng(observer.x, observer.y, tx, ty);

            // Relative angle in own VPF (0 = straight ahead, positive = right)
            double rel_bearing = angle360(bearing - observer.heading);
            if(rel_bearing > 180.0) rel_bearing -= 360.0;

            // Aspect-Angle: How much the boat is turned compared to line of sight
            double gamma = degToRad(angle360(target.heading - bearing));

            // Apparent width (Boat Profile)
            // Width = L * |sin(gamma)| + Beam * |cos(gamma)|
            double w_app = m_len * std::abs(std::sin(gamma)) + m_beam * std::abs(std::cos(gamma));
        
            // Angle width (alpha), of the Blob in VPF
            double alpha = 2.0 * radToDeg(std::atan2(w_app / 2.0, dist));

            // 3. Mapping Angle to 1D-array (Bins)
            // Test if contact is (partially) inside FoV
            if(std::abs(rel_bearing) < (m_fov / 2.0 + alpha / 2.0)) {
                
                // Find center-bin (0 is moved to the left border of the FoV)
                // FoV goes from - FOV/2 to FOV/2
                // Bin 0 => -FOV/2, Bin 'resolution-1' => FOV/2
                int center_bin = (int)((rel_bearing + m_fov / 2.0) / degrees_per_bin);
                int bins_span = (int)((alpha / degrees_per_bin) / 2.0);
        
                for(int i = center_bin - bins_span; i <= center_bin + bins_span; ++i) {
                    if(i >= 0 && i < m_resolution) {
                        vpf[i] += 1; // 1 = Boat visible (occlusions are implicitly correct)
                        // Adding boat-blobs onto each other (+=), resulting in a density-VPF
                    }
                }
            }
        }
        
        // 4. Convert VPF-array into string for MOOSDB
        std::string s_vpf = "";
        for(size_t i = 0; i < vpf.size(); i++) {
            s_vpf += std::to_string(vpf[i]);
            if(i < vpf.size() - 1) {
                s_vpf += ",";
            }
        }
    
        // Publish vehicle specific VPF
        std::string name = observer.name;
        std::transform(name.begin(), name.end(), name.begin(),
               [](unsigned char c) { return std::toupper(c); });

        std::string var_name = "VPF_" + name;

        m_Comms.Notify(var_name, s_vpf);
    }

    //--------------------------------------------------
    // Optional: post an FOV cone (wedge) in front of every boat so that
    // pMarineViewer can draw the simulated camera's field of view. The cone
    // angle always matches m_fov. Wedges carry a stable per-boat label, so
    // repeated posts update the existing cone instead of adding new ones.
    if(m_post_fov_cones) {
        double half_fov = m_fov / 2.0;

        std::map<std::string, ContactState>::iterator fov_it;
        for(fov_it = m_contacts.begin(); fov_it != m_contacts.end(); fov_it++) {
            ContactState &contact = fov_it->second;

            // Same 5 s freshness window the VPF loop uses above
            bool fresh = (current_time - contact.time) <= 5.0;

            // A dead boat's cone was already removed once -- nothing to do
            if(!fresh && m_fov_cones_inactivated.count(contact.name) > 0)
                continue;

            std::string wedge = "x=" + doubleToStringX(contact.x, 3)
                + ",y=" + doubleToStringX(contact.y, 3)
                + ",rad_low=0"
                + ",rad_hgh=" + doubleToStringX(m_fov_cone_radius, 3)
                + ",ang_low=" + doubleToStringX(angle360(contact.heading - half_fov), 3)
                + ",ang_hgh=" + doubleToStringX(angle360(contact.heading + half_fov), 3)
                + ",label=FOV_" + contact.name
                + ",active=" + (fresh ? "true" : "false")
                + ",fill_color=" + m_fov_cone_color
                + ",fill_transparency=" + doubleToStringX(m_fov_cone_transparency, 2)
                + ",edge_color=" + m_fov_cone_color
                + ",edge_size=1";

            m_Comms.Notify("VIEW_WEDGE", wedge);

            if(fresh)
                m_fov_cones_inactivated.erase(contact.name);
            else
                m_fov_cones_inactivated.insert(contact.name);
        }
    }


    return true;
}

//---------------------------------------------------------
// Helpers

// Angle from (x1,y1) to (x2,y2) in Nav-format (0 = North, 90 = East)
double SimVisionServer::relAng(double x1, double y1, double x2, double y2) {
    double ang = std::atan2(y2 - y1, x2 - x1); 
    ang = 90.0 - radToDeg(ang);
    return angle360(ang);
}

// Normalize angles to [0, 360)
double SimVisionServer::angle360(double angle) {
    while(angle < 0.0) angle += 360.0;
    while(angle >= 360.0) angle -= 360.0;
    return angle;
}

double SimVisionServer::degToRad(double deg) {
    return deg * (M_PI / 180.0);
}

double SimVisionServer::radToDeg(double rad) {
    return rad * (180.0 / M_PI);
}