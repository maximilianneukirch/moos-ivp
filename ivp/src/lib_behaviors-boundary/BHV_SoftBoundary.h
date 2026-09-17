#ifndef BHV_SOFT_BOUNDARY_HEADER
#define BHV_SOFT_BOUNDARY_HEADER

#include <vector>
#include <string>
#include "IvPBehavior.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYPolygon.h"

class BHV_SoftBoundary : public IvPBehavior {
public:
    BHV_SoftBoundary(IvPDomain);
    ~BHV_SoftBoundary() {};

    bool         setParam(std::string, std::string);
    IvPFunction* onRunState();

protected:
    bool         updateInfoIn();
    void         postViewPolygon();

private:
    XYPolygon    m_boundary_polygon;
    double       m_max_range;       // Distance from boundary where the behavior starts to act
    double       m_min_range;       // Distance from boundary where the corrective delta is at its max
    double       m_max_delta;       // Max corrective heading delta (deg); 91 deg keeps the boat gliding alongside the boundary
    double       m_curve_power;     // Proximity-ramp exponent (1=linear, 2=quadratic, higher=gentler until close)
    double       m_peak_width;      // ZAIC_PEAK flat-top width (deg)
    double       m_base_width;      // ZAIC_PEAK base width (deg)
    double       m_summit_delta;    // ZAIC_PEAK rise from base to summit
    std::string  m_boundary_var;    // MOOS-variable for polygon definition
    double       m_min_speed;       // Minimum speed required to get any steering effect through rudder
};

#ifdef WIN32
	// Windows needs to explicitly specify functions to export from a dll
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_SoftBoundary(domain);}
}

#endif