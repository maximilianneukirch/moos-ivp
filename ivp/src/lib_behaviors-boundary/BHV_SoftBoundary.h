#ifndef BHV_SOFT_BOUNDARY_HEADER
#define BHV_SOFT_BOUNDARY_HEADER

#include <vector>
#include <string>
#include "IvPBehavior.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"

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
    std::vector<std::pair<double, double>> m_boundary_polygon; // List of (x,y) points of polygon
    double       m_max_range;       // Max distance at where repulsion begins
    double       m_min_range;       // Min distance at where repulsion is biggest
    double       m_peak_width;      // ZAIC Peak width (= Strength of repulsion)
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