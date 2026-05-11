#ifndef BHV_SOFT_BOUNDARY_HEADER
#define BHV_SOFT_BOUNDARY_HEADER

#include <vector>
#include <string>
#include "IvPBehavior.h"
#include "ZAIC_PEAK.h"

class BHV_SoftBoundary : public IvPBehavior {
public:
    BHV_SoftBoundary(IvPDomain);
    ~BHV_SoftBoundary() {};

    bool         setParam(std::string, std::string);
    IvPFunction* onRunState();

protected:
    bool         updateInfoIn();
    double       computeDistanceToBoundary();
    void         postViewPoint();

private:
    std::vector<std::pair<double, double>> m_boundary_polygon; // Liste der (x,y)-Punkte des Polygons
    double       m_max_range;       // Maximale Distanz, ab der die Abstoßung beginnt
    double       m_min_range;       // Minimale Distanz, bei der die Abstoßung am stärksten ist
    double       m_peak_width;      // Breite des Peaks (Stärke der Abstoßung)
    std::string  m_boundary_var;    // MOOS-Variable für die Polygon-Definition (optional)
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