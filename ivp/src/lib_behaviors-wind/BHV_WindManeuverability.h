#ifndef BHV_WIND_MANEUVER_HEADER
#define BHV_WIND_MANEUVER_HEADER

#include <string>
#include <map>
#include "IvPBehavior.h"

class BHV_WindManeuverability : public IvPBehavior {
public:
  BHV_WindManeuverability(IvPDomain);
  ~BHV_WindManeuverability() {}
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected:
  bool         parsePolarPlot(std::string str);
  double       getPolarUtility(double rel_wind);

private: 
  // State variables
  double       m_app_wind_dir;
  bool         m_wind_valid;

  // Polar plot mapping (Relative Wind Angle -> Speed/Utility)
  std::map<double, double> m_polar_map;
  double       m_max_polar_speed;
};

//--------------------------------------------------------
// Dynamic Loading Block
extern "C" {
  IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {
    return new BHV_WindManeuverability(domain);
  }
}

#endif