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
  double       interpSpeedFraction(double rel_wind);
  double       getPolarUtility(double rel_wind);

private:
  // State variables
  double       m_app_wind_dir;
  bool         m_wind_valid;
  double       m_current_hdg;
  bool         m_hdg_valid;

  // Polar plot mapping (Relative Wind Angle -> Speed)
  std::map<double, double> m_polar_map;
  double       m_max_polar_speed;

  // No-go configuration
  double       m_nogo_speed_frac;  // speed/max speed threshold for no-go
  double       m_nogo_penalty;     // strongly negative utility for no-go headings

  // Hysteresis to avoid tack thrashing (only outside no-go)
  double       m_hysteresis_deg;
  double       m_hysteresis_bonus;
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