#ifndef BHV_VISFLOCKING
#define BHV_VISFLOCKING

#include "IvPBehavior.h"
#include "VisionModel.h"
#include <vector>
#include <string>

class BHV_VisFlocking : public IvPBehavior {
public:
  BHV_VisFlocking(IvPDomain);
  ~BHV_VisFlocking() {};

  bool setParam(std::string, std::string);
  void onSetParamComplete();
  void onCompleteState();
  void onIdleState();
  void onHelmStart();
  void postConfigStatus();
  void onRunToIdleState();
  void onIdleToRunState();
  
  // Gets called every Tick
  IvPFunction* onRunState();

protected:
  // Math-Helper
  //void computeStateVariables(const std::vector<int>& vpf, double& out_dv, double& out_dpsi);
  VisionModel m_vision_model;

  // Polar Plot / Wind State
  std::map<double, double> m_polar_map;
  double m_max_polar_speed;
  std::string m_last_polar_str;
  double m_apparent_wind_heading;
  bool m_wind_received;

  bool parsePolarPlot(std::string str);
  double getPolarMultiplier(double candidate_heading);

protected:
  std::string m_vpf_var_name;

  double a0;
  double a1;
  double b0;
  double b1;

  double m_v0;  // Active cruising speed, modified by the behavior
  double m_gam; // Relaxation factor

  double fov;

  // Phase 0.2/0.3: overridable from the .bhv
  double m_turn_lookahead;   // s; heading += dpsi * lookahead (tick-rate independent)
  double m_max_speed_error;  // m/s; anti-windup clamp on |internal - actual| speed
  double m_speed_cap_factor; // speed command cap = factor * v0

  // States
  double m_current_speed;
  double m_current_heading;

  // Internal states for integration
  double m_internal_speed;
  bool m_is_initialized;

  // Hold the last commanded heading while visual input is all zeros.
  double m_last_desired_heading;
  bool m_have_last_desired_heading;

  // To measure time difference between iterations
  double m_last_time;
};

#ifdef WIN32
	// Windows needs to explicitly specify functions to export from a dll
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_VisFlocking(domain);}
}
#endif