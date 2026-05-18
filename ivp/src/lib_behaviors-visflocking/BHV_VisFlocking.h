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

protected:
  std::string m_vpf_var_name;

  double a0;
  double a1;
  double b0;
  double b1;

  double m_v0;  // Active cruising speed, modified by the behavior
  double m_gam; // Relaxation factor

  double fov;
  int resolution;

  // Internal states
  double m_current_speed;
  double m_current_heading;

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