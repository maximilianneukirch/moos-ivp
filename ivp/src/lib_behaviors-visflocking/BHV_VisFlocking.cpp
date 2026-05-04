#include "BHV_VisFlocking.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include <cmath>
#include <iostream>

using namespace std;

//---------------------------------------------------------------
// Procedure: Constructor
BHV_VisFlocking::BHV_VisFlocking(IvPDomain domain) :
  IvPBehavior(domain)
{
  this->setParam("descriptor", "vision_flocking");

  // All distances are in meters, all speed in meters per second
  // Default values for configuration parameters 

  m_vpf_var_name = "VISION_PROJECTION_FIELD";
  a0 = 1.25;
  b0 = 1.75;
  a1 = 1.25;
  b1 = 1.75;
  fov = 175.0;
  resolution = 320;
  m_current_speed = 0.0;
  m_current_heading = 0.0;

  // To measure time difference between iterations
  m_last_time = 0.0;

  addInfoVars("NAV_HEADING, NAV_SPEED, " + m_vpf_var_name);
}

//---------------------------------------------------------------
// Procedure: setParam - handle behavior configuration parameters

bool BHV_VisFlocking::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  if(IvPBehavior::setParam(param, val)) return true;

  if(param == "vpf_variable") {
    m_vpf_var_name = val;
    addInfoVars(m_vpf_var_name);
    return true;
  }
  else if(param == "fov") {
    fov = atof(val.c_str());
    return true;
  }
  else if(param == "resolution") {
    resolution = atof(val.c_str());
    return true;
  }
  else if(param == "a0"){
    a0 = atof(val.c_str());
    return true;
  }
  else if(param == "a1"){
    a1 = atof(val.c_str());
    return true;
  }
  else if(param == "b0"){
    b0 = atof(val.c_str());
    return true;
  }
  else if(param == "b1"){
    b1 = atof(val.c_str());
    return true;
  }
  return false;
}

void BHV_VisFlocking::onSetParamComplete() {}
void BHV_VisFlocking::onCompleteState() {}
void BHV_VisFlocking::onIdleState() {}
void BHV_VisFlocking::onHelmStart() {}
void BHV_VisFlocking::postConfigStatus() {}
void BHV_VisFlocking::onRunToIdleState() {}
void BHV_VisFlocking::onIdleToRunState() {}

//---------------------------------------------------------------
// Procedure: onRunState

IvPFunction *BHV_VisFlocking::onRunState()
{
  // Part 1: Get vehicle data from InfoBuffer and post a 
  // warning if problem is encountered
  bool ok1, ok2, ok3;
  m_current_heading = getBufferDoubleVal("NAV_HEADING", ok1);
  m_current_speed   = getBufferDoubleVal("NAV_SPEED", ok2);
  string s_vpf      = getBufferStringVal(m_vpf_var_name, ok3);

  if(!ok1 || !ok2 || !ok3 || s_vpf.empty()) {
    postWMessage("No ownship SPEED/HEADING or VPF info in info_buffer.");
    return(0); // No data, no steering
  }

  // Get time difference to last Helm iteration
  double curr_time = getBufferCurrTime();
  double dt = 0.0;

  if (m_last_time > 0.0) {
    dt = curr_time - m_last_time;
  }

  m_last_time = curr_time;

  // If dt is too big (helm was in idle too long or stalled), cap dt
  if (dt > 1.0) {
    dt = 0.0; 
  }

  // Part 2: String to VPF-Array
  vector<string> s_tokens = parseString(s_vpf, ',');
  vector<int> vpf;
  for(const string& s : s_tokens) {
    vpf.push_back(atoi(s.c_str()));
  }

  // Part 3: Calculate dv and dpsi as by Mezey et. al.
  double dv = 0.0;
  double dpsi = 0.0;

  //computeStateVariables(vpf, dv, dpsi);
  // set params order: a0, a1, b0, b1, fov, res
  m_vision_model.setParams(a0, a1, b0, b1, fov, resolution);
  m_vision_model.compute(vpf, dv, dpsi);

  // 4. Calculate desired values
  double desired_heading = m_current_heading + (dpsi * dt);
  double desired_speed   = m_current_speed + (dv * dt);

  while(desired_heading >= 360.0) desired_heading -= 360.0;
  while(desired_heading < 0.0)    desired_heading += 360.0;
  
  /*
  if(desired_speed > 2.0) desired_speed = 2.0;
  if(desired_speed < 0.0) desired_speed = 0.0;*/

  //-----------------------------------------------------------
  // Procedure: buildFunctionWithZAIC

  // SPEED
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(desired_speed);
  spd_zaic.setPeakWidth(0.2);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.0);
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  
  // HEADING
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(desired_heading);
  crs_zaic.setPeakWidth(10.0); // +/- 10 Grad ist die "Wohlfühlzone"
  crs_zaic.setBaseWidth(45.0); // Außerhalb von 45 Grad fällt der Nutzen auf 0
  crs_zaic.setSummitDelta(0.0);
  crs_zaic.setValueWrap(true); // Wichtig für 360-0 Übergang
  if(crs_zaic.stateOK() == false) {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();
  
  // Couple both functions
  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);

  // Speicher freigeben
  //if(crs_ipf) delete(crs_ipf);
  //if(spd_ipf)   delete(spd_ipf);

  return(ivp_function);
  //postWMessage("LOOOOOL");
}