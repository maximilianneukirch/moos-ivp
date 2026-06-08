#include <iostream>
#include <cmath>
#include "BHV_WindManeuverability.h"
#include "BuildUtils.h"
#include "ZAIC_Vector.h"
#include "AngleUtils.h"

using namespace std;

//--------------------------------------------------------
// Constructor
BHV_WindManeuverability::BHV_WindManeuverability(IvPDomain domain) :
  IvPBehavior(domain)
{
  this->setParam("descriptor", "wind_maneuver");
  
  m_domain = subDomain(m_domain, "course");

  m_app_wind_dir = 0;
  m_wind_valid = false;
  m_max_polar_speed = 0;

  // Apparent wind direction from the MOOSDB
  addInfoVars("NAV_WIND_DIR_APP"); 
}

//--------------------------------------------------------
// Procedure: setParam
bool BHV_WindManeuverability::setParam(string param, string val)
{
  if(IvPBehavior::setParam(param, val))
    return(true);

  if(param == "polar_plot") {
    return(parsePolarPlot(val));
  }
  return(false);
}

//--------------------------------------------------------
// Procedure: parsePolarPlot
// Example input: "0,0 : 45,50 : 90,100 : 180,80"
bool BHV_WindManeuverability::parsePolarPlot(string str)
{
  m_polar_map.clear();
  m_max_polar_speed = 0;

  vector<string> svector = parseString(str, ':');
  for(unsigned int i=0; i<svector.size(); i++) {
    string pair_str = svector[i];
    string angle_str = biteStringX(pair_str, ',');
    string speed_str = pair_str;

    if(!isNumber(angle_str) || !isNumber(speed_str))
      return(false);

    double angle = atof(angle_str.c_str());
    double speed = atof(speed_str.c_str());
    
    m_polar_map[angle] = speed;
    if(speed > m_max_polar_speed)
      m_max_polar_speed = speed;
  }
  return(m_polar_map.size() > 0);
}

//--------------------------------------------------------
// Procedure: getPolarUtility
// Interpolates the speed for a given relative wind angle (0 to 180)
// Returns a utility score from 0.0 to 100.0
double BHV_WindManeuverability::getPolarUtility(double rel_wind)
{
  if(m_polar_map.empty() || m_max_polar_speed == 0) return(0.0);

  // If exact match exists
  if(m_polar_map.count(rel_wind)) { // if rel_wind occurrs exactly in m_polar_map
    return (m_polar_map[rel_wind] / m_max_polar_speed) * 100.0;
  }

  // Find lower and upper bounds for interpolation
  double lower_angle = 0, lower_spd = 0;
  double upper_angle = 180, upper_spd = 0;
  
  map<double, double>::iterator it;
  for(it = m_polar_map.begin(); it != m_polar_map.end(); it++) {
    if(it->first < rel_wind) {
      lower_angle = it->first;
      lower_spd = it->second;
    } else if (it->first > rel_wind) {
      upper_angle = it->first;
      upper_spd = it->second;
      break; // Found the immediate upper bound
    }
  }

  // Linear Interpolation
  double pct = (rel_wind - lower_angle) / (upper_angle - lower_angle);
  double interp_spd = lower_spd + (pct * (upper_spd - lower_spd));

  return (interp_spd / m_max_polar_speed) * 100.0;
}

//--------------------------------------------------------
// Procedure: onRunState
IvPFunction* BHV_WindManeuverability::onRunState()
{
  // 1. Get the wind direction from the info buffer
  bool ok;
  m_app_wind_dir = getBufferDoubleVal("NAV_WIND_DIR_APP", ok);

  if(!ok) return(0); // Cannot generate function without wind

  // 2. Prepare the Vector array to hold the utility of all 360 headings
  int crs_ix = m_domain.getIndex("course");
  int crs_pts = m_domain.getVarPoints("course");

  //int crs_pts = m_domain.getVarPoints("course");
  vector<double> utility_vec(crs_pts, 0.0);

  // 3. Evaluate every candidate heading
  for(int i=0; i<crs_pts; i++) {
    double candidate_heading = m_domain.getVal(crs_ix, i);
    
    // Calculate relative wind angle [0, 180]
    double rel_wind = candidate_heading - m_app_wind_dir;
    if(rel_wind < -180) rel_wind += 360;
    if(rel_wind >  180) rel_wind -= 360;
    rel_wind = std::abs(rel_wind);

    // Get utility based on polar plot capability
    double utility = getPolarUtility(rel_wind);
    
    // Set the value in our vector
    utility_vec[i] = utility;
  }

  if(utility_vec.empty()) {
    postWMessage("FAIL: Utility Vec is empty!");
    return(0);
  }

  // 4. Build the IvPFunction using ZAIC_Vector
  vector<double> domain_vec(crs_pts, 0.0);
  for(int i=0; i<crs_pts; i++) {
    domain_vec[i] = m_domain.getVal(crs_ix, i);
  }

  // 5. Build the IvPFunction using ZAIC_Vector
  ZAIC_Vector zaic_vec(m_domain, "course");
  // X-axis (the 360 headings)
  zaic_vec.setDomainVals(domain_vec); 
  // Y-axis (the 360 utility scores we calculated)
  zaic_vec.setRangeVals(utility_vec); 

  if(zaic_vec.stateOK() == false) {
    postWMessage("ZAIC problems: " + zaic_vec.getWarnings());
    return(0);
  }
  
  IvPFunction *ipf = zaic_vec.extractIvPFunction();

  if(!ipf) {
    postWMessage("FAIL: ZAIC_Vector extracted a NULL function");
    return(0);
  }

  ipf->setPWT(m_priority_wt);

  return(ipf);
}

//---------------------------------------------------------------
// Empty placeholder functions

void BHV_WindManeuverability::onSetParamComplete() {}
void BHV_WindManeuverability::onCompleteState() {}
void BHV_WindManeuverability::onIdleState() {}
void BHV_WindManeuverability::onHelmStart() {}
void BHV_WindManeuverability::postConfigStatus() {}
void BHV_WindManeuverability::onRunToIdleState() {}
void BHV_WindManeuverability::onIdleToRunState() {}
