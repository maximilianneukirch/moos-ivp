#include <cmath>
#include "BHV_WindManeuverability.h"
#include "BuildUtils.h"
#include "MBUtils.h"
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
  m_current_hdg = 0;
  m_hdg_valid = false;

  m_max_polar_speed = 0;

  m_nogo_speed_frac = 0.15;
  m_nogo_penalty = -1000.0;
  m_hysteresis_deg = 30.0;
  m_hysteresis_bonus = 10.0;

  addInfoVars("NAV_WIND_DIR_APP");
  addInfoVars("NAV_HEADING");
}

//--------------------------------------------------------
// Procedure: setParam
bool BHV_WindManeuverability::setParam(string param, string val)
{
  param = tolower(param);

  if(IvPBehavior::setParam(param, val))
    return(true);

  if(param == "polar_plot")
    return(parsePolarPlot(val));
  else if(param == "nogo_speed_frac") {
    m_nogo_speed_frac = atof(val.c_str());
    return(true);
  }
  else if(param == "nogo_penalty") {
    m_nogo_penalty = atof(val.c_str());
    return(true);
  }
  else if(param == "hysteresis_deg") {
    m_hysteresis_deg = atof(val.c_str());
    return(true);
  }
  else if(param == "hysteresis_bonus") {
    m_hysteresis_bonus = atof(val.c_str());
    return(true);
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
// Procedure: interpSpeedFraction
// Returns interpolated speed / max_speed for rel_wind in [0,180]
double BHV_WindManeuverability::interpSpeedFraction(double rel_wind)
{
  if(m_polar_map.empty() || m_max_polar_speed <= 0.0)
    return(0.0);

  if(m_polar_map.count(rel_wind))
    return(m_polar_map[rel_wind] / m_max_polar_speed);

  double lower_angle = 0.0;
  double lower_spd = 0.0;
  double upper_angle = 180.0;
  double upper_spd = 0.0;

  map<double, double>::iterator it;
  for(it = m_polar_map.begin(); it != m_polar_map.end(); it++) {
    if(it->first < rel_wind) {
      lower_angle = it->first;
      lower_spd = it->second;
    }
    else if(it->first > rel_wind) {
      upper_angle = it->first;
      upper_spd = it->second;
      break;
    }
  }

  if(upper_angle <= lower_angle)
    return(lower_spd / m_max_polar_speed);

  double pct = (rel_wind - lower_angle) / (upper_angle - lower_angle);
  double interp_spd = lower_spd + (pct * (upper_spd - lower_spd));

  return(interp_spd / m_max_polar_speed);
}

//--------------------------------------------------------
// Procedure: getPolarUtility
// Returns utility for rel_wind in [0,180].
// - no-go headings -> m_nogo_penalty (negative well)
// - sailable headings -> rescaled [0,100]
double BHV_WindManeuverability::getPolarUtility(double rel_wind)
{
  if(m_polar_map.empty() || m_max_polar_speed <= 0.0)
    return(m_nogo_penalty);

  double speed_frac = interpSpeedFraction(rel_wind);

  if(speed_frac <= m_nogo_speed_frac)
    return(m_nogo_penalty);

  double denom = 1.0 - m_nogo_speed_frac;
  double rescaled = (denom > 0.0) ?
    ((speed_frac - m_nogo_speed_frac) / denom) : 1.0;

  if(rescaled < 0.0)
    rescaled = 0.0;
  if(rescaled > 1.0)
    rescaled = 1.0;

  return(rescaled * 100.0);
}

//--------------------------------------------------------
// Procedure: onRunState
IvPFunction* BHV_WindManeuverability::onRunState()
{
  bool ok = false;
  m_app_wind_dir = getBufferDoubleVal("NAV_WIND_DIR_APP", ok);
  m_wind_valid = ok;

  if(!m_wind_valid)
    return(0);

  m_current_hdg = getBufferDoubleVal("NAV_HEADING", ok);
  m_hdg_valid = ok;

  int crs_ix = m_domain.getIndex("course");
  int crs_pts = m_domain.getVarPoints("course");

  vector<double> domain_vec(crs_pts, 0.0);
  vector<double> utility_vec(crs_pts, 0.0);

  for(int i=0; i<crs_pts; i++) {
    double candidate_heading = m_domain.getVal(crs_ix, i);
    domain_vec[i] = candidate_heading;

    double rel_wind = candidate_heading - m_app_wind_dir;
    if(rel_wind < -180.0) rel_wind += 360.0;
    if(rel_wind >  180.0) rel_wind -= 360.0;
    rel_wind = fabs(rel_wind);

    double utility = getPolarUtility(rel_wind);

    // Apply hysteresis only on sailable headings; never inside the
    // no-go well where utility is negative.
    if((utility > 0.0) && m_hdg_valid) {
      double current_diff = fabs(angle180(candidate_heading - m_current_hdg));
      if(current_diff <= m_hysteresis_deg) {
        utility += m_hysteresis_bonus *
          (1.0 - (current_diff / m_hysteresis_deg));
      }
    }

    utility_vec[i] = utility;
  }

  ZAIC_Vector zaic_vec(m_domain, "course");
  zaic_vec.setDomainVals(domain_vec);
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