#include "BHV_VisFlocking.h"
#include "AngleUtils.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_Vector.h"
#include "ZAIC_SPD.h"
#include "ZAIC_HDG.h"
#include "OF_Coupler.h"
#include <cmath>
#include <iostream>
#include <algorithm>

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
  m_v0 = 1.0;
  m_gam = 0.1;
  fov = 175.0;

  // Phase 0.2/0.3: overridable from the .bhv
  m_turn_lookahead = 2.5;    // s; reproduces the old turn_gain=10.0 * dt at AppTick=4 (dt=0.25 s)
  m_max_speed_error = 1.0;   // m/s; max |internal - actual| speed gap (anti-windup)
  m_speed_cap_factor = 2.0;  // speed command cap = factor * v0

  m_current_speed = 0.0;
  m_current_heading = 0.0;

  // To measure time difference between iterations
  m_last_time = 0.0;

  m_internal_speed = 0.0;
  m_is_initialized = false;

  m_last_desired_heading = 0.0;
  m_have_last_desired_heading = false;

  m_max_polar_speed = 0.0;
  m_last_polar_str = "";
  m_apparent_wind_heading = 0.0;
  m_wind_received = false;

  addInfoVars("NAV_HEADING, NAV_SPEED, NAV_WIND_DIR_APP, POLAR_PLOT, " + m_vpf_var_name);
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
  } else if(param == "v0"){
    m_v0 = atof(val.c_str());
    return true;
  } else if(param == "gam"){
    m_gam = atof(val.c_str());
    return true;
  }
  else if(param == "turn_lookahead"){
    m_turn_lookahead = atof(val.c_str());
    return true;
  }
  else if(param == "max_speed_error"){
    m_max_speed_error = atof(val.c_str());
    return true;
  }
  else if(param == "speed_cap_factor"){
    m_speed_cap_factor = atof(val.c_str());
    return true;
  }
  return false;
}

//---------------------------------------------------------------
// Empty placeholder functions

void BHV_VisFlocking::onSetParamComplete() {}
void BHV_VisFlocking::onCompleteState() {}
void BHV_VisFlocking::onIdleState() {}
void BHV_VisFlocking::onHelmStart() {}
void BHV_VisFlocking::postConfigStatus() {}
void BHV_VisFlocking::onRunToIdleState() {}
void BHV_VisFlocking::onIdleToRunState() {}

//---------------------------------------------------------------
// Procedure: parsePolarPlot()
bool BHV_VisFlocking::parsePolarPlot(string str)
{
  m_polar_map.clear();
  m_max_polar_speed = 0.0;

  vector<string> svector = parseString(str, ':');
  for(unsigned int i = 0; i < svector.size(); i++) {
    string pair_str = svector[i];
    string angle_str = biteStringX(pair_str, ',');
    string speed_str = pair_str;

    if(!isNumber(angle_str) || !isNumber(speed_str))
      return(false);

    double angle = atof(angle_str.c_str());
    double speed = atof(speed_str.c_str());

    m_polar_map[angle] = speed;
    if(speed > m_max_polar_speed) {
      m_max_polar_speed = speed;
    }
  }

  return(m_polar_map.size() > 0);
}

//--------------------------------------------------------
// Procedure: getPolarMultiplier
// Purpose: Calculates relative wind and interpolates utility [0.0, 1.0]

double BHV_VisFlocking::getPolarMultiplier(double candidate_heading)
{
  if(m_polar_map.empty() || m_max_polar_speed == 0.0) {
    return(1.0); // Fail open if no valid polar plot exists
  }

  double rel_wind = candidate_heading - m_apparent_wind_heading;
  if(rel_wind < -180.0) rel_wind += 360.0;
  if(rel_wind >  180.0) rel_wind -= 360.0;
  rel_wind = fabs(rel_wind);

  if(m_polar_map.count(rel_wind)) {
    return (m_polar_map[rel_wind] / m_max_polar_speed);
  }

  double lower_angle = 0.0, lower_spd = 0.0;
  double upper_angle = 180.0, upper_spd = 0.0;

  map<double, double>::iterator it;
  for(it = m_polar_map.begin(); it != m_polar_map.end(); it++) {
    if(it->first < rel_wind) {
      lower_angle = it->first;
      lower_spd = it->second;
    } else if (it->first > rel_wind) {
      upper_angle = it->first;
      upper_spd = it->second;
      break;
    }
  }

  // Linear interpolation
  double pct = (rel_wind - lower_angle) / (upper_angle - lower_angle);
  double interp_spd = lower_spd + (pct * (upper_spd - lower_spd));

  // [0.0, 1.0] multiplier
  double utility_multiplier = interp_spd / m_max_polar_speed;

  if(utility_multiplier < 0.0) utility_multiplier = 0.0;
  if(utility_multiplier > 1.0) utility_multiplier = 1.0;

  return(utility_multiplier);
}

//---------------------------------------------------------------
// Procedure: onRunState

IvPFunction *BHV_VisFlocking::onRunState()
{
  // -----------------------------------------------------------------------
  // Wind-penalty part

  //bool ok_polar = false;
  //if (getBufferVarUpdated("POLAR_PLOT")) {
  //  string polar_str = getBufferStringVal("POLAR_PLOT", ok_polar);
  //  if(ok_polar && (polar_str != m_last_polar_str)) {
  //    if(parsePolarPlot(polar_str)) {
  //      m_last_polar_str = polar_str;
  //    } else {
  //      postWMessage("Failed to parse incoming POLAR_PLOT string");
  //    }
  //  }
  //}
//
//if (getBufferVarUpdated("NAV_WIND_DIR_APP")) {
//  bool ok_wind = false;
//  m_apparent_wind_heading = getBufferDoubleVal("NAV_WIND_DIR_APP", ok_wind);
//  if (ok_wind) {
//    m_wind_received = true;
//  }
//}
//
//if (!m_wind_received || m_polar_map.empty() || m_max_polar_speed == 0.0) {
//  return(0); // Cannot safely navigate without wind data
//}

  // -----------------------------------------------------------------------
  // VisFlocking part

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

  // TODO: INTERSECTION, APPLY EXPLORE/BEHAVE HERE

  // Part 3: Calculate dv and dpsi as by Mezey et. al.
  double dv = 0.0;
  double dpsi = 0.0;

  // No visual information: keep the last desired heading so the boat holds a
  // straight course instead of integrating tiny turn noise.
  bool all_zero_vpf = (!vpf.empty() &&
                       std::all_of(vpf.begin(), vpf.end(),
                                   [](int x){return x == 0;}));

  // Phase 0.1: always compute. An all-zero VPF already yields
  // dv = gam*(v0 - v) and dpsi = 0, i.e. the paper's "no visual input"
  // relaxation back to v0. Skipping it here used to let a boat that lost the
  // flock coast at its current speed forever.
  // set params order: a0, a1, b0, b1, fov
  m_vision_model.setParams(a0, a1, b0, b1, m_v0, m_gam, fov);
  m_vision_model.compute(m_current_speed, vpf, dv, dpsi);

  // TODO: DIFFERENTIATE BETWEEN FULL VPF and PARTIAL VPF (edge-wrapping)

  // Convert dpsi from rad/s to deg/s
  ////double dpsi_deg = dpsi * (180.0 / M_PI);

  // 2. Größeren Hebel für den Regler nutzen (z.B. 3 Sekunden in die Zukunft)
  //double lookahead_time = 0.5;

  // 3. Vorzeichen umkehren (MOOS-Kompass-Logik)
  ////double desired_heading = m_current_heading - (dpsi_deg * dt);

  // 4. Calculate desired values
  // Heading
  //double desired_heading = m_current_heading + (dpsi_deg * dt);

  double desired_heading = m_current_heading;

  if(all_zero_vpf) {
    if(m_have_last_desired_heading)
      desired_heading = m_last_desired_heading;
  } else {
    // Phase 0.2: dpsi is a rate (rad/s), so a fixed lookahead horizon in
    // seconds keeps the turn authority independent of the helm tick rate.
    double raw_heading = m_current_heading + radToDegrees(dpsi * m_turn_lookahead);
    desired_heading = angle360(raw_heading);
  }

  while(desired_heading >= 360.0) desired_heading -= 360.0;
  while(desired_heading < 0.0)    desired_heading += 360.0;
  m_last_desired_heading = desired_heading;
  m_have_last_desired_heading = true;

  // Speed
  if (!m_is_initialized) {
    m_internal_speed = m_current_speed;
    m_is_initialized = true;
  }

  //m_internal_speed += (dv * dt);
  m_internal_speed += (dv * 2.5);

  // Desired speed clamped to [0, speed_cap_factor * v0]
  if(m_internal_speed > (m_speed_cap_factor * m_v0)) m_internal_speed = (m_speed_cap_factor * m_v0);
  if(m_internal_speed < 0.0) m_internal_speed = 0.0;

  // Phase 0.3 anti-windup: keep the integrator near the hull's actual speed.
  // External forces (sail, motor, current) push actual speed around; without
  // this clamp the integral drifts away and the command saturates at the cap
  // even though the boat is nowhere near it.
  double speed_err = m_internal_speed - m_current_speed;
  if(speed_err > m_max_speed_error)
    m_internal_speed = m_current_speed + m_max_speed_error;
  if(speed_err < -m_max_speed_error)
    m_internal_speed = m_current_speed - m_max_speed_error;

  double desired_speed = m_internal_speed;


  postMessage("DEBUG_DESIRED_HEADING", desired_heading);
  postMessage("DEBUG_DESIRED_SPEED", desired_speed);

  //-----------------------------------------------------------
  // Build function with ZAIC

  // SPEED
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(desired_speed);
  spd_zaic.setPeakWidth(0.2);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.0);
  //ZAIC_SPD spd_zaic(m_domain, "speed");
  //spd_zaic.setMedSpeed(desired_speed);
  //spd_zaic.setLowSpeed(0.1);
  //spd_zaic.setHghSpeed(desired_speed + 0.4);
  //spd_zaic.setLowSpeedUtil(50);
  //spd_zaic.setHghSpeedUtil(50);
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  // HEADING (wind-aware)
  int crs_ix = m_domain.getIndex("course");
  int crs_pts = m_domain.getVarPoints("course");
  vector<double> domain_vec(crs_pts, 0.0);
  vector<double> utility_vec(crs_pts, 0.0);

  //for (const auto& entry : m_polar_map){
  //    cout << entry.first << ":" << entry.second << " ";
  //}
  //cout << endl;

  for(int i = 0; i < crs_pts; i++) {
    double h = m_domain.getVal(crs_ix, i);
    domain_vec[i] = h;

    // Base Utility
    // Drops from 100 at the desired_heading to 0 at +/- 90 degrees away
    double diff = fabs(angle180(h - desired_heading));
    double base_util = 0.0;
    if(diff <= 90.0) {
      base_util = 100.0 * (1.0 - (diff / 90.0));
    }

    // Wind Penalty Multiplier [0.0 to 1.0]
    //double wind_mult = getPolarMultiplier(h);

    //if (int(h) % 45 == 0) {
    //  cout << h << ":-:" << wind_mult << endl;
    //}

    // Multiplication
    //utility_vec[i] = base_util * wind_mult;
    utility_vec[i] = base_util;
  }

  //ZAIC_Vector crs_zaic(m_domain, "course");
  //crs_zaic.setDomainVals(domain_vec);
  //crs_zaic.setRangeVals(utility_vec);

  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(desired_heading);
  crs_zaic.setPeakWidth(0.0); // +/- 0°
  crs_zaic.setBaseWidth(180.0); // Outside of 180° is the utility dropped to 0
  crs_zaic.setSummitDelta(0.0);
  crs_zaic.setValueWrap(true); // wrap 360 to 0

  if(crs_zaic.stateOK() == false) {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

  // Couple both functions
  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 0.5, 0.5);

  ivp_function->setPWT(m_priority_wt);

  // Speicher freigeben
  //if(crs_ipf) delete(crs_ipf);
  //if(spd_ipf)   delete(spd_ipf);

  return(ivp_function);
  //postWMessage("LOOOOOL");
}
