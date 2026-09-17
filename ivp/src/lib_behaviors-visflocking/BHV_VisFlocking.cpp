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

  // Overridable from the .bhv
  m_time_scale = 1.0;        // model timesteps per second (1.0 = "1 ts == 1 s")
  m_mask_sigmoid = false;    // cos/sin masks unless the mission asks otherwise
  m_max_speed_error = 0.0;   // m/s; anti-windup clamp, 0 = off (see onRunState)
  m_speed_cap_factor = 3.0;  // speed command cap = factor * v0
  m_max_heading_error = 0.0; // deg; model-vs-hull heading cap, 0 = off

  m_current_speed = 0.0;
  m_current_heading = 0.0;

  // To measure time difference between iterations
  m_last_time = 0.0;

  m_internal_speed = 0.0;
  m_is_initialized = false;

  m_internal_heading = 0.0;
  m_have_internal_heading = false;

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
    // Deprecated no-op, accepted so older mission files (visflocking_test)
    // still load. The behavior now integrates dpsi into its own heading state
    // scaled by time_scale, instead of projecting a target heading a fixed
    // horizon ahead of the hull's current one.
    postWMessage("turn_lookahead is deprecated and ignored; use time_scale");
    return true;
  }
  else if(param == "mask_shape"){
    // "sigmoid" = the masks the paper's simulator actually uses; "trig" = the
    // cos/sin of the published equations. See VisionModel::setMaskSigmoid.
    string v = tolower(val);
    if(v == "sigmoid")   { m_mask_sigmoid = true;  return true; }
    if(v == "trig" || v == "cos_sin") { m_mask_sigmoid = false; return true; }
    return false;
  }
  else if(param == "time_scale"){
    m_time_scale = atof(val.c_str());
    return true;
  }
  else if(param == "max_heading_error"){
    m_max_heading_error = atof(val.c_str());
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

  // If dt is too big (helm was idle or stalled), clamp it rather than zeroing
  // it: zeroing froze both integrators on every hiccup.
  if (dt > 0.5) dt = 0.5;
  if (dt <= 0.0) return(0);   // first tick of a run; no time has passed yet

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

  // Phase 0.1: always compute. An all-zero VPF already yields
  // dv = gam*(v0 - v) and dpsi = 0, i.e. the paper's "no visual input"
  // relaxation back to v0. Skipping it here used to let a boat that lost the
  // flock coast at its current speed forever.
  // set params order: a0, a1, b0, b1, v0, gam, fov
  m_vision_model.setMaskSigmoid(m_mask_sigmoid);
  m_vision_model.setParams(a0, a1, b0, b1, m_v0, m_gam, fov);
  m_vision_model.compute(m_current_speed, vpf, dv, dpsi);

  // Raw model output (per model timestep) before the time-scale conversion
  // below -- useful to check whether a weak/flat response to some parameter is
  // the model itself vs. downstream steering/PID.
  postMessage("DEBUG_DPSI", dpsi);
  postMessage("DEBUG_DV", dv);

  // ---------------------------------------------------------------
  // Model time base.
  //
  // The paper's model is written in simulation timesteps, not seconds: an
  // agent advances v0 = 1 px per timestep, and gam, a0 and b0 are rates per
  // timestep (ABM vf_agent.update_agent_position: orientation += dphi,
  // velocity += dv, position += velocity). Mapping it onto a MOOS boat needs
  // to say how long one timestep is. One px is R_A/5.5 (the paper's agents
  // are 5.5 px in radius), so with this mission's 0.55 m agent radius one px
  // is 0.1 m, and a boat cruising at v0 = 1 m/s covers it in 0.1 s: one model
  // timestep = 0.1 s, i.e. time_scale = 10 model timesteps per second.
  //
  // Feeding the paper's parameters in without this conversion (what this
  // behavior used to do, via a fixed turn_lookahead and a hardcoded 2.5 on the
  // speed integrator) leaves every response an order of magnitude too weak
  // relative to how far the boat travels while responding.
  double model_dt = dt * m_time_scale;   // model timesteps elapsed this tick

  // Heading: integrate the model's own heading state, exactly as the paper's
  // agents do. An all-zero VPF gives dpsi = 0 and the state simply holds, so
  // no "freeze the last target" special case is needed.
  if(!m_have_internal_heading) {
    m_internal_heading = m_current_heading;
    m_have_internal_heading = true;
  }
  m_internal_heading = angle360(m_internal_heading + radToDegrees(dpsi * model_dt));

  // Optional guard for a hull that physically cannot keep up: don't let the
  // model state run arbitrarily far ahead of the boat (0 = disabled).
  if(m_max_heading_error > 0.0) {
    double hdg_err = angle180(m_internal_heading - m_current_heading);
    if(hdg_err >  m_max_heading_error)
      m_internal_heading = angle360(m_current_heading + m_max_heading_error);
    if(hdg_err < -m_max_heading_error)
      m_internal_heading = angle360(m_current_heading - m_max_heading_error);
  }

  double desired_heading = m_internal_heading;

  // Turn-rate demand and tracking error, for checking whether the vehicle can
  // actually fly the model's commanded turn rate.
  postMessage("DEBUG_DPSI_DEGPS", radToDegrees(dpsi * m_time_scale));
  postMessage("DEBUG_HDG_ERR", angle180(m_internal_heading - m_current_heading));

  // Speed
  if (!m_is_initialized) {
    m_internal_speed = m_current_speed;
    m_is_initialized = true;
  }

  m_internal_speed += (dv * model_dt);

  // Safety clamp only -- the paper's agents run unclamped (ABM's
  // VF_LIMIT_MOVEMENT defaults to 0); this just keeps a runaway out of the
  // helm's speed domain.
  if(m_internal_speed > (m_speed_cap_factor * m_v0)) m_internal_speed = (m_speed_cap_factor * m_v0);
  if(m_internal_speed < 0.0) m_internal_speed = 0.0;

  // Optional anti-windup, OFF by default (max_speed_error = 0). It used to be
  // on at 1.0 m/s and was the reason alpha0 had no effect at all: with the
  // vehicle's speed loop under-geared (pMarinePIDV22 speed_factor vs the
  // thrust_map), NAV_SPEED never reached v0, so gam*(v0 - v) stayed positive,
  // the integrator ramped every tick and this clamp pinned the command at
  // NAV_SPEED + 1.0 regardless of the social terms. Only enable it if the
  // hull genuinely cannot follow the commanded speed.
  if(m_max_speed_error > 0.0) {
    double speed_err = m_internal_speed - m_current_speed;
    if(speed_err > m_max_speed_error)
      m_internal_speed = m_current_speed + m_max_speed_error;
    if(speed_err < -m_max_speed_error)
      m_internal_speed = m_current_speed - m_max_speed_error;
    if(m_internal_speed < 0.0) m_internal_speed = 0.0;
  }

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
