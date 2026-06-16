/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_AvoidCollisionSail.cpp                               */
/*    DATE: Nov 18th 2006                                        */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#include <iostream>
#include <cmath>
#include <cstdlib>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "AOF_AvoidCollision.h"
#include "AOF_AvoidCollisionDepth.h"
#include "BHV_AvoidCollisionSail.h"
#include "OF_Coupler.h"
#include "OF_Reflector.h"
#include "RefineryCPA.h"
#include "BuildUtils.h"
#include "MBUtils.h"
#include "CPA_Utils.h"
#include "XYSegList.h"
#include "ZAIC_Vector.h"

using namespace std;


template <typename BaseAOF>
class AOF_WindWrapper : public BaseAOF {
private:
  IvPDomain m_gdomain;
  std::map<double, double> m_polar_map;
  double m_max_polar_speed;
  double m_app_wind;
  int m_crs_ix;

  // Calculates the [0.0, 1.0] multiplier based on the polar plot
  double calcWindMult(double crs) const {
    if(m_polar_map.empty() || m_max_polar_speed == 0.0) return 1.0;
    
    double rel_wind = crs - m_app_wind;
    if(rel_wind < -180.0) rel_wind += 360.0;
    if(rel_wind >  180.0) rel_wind -= 360.0;
    rel_wind = std::abs(rel_wind);

    if(m_polar_map.count(rel_wind)) {
      return m_polar_map.find(rel_wind)->second / m_max_polar_speed;
    }

    double lower_angle = 0.0, lower_spd = 0.0;
    double upper_angle = 180.0, upper_spd = 0.0;
    
    for(std::map<double, double>::const_iterator it = m_polar_map.begin(); it != m_polar_map.end(); ++it) {
      if(it->first < rel_wind) { 
        lower_angle = it->first; 
        lower_spd = it->second; 
      } else if (it->first > rel_wind) { 
        upper_angle = it->first; 
        upper_spd = it->second; 
        break; 
      }
    }
    
    double pct = (rel_wind - lower_angle) / (upper_angle - lower_angle);
    double interp_spd = lower_spd + (pct * (upper_spd - lower_spd));
    double mult = interp_spd / m_max_polar_speed;
    
    if(mult < 0.0) mult = 0.0;
    if(mult > 1.0) mult = 1.0;
    return mult;
  }

public:
  AOF_WindWrapper(IvPDomain gdomain, std::map<double, double> p_map, double max_spd, double app_wind)
    : BaseAOF(gdomain), m_gdomain(gdomain), m_polar_map(p_map), m_max_polar_speed(max_spd), m_app_wind(app_wind) 
  {
    m_crs_ix = gdomain.getIndex("course");
  }

  // Overrides standard MOOS evaluation to inject our multiplication
  virtual double evalBox(const IvPBox* b) const {
    // 1. Get the standard collision avoidance utility
    double base_util = BaseAOF::evalBox(b);
    
    // 2. Find the center course heading for this box
    int crs_low_idx  = b->pt(m_crs_ix, 0);
    int crs_high_idx = b->pt(m_crs_ix, 1);
    double crs_low   = m_gdomain.getVal(m_crs_ix, crs_low_idx);
    double crs_high  = m_gdomain.getVal(m_crs_ix, crs_high_idx);
    double crs_mid   = (crs_low + crs_high) / 2.0;
    
    // 3. True mathematical multiplication
    return base_util * calcWindMult(crs_mid);
  }
};

//-----------------------------------------------------------
// Procedure: Constructor

BHV_AvoidCollisionSail::BHV_AvoidCollisionSail(IvPDomain gdomain) : 
  IvPContactBehavior(gdomain)
{
  this->setParam("descriptor", "avoid_collision");
  this->setParam("build_info", "uniform_piece = discrete @ course:3,speed:3");
  this->setParam("build_info", "uniform_grid  = discrete @ course:9,speed:6");
  
  if(m_domain.hasDomain("depth"))
    m_domain = subDomain(m_domain, "course,speed,depth");
  else
    m_domain = subDomain(m_domain, "course,speed");
  
  m_collision_depth   = 0;

  m_completed_dist    = 500;
  m_pwt_outer_dist    = 200;
  m_pwt_inner_dist    = 50;
  m_min_util_cpa_dist = 10; 
  m_max_util_cpa_dist = 75; 
  m_pwt_grade         = "quasi";
  m_roc_max_dampen    = -2.0; 
  m_roc_max_heighten  = 2.0; 
  m_bearing_line_show = false;
  m_time_on_leg       = 120;  // Overriding the superclass default=60

  m_no_alert_request  = false;

  // Initialize state variables
  m_curr_closing_spd  = 0;
  m_total_evals       = 0;
  m_avoiding          = false;

  m_max_polar_speed = 0.0;
  m_last_polar_str = "";
  m_apparent_wind_heading = 0.0;

  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING, NAV_WIND_DIR_APP, AVOIDING, POLAR_PLOT");

  // Release 19.8 additions
  m_use_refinery   = false;
  m_check_plateaus = false;
  m_check_validity = false;
  m_pcheck_thresh  = 0.001;
  m_verbose        = false;
}

//-----------------------------------------------------------
// Procedure: setParam

bool BHV_AvoidCollisionSail::setParam(string param, string param_val) 
{
  if(IvPContactBehavior::setParam(param, param_val))
    return(true);

  double dval = atof(param_val.c_str());
  bool non_neg_number = (isNumber(param_val) && (dval >= 0));

  if(param == "pwt_inner_dist") {
    return(setMinPartOfPairOnString(m_pwt_inner_dist,
				    m_pwt_outer_dist, param_val));
  }  
  else if(param == "pwt_outer_dist") { 
    return(setMaxPartOfPairOnString(m_pwt_inner_dist,
				    m_pwt_outer_dist, param_val));
  }

  else if(param == "min_util_cpa_dist") {
    return(setMinPartOfPairOnString(m_min_util_cpa_dist,
				    m_max_util_cpa_dist, param_val));
  }    
  else if(param == "max_util_cpa_dist") {
    return(setMaxPartOfPairOnString(m_min_util_cpa_dist,
				    m_max_util_cpa_dist, param_val));
  }
  
  else if(param == "completed_dist")
    return(setNonNegDoubleOnString(m_completed_dist, param_val));
  else if((param == "contact_type_required") && (param_val != "")) 
    return(IvPContactBehavior::setParam("match_type", param_val));

  else if(param == "collision_depth") {
    if(dval <= 0)
      return(false);
    if(!m_domain.hasDomain("depth"))
      return(false);
    if(dval >= m_domain.getVarHigh("depth"))
      return(true);  // Should not be considered a config error
    m_collision_depth = dval;
    return(true);
  }  
  
  else if(param == "pwt_grade") {
    param_val = tolower(param_val);
    if((param_val!="linear") && (param_val!="quadratic") && 
       (param_val!="quasi"))
      return(false);
    m_pwt_grade = param_val;
    return(true);
  }  
  else if(param == "roc_max_heighten") {
    if(!non_neg_number)
      return(false);
    m_roc_max_heighten = dval;
    if(m_roc_max_dampen > m_roc_max_heighten)
      m_roc_max_dampen = m_roc_max_heighten;
    return(true);
  }  
  else if(param == "roc_max_dampen") {
    if(!non_neg_number)
      return(false);
    m_roc_max_dampen = dval;
    if(m_roc_max_heighten < m_roc_max_heighten)
      m_roc_max_heighten = m_roc_max_dampen;
    return(true);
  }  
  else if(param == "no_alert_request")
    return(setBooleanOnString(m_no_alert_request, param_val));
  else if(param == "verbose")
    return(setBooleanOnString(m_verbose, param_val));
  else if(param == "use_refinery")
    return(setBooleanOnString(m_use_refinery, param_val));
  else if(param == "check_plateaus")
    return(setBooleanOnString(m_check_plateaus, param_val));
  else if(param == "check_validity") 
    return(setBooleanOnString(m_check_validity, param_val));
  else if(param == "pcheck_thresh")
    return(setNonNegDoubleOnString(m_pcheck_thresh, param_val));

  // Safety check, in case user did not explicitly set completed dist
  if(m_completed_dist < m_pwt_outer_dist)
    m_completed_dist = m_pwt_outer_dist;
  
  return(false);
}


//-----------------------------------------------------------
// Procedure: onHelmStart()
//      Note: This function is called when the helm starts, even if,
//            especially if, the behavior is just a template at start
//            time to be spawned later. 
//      Note: An alert request will be sent to the contact manager if
//            the behavior is configured with templating enabled, and
//            an updates variable has been provided.  In the rare case
//            that the above is true but the user still does not want
//            an alert request generated, this can be done by setting
//            m_no_alert_request to true.

void BHV_AvoidCollisionSail::onHelmStart()
{
  if(m_no_alert_request || (m_update_var == "") || !m_dynamically_spawnable)
    return;

  string alert_templ = m_update_var + "=name=$[VNAME] # contact=$[VNAME]";
  string request = "id=" + getDescriptor();
  request += ", on_flag=" + alert_templ;
  request += ",alert_range=" + doubleToStringX(m_pwt_outer_dist,1);
  request += ", cpa_range=" + doubleToStringX(m_completed_dist,1);
  request = augmentSpec(request, getFilterSummary());
  
  postMessage("BCM_ALERT_REQUEST", request);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_AvoidCollisionSail::onIdleState() 
{
  if(!updatePlatformInfo())
    return;

  postRange();

  // See the Mod July 30th, 2022 comment below.
  if(!filterCheckHolds() || (m_contact_range >= (m_completed_dist * 1.1)))
    setComplete();  
}

//-----------------------------------------------------------
// Procedure: onInactiveState()

void BHV_AvoidCollisionSail::onInactiveState() 
{
  if(m_bearing_line_show)
    postViewableBearingLine(false);
}

//-----------------------------------------------------------
// Procedure: getInfo()

string BHV_AvoidCollisionSail::getInfo(string str) 
{
  if(str == "debug1")
    return(uintToString(m_total_evals));

  return("");
}

//-----------------------------------------------------------
// Procedure: parsePolarPlot()

bool BHV_AvoidCollisionSail::parsePolarPlot(string str)
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
  
// double BHV_AvoidCollisionSail::getPolarMultiplier(double candidate_heading)
// {
//   if(m_polar_map.empty() || m_max_polar_speed == 0.0) {
//     return(0);
//   }

//   double rel_wind = candidate_heading - m_apparent_wind_heading;
//   if(rel_wind < -180.0) rel_wind += 360.0;
//   if(rel_wind >  180.0) rel_wind -= 360.0;
//   rel_wind = abs(rel_wind);

//   if(m_polar_map.count(rel_wind)) {
//     return (m_polar_map[rel_wind] / m_max_polar_speed);
//   }

//   double lower_angle = 0.0, lower_spd = 0.0;
//   double upper_angle = 180.0, upper_spd = 0.0;

//   map<double, double>::iterator it;
//   for(it = m_polar_map.begin(); it != m_polar_map.end(); it++) {
//     if(it->first < rel_wind) {
//       lower_angle = it->first;
//       lower_spd = it->second;
//     } else if (it->first > rel_wind) {
//       upper_angle = it->first;
//       upper_spd = it->second;
//       break;
//     }
//   }

//   // Linear interpolation
//   double pct = (rel_wind - lower_angle) / (upper_angle - lower_angle);
//   double interp_spd = lower_spd + (pct * (upper_spd - lower_spd));

//   double utility_multiplier = interp_spd / m_max_polar_speed;

//   if(utility_multiplier < 0.0) utility_multiplier = 0.0;
//   if(utility_multiplier > 1.0) utility_multiplier = 1.0;

//   return(utility_multiplier);
// }

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_AvoidCollisionSail::onRunState() 
{
  bool ok_wind, ok_polar;

  m_apparent_wind_heading = getBufferDoubleVal("NAV_WIND_DIR_APP", ok_wind);
  string polar_str = getBufferStringVal("POLAR_PLOT", ok_polar);

  if(ok_polar && (polar_str != m_last_polar_str)) {
    if(parsePolarPlot(polar_str)) {
      m_last_polar_str = polar_str;
    } else {
      postWMessage("Failed to parse incoming POLAR_PLOT string");
    }
  }

  if (!ok_wind || m_polar_map.empty() || m_max_polar_speed == 0.0) {
    return(0);
  }

  m_total_evals = 0;
  if(!platformUpdateOK()) {
    postRange();
    return(0);
  }
  
  postRange();

  // Mod July 30th, 2022: Changed the actual comleted distance to be
  // 10pct greater than set completed distance. This is to ensure that
  // the behavior isn't spawned and killed immediately. If the
  // spawning and completed distances are exactly the same, then it's
  // possible that, if the entity doing the spawning and the helm
  // disagree slightly on the range between vessels due to timing, the
  // behavior will meet the complete threshold immediately, and the
  // entity doing the spawning may not notice and re-alert the helm
  // since from its perspective the range never crossedb back over the
  // threshhold for sending a new alert.
  
  if(!filterCheckHolds() || (m_contact_range >= (m_completed_dist * 1.1))) {
    setComplete();
    return(0);
  }
  
  m_relevance = getRelevance();
  if(m_relevance <= 0)
    return(0);

  IvPFunction *ipf = 0;

  if(m_collision_depth > 0)
    ipf = getAvoidDepthIPF();
  else
    ipf = getAvoidIPF();

  
  if(ipf) {
    //IvPFunction* penalized_ipf = applyWindPenalty(ipf);

    //if(penalized_ipf && (penalized_ipf != ipf)) {
    //  delete ipf;
    //  ipf = penalized_ipf;
    //}

    ipf->getPDMap()->normalize(0, 100);
    ipf->setPWT(m_relevance * m_priority_wt);
  }

  postViewableBearingLine();

  return(ipf);
}


//-----------------------------------------------------------
// Procedure: applyWindPenalty
// Purpose:   Multiplies the base collision avoidance objective function
//            by a wind mask to penalize headings inside the dead zone.

// IvPFunction* BHV_AvoidCollisionSail::applyWindPenalty(IvPFunction* base_ipf)
// {
//   if(!base_ipf) {
//     return(0);
//   }

//   ZAIC_Vector zaic_v(m_domain, "course");

//   int pts = m_domain.getVarPoints("course");
//   double delta = m_domain.getVarDelta("course");
//   double low = m_domain.getVarLow("course");

//   vector<double> wind_utilities;

//   for(int i = 0; i < pts; i++) {
//     double heading = low + (i * delta);

//     double multiplier = getPolarMultiplier(heading);

//     double util_val = multiplier * 100.0;
//     wind_utilities.push_back(util_val);
//   }

//   zaic_v.setDomainVals(wind_utilities);
//   IvPFunction* ipf_wind = zaic_v.extractIvPFunction();

//   if(!ipf_wind) {
//     return(base_ipf);
//   }

//   OF_Coupler coupler;
//   IvPFunction* final_ipf = coupler.couple(base_ipf, ipf_wind, 50, 50, "intersect");

//   delete ipf_wind;

//   return(final_ipf);
// }



//-----------------------------------------------------------
// Procedure: getAvoidIPF()

IvPFunction *BHV_AvoidCollisionSail::getAvoidIPF()
{
  // ===========================================================
  // Prepare the AOF to be passed to the Reflector
  // ===========================================================
  double min_util_cpa_dist = m_min_util_cpa_dist;
  if(m_contact_range <= m_min_util_cpa_dist)
    min_util_cpa_dist = (m_contact_range / 2);
  
  AOF_AvoidCollision aof(m_domain);
  aof.setCPAEngine(m_cpa_engine);
  aof.setOwnshipParams(m_osx, m_osy);
  aof.setContactParams(m_cnx, m_cny, m_cnh, m_cnv);
  aof.setParam("tol", m_time_on_leg);
  aof.setParam("collision_distance", min_util_cpa_dist);
  aof.setParam("all_clear_distance", m_max_util_cpa_dist);
  bool ok = aof.initialize();
  
  if(!ok) {
    postEMessage("Unable to init AOF_AvoidCollision.");
    return(0);
  }    
  
  OF_Reflector reflector(&aof, 1);
  m_domain = subDomain(m_domain, "course,speed");
  
  // ===========================================================
  // Utilize the Refinery to identify plateau regions
  // ===========================================================
  if(m_use_refinery) {
    RefineryCPA refinery;
    refinery.init(m_osx, m_osy, m_cnx, m_cny, m_cnh, m_cnv, m_time_on_leg,
		  m_min_util_cpa_dist, m_max_util_cpa_dist, m_domain,
		  &m_cpa_engine);
    refinery.setVerbose(m_verbose);
    
    vector<IvPBox> regions;
    regions = refinery.getRefineRegions();
    m_total_evals = refinery.getTotalQueriesCPA();

    for(unsigned int i=0; i<regions.size(); i++) {
      reflector.setParam("plateau_region", regions[i]);
      //string warnings = reflector.getWarnings();
      //postMessage("AVD_DEBUG", "["+intToString(i)+"]: "+warnings);
    }

    // Check plateaus code was used to verify correctness of plateaus
    // and remains for future validation.
    if(m_check_plateaus) {
      reflector.setParam("pcheck_thresh", m_pcheck_thresh);
      double worst_fail = reflector.checkPlateaus(true);
      bool ok_plateaus = (worst_fail == 0);

      if(m_verbose) {
	cout.precision(15);
	cout << "Worst fail: " << worst_fail << endl;
	cout << "ok_plateaus: " << boolToString(ok_plateaus) << endl;
	cout << "  BHV_AvoidCollisionSail checkPlateaus   START()" << endl;
	cout << "  BHV_AvoidCollisionSail plat_thresh: " <<
	  m_pcheck_thresh << endl;
	cout << "Plateaus OK: " << boolToString(ok_plateaus) << endl;
	cout << "  BHV_AvoidCollisionSail checkPlateaus     END()" << endl;
	postBoolMessage("PLATEAU_CHECK_OK", ok_plateaus);
	postRepeatableMessage("PLATEAU_LOGIC_CASE",
			      refinery.getLogicCase());
      }

      if(!ok_plateaus)
	postRepeatableMessage("PLATEAU_WORST_FAIL", worst_fail);
    }
  }

  // ===========================================================
  // Build the IvP Function
  // ===========================================================
  reflector.create(m_build_info);
  IvPFunction *ipf = reflector.extractIvPFunction();

  m_total_evals += reflector.getTotalEvals();
  
  string warnings = reflector.getWarnings();
  if(warnings != "")
    postMessage("AVD_DEBUG", warnings);

  if(m_check_validity) {
    bool valid_ipf = true;
    if(ipf) 
      valid_ipf = ipf->valid();
 
    if(m_verbose)
      cout << boolToString(valid_ipf) << endl;
    postBoolMessage("VALID_CHECK_OK", valid_ipf);
  }
    
  return(ipf);
}

//-----------------------------------------------------------
// Procedure: getAvoidDepthIPF()

IvPFunction *BHV_AvoidCollisionSail::getAvoidDepthIPF()
{
  AOF_AvoidCollisionDepth aof(m_domain);
  bool ok = true;
  ok = ok && aof.setParam("osy", m_osy);
  ok = ok && aof.setParam("osx", m_osx);
  ok = ok && aof.setParam("osh", m_osh);
  ok = ok && aof.setParam("osv", m_osv);
  ok = ok && aof.setParam("cny", m_cny);
  ok = ok && aof.setParam("cnx", m_cnx);
  ok = ok && aof.setParam("cnh", m_cnh);
  ok = ok && aof.setParam("cnv", m_cnv);
  ok = ok && aof.setParam("tol", m_time_on_leg);
  ok = ok && aof.setParam("collision_distance", m_min_util_cpa_dist);
  ok = ok && aof.setParam("all_clear_distance", m_max_util_cpa_dist);
  ok = ok && aof.setParam("collision_depth", m_collision_depth);
  ok = ok && aof.initialize();
 
  //double roc = aof.getROC();
  //postMessage("ROC", roc);

  if(!ok) {
    postEMessage("Unable to init AOF_AvoidCollision.");
    return(0);
  }    
  OF_Reflector reflector(&aof, 1);
  
  unsigned int index = (unsigned int)(m_domain.getIndex("depth"));
  unsigned int disc_val = m_domain.getDiscreteVal(index, m_collision_depth, 1);
  
  IvPBox region_bot = domainToBox(m_domain);
  region_bot.pt(index,0) = disc_val;
  
  IvPBox region_top = domainToBox(m_domain);
  region_top.pt(index,1) = disc_val-1;
  
  reflector.setParam("uniform_amount", "1");
  reflector.setParam("refine_region", region_top);
  reflector.setParam("refine_piece", "discrete @ course:10,speed:5,depth:100");
  
  reflector.setParam("refine_region", region_bot);
  reflector.setParam("refine_piece", region_bot);
  reflector.create();
  IvPFunction *ipf = reflector.extractIvPFunction();
  
  m_total_evals = reflector.getTotalEvals();
  
  string warnings = reflector.getWarnings();
  postMessage("AVD_STATUS", warnings);

  return(ipf);
}


//-----------------------------------------------------------
// Procedure: getRelevance
//            Calculate the relevance first. If zero-relevance, 
//            we won't bother to create the objective function.

double BHV_AvoidCollisionSail::getRelevance()
{
  // First declare the range of relevance values to be calc'ed
  double min_dist_relevance = 0.0;
  double max_dist_relevance = 1.0;
  double rng_dist_relevance = max_dist_relevance - min_dist_relevance;
  
  m_contact_range = hypot((m_osx - m_cnx),(m_osy - m_cny));
  m_curr_closing_spd = closingSpeed(m_osx, m_osy, m_osv, m_osh,
				    m_cnx, m_cny, m_cnv, m_cnh);

  postMessage("AVD_DEBUG", "In getRelevance: " + doubleToString(m_contact_range));
  if(m_contact_range >= m_pwt_outer_dist) {
    return(0);
  }

  double dpct, drange;
  if(m_contact_range <= m_pwt_inner_dist)
    dpct = max_dist_relevance;
  
  // Note: drange should never be zero since either of the above
  // conditionals would be true and the function would have returned.
  drange = (m_pwt_outer_dist - m_pwt_inner_dist);
  dpct = (m_pwt_outer_dist - m_contact_range) / drange;
  
  // Apply the grade scale to the raw distance
  double mod_dpct = dpct; // linear case
  if(m_pwt_grade == "quadratic")
    mod_dpct = dpct * dpct;
  else if(m_pwt_grade == "quasi")
    mod_dpct = pow(dpct, 1.5);

  double d_relevance = (mod_dpct * rng_dist_relevance) + min_dist_relevance;

  return(d_relevance);  
}

//-----------------------------------------------------------
// Procedure: postRange()

void BHV_AvoidCollisionSail::postRange()
{
  // Sanity check: Postings made to variables that contain contact
  // name may be disabled. Set post_per_contact_info=true to enable.
  // By default this is false.
  if(!postingPerContactInfo())
    return;
  
  // Round the speed a bit first so to reduce the number of posts 
  // to the db which are based on change detection.
  double cls_speed = snapToStep(m_curr_closing_spd, 0.1);
  postMessage(("CLSG_SPD_AVD_" + m_contact), cls_speed);
    
  // Post to integer precision unless very close to contact
  if(m_contact_range <= 10)
    postMessage(("RANGE_AVD_" + m_contact), m_contact_range);
  else
    postIntMessage(("RANGE_AVD_" + m_contact), m_contact_range);
}

