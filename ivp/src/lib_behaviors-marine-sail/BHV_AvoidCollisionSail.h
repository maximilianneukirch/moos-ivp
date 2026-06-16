/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_AvoidCollisionSail.h                                 */
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
 
#ifndef BHV_AVOID_COLLISION_SAIL_HEADER
#define BHV_AVOID_COLLISION_SAIL_HEADER

#include "IvPContactBehavior.h"

class IvPDomain;
class BHV_AvoidCollisionSail : public IvPContactBehavior {
public:
  BHV_AvoidCollisionSail(IvPDomain);
  ~BHV_AvoidCollisionSail() {}

  void         onHelmStart();
  IvPFunction* onRunState();
  bool         setParam(std::string, std::string);
  void         onIdleState();
  void         onInactiveState();
  bool         isConstraint() {return(true);}

  std::string  getInfo(std::string);
  
 protected:
  double getRelevance();
  double getPriority();
  void   postRange();

  // Polar Plot
  std::map<double, double> m_polar_map;
  double m_max_polar_speed;
  std::string m_last_polar_str;
  double m_apparent_wind_heading;

  bool parsePolarPlot(std::string str);
  //double getPolarMultiplier(double candidate_heading);

  IvPFunction* getAvoidIPF();
  IvPFunction* getAvoidDepthIPF();
  //IvPFunction* applyWindPenalty(IvPFunction* base_ipf);
  
 private: // Configuration Parameters

  std::string m_pwt_grade;
  std::string m_contact_type_required;

  double m_completed_dist;
  double m_pwt_outer_dist;
  double m_pwt_inner_dist;
  double m_min_util_cpa_dist;
  double m_max_util_cpa_dist;
  double m_roc_max_heighten; // Rate of Closure w/ max relevance heightening
  double m_roc_max_dampen;   // Rate of Closure w/ max relevance dampening
  bool   m_no_alert_request;

  double m_collision_depth;

  // Release 19.8 additions
  bool   m_use_refinery;
  bool   m_check_plateaus;
  bool   m_check_validity;
  double m_pcheck_thresh;
  bool   m_verbose;
  
private:  // State Variables
  double m_curr_closing_spd;
  bool   m_avoiding;

  unsigned int m_total_evals;
};

#ifdef WIN32
	// Windows needs to explicitly specify functions to export from a dll
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_AvoidCollisionSail(domain);}
}
#endif

