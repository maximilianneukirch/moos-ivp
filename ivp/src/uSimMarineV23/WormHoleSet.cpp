/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: WormHoleSet.cpp                                      */
/*    DATE: Jan 21st 2021                                        */
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

#include <cstdlib>
#include <cmath>
#include <iostream>
#include "WormHoleSet.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

//----------------------------------------------------------------
// Constructor()

WormHoleSet::WormHoleSet()
{
  // Init config vars
  m_tunnel_time = 10;  // seconds

  // Init state vars
  m_worm_hole_state  = "normal";    // or fading, emerging
  m_worm_hole_tstamp = 0;           
  m_transparency     = 1;           // Range [0,1]

  m_inx = 0;   // Observed wormhole entry point
  m_iny = 0;

  m_cooldown_active = false;
  m_land_x = 0;
  m_land_y = 0;
  m_min_clear_dist = 0;

  m_prev_x = 0;
  m_prev_y = 0;
  m_has_prev = false;
}

//----------------------------------------------------------------
// Procedure: setMinClearDist()

void WormHoleSet::setMinClearDist(double dist)
{
  if(dist < 0)
    dist = 0;
  m_min_clear_dist = dist;
}

//----------------------------------------------------------------
// Procedure: setTunnelTime()

void WormHoleSet::setTunnelTime(double tunnel_time)
{
  if(tunnel_time < 0)
    tunnel_time = 0;

  m_tunnel_time = tunnel_time;  
}

//----------------------------------------------------------------
// Procedure: setTransparency()

void WormHoleSet::setTransparency(double transp)
{
  if(transp < 0)
    transp = 0;
  else if(transp > 1)
    transp = 1;
  
  m_transparency = transp;
}

//----------------------------------------------------------------
// Procedure: addWormHoleConfig()

bool WormHoleSet::addWormHoleConfig(std::string str)
{
  string tag = tokStringParse(str, "tag", ',', '=');
  if(tag == "")
    return(false);
  tag = tolower(tag);

  unsigned int whix = 0;
  bool new_worm_hole = true;
  for(unsigned int i=0; i<m_worm_hole_tags.size(); i++) {
    if(tag == m_worm_hole_tags[i]) {
      whix = i;
      new_worm_hole = false;
    }
  }
  if(new_worm_hole) {
    WormHole new_worm_hole(tag);
    m_worm_holes.push_back(new_worm_hole);
    m_worm_hole_tags.push_back(tag);
    whix = m_worm_holes.size() - 1;
  }

  WormHole worm_hole = m_worm_holes[whix];

  vector<string> svector = parseStringZ(str, ',', "{");
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];

    cout << "param:[" << param << "], value:[" << value << "]" << endl;
    
    if(param == "madrid_poly") {
      if(isBraced(value))
	value = stripBraces(value);
      else if(!strBegins(value, "pts="))
	value = "pts={" + value + "}";
      cout << "  value:[" << value << "]" << endl; 

      XYPolygon poly = string2Poly(value);
      poly.set_label(tag + "_madrid");
      poly.set_color("fill", "white");
      poly.set_label_color("invisible");

      poly.set_transparency(0.1);
      poly.set_color("edge", "invisible");
      poly.set_color("vertex", "invisible");

      if(!poly.is_convex())
	return(false);
      worm_hole.setMadridPoly(poly);
    }
    else if(param == "weber_poly") {
      if(isBraced(value))
	value = stripBraces(value);
      else if(!strBegins(value, "pts="))
	value = "pts={" + value + "}";
      cout << "  value:[" << value << "]" << endl; 

      XYPolygon poly = string2Poly(value);

      poly.set_label(tag + "_weber");
      poly.set_color("fill", "white");
      poly.set_label_color("invisible");

      poly.set_transparency(0.1);
      poly.set_color("edge", "invisible");
      poly.set_color("vertex", "invisible");

      if(!poly.is_convex())
	return(false);
      worm_hole.setWeberPoly(poly);
    }
    else if(param == "connection") {
      value = tolower(value);
      if((value != "both") && (value != "from_madrid")
	 && (value != "from_weber"))
	return(false);
      worm_hole.setConnectionType(value);
    }
    else if(param == "delay") {
      if(!isNumber(value))
	return(false);
      double dval = atof(value.c_str());
      if(dval < 0)
	return(false);
      worm_hole.setDelay(dval);
    }
    else if(param == "id_change") {
      value = tolower(value);
      if(!isBoolean(value))
	return(false);
      bool bval = (value == "true");
      worm_hole.setIDChange(bval);
    }
    else if(param != "tag")
      return(false);
  }

  m_worm_holes[whix] = worm_hole;
  
  cout << "returning true!" << endl;
  return(true);
}


//----------------------------------------------------------------
// Procedure: getConfigSummary()

vector<string> WormHoleSet::getConfigSummary() const
{
  vector<string> set_summary;
  string str = "Total WormHoles: " + uintToString(m_worm_holes.size());
  set_summary.push_back(str);
  
  for(unsigned int i=0; i< m_worm_holes.size(); i++) {
    WormHole worm_hole = m_worm_holes[i];
    vector<string> wh_summary = worm_hole.getConfigSummary();
    for(unsigned int j=0; j<wh_summary.size(); j++)
      set_summary.push_back(wh_summary[j]);
  }
  
  return(set_summary);
}


//----------------------------------------------------------------
// Procedure: getWormHole()

WormHole WormHoleSet::getWormHole(unsigned int ix)
{
  WormHole null_worm_hole;
  if(ix >= m_worm_holes.size())
    return(null_worm_hole);
  
  return(m_worm_holes[ix]);
}


//----------------------------------------------------------------
// Procedure: getWormHole()

WormHole WormHoleSet::getWormHole(string tag)
{
  WormHole null_worm_hole;

  // Sanity Check
  if(m_worm_holes.size() != m_worm_hole_tags.size())
    return(null_worm_hole);

  for(unsigned int i=0; i<m_worm_hole_tags.size(); i++) {
    if(tag == m_worm_hole_tags[i])
      return(m_worm_holes[i]);
  }

  return(null_worm_hole);
}


//----------------------------------------------------------------
// Procedure: apply()
//   Returns: true if ownship position was transported

bool WormHoleSet::apply(double curr_time, double osx, double osy,
			double& newx, double& newy)
{
  // Remember this tick's (pre-transport) position for next call's
  // segment-crossing check, before anything below can overwrite it via a
  // transport. Captured unconditionally, every call, regardless of state.
  double prev_x = m_prev_x;
  double prev_y = m_prev_y;
  bool   has_prev = m_has_prev;
  m_prev_x = osx;
  m_prev_y = osy;
  m_has_prev = true;

  // Case 1: First handle where ownship is emerging from a wormhole
  // In this case the only task is to adjust the transparency
  if(m_worm_hole_state == "emerging") {
    double delta_time = curr_time - m_worm_hole_tstamp;

    if(delta_time > (m_tunnel_time/2)) {
      m_worm_hole_state  = "normal";
      m_worm_hole_tstamp = curr_time;
      m_transparency = 1;
    }
    else {
      double pct = delta_time / (m_tunnel_time/2);
      m_transparency = pct;
    }
    return(false);
  }

  // Case 2: First encounter with a wormhole
  if(m_worm_hole_state == "normal") {
    // Distance-based cooldown: don't even look for a new entry until
    // ownship has moved clear of where it last landed (see header comment
    // on m_min_clear_dist for why this must be distance-, not time-based).
    if(m_cooldown_active) {
      double dist = hypot(osx - m_land_x, osy - m_land_y);
      if(dist < m_min_clear_dist)
	return(false);
      m_cooldown_active = false;
    }

    string worm_hole_tag = findWormHoleEntry(prev_x, prev_y, has_prev, osx, osy);
    if(worm_hole_tag == "")
      return(false);
    else {
      m_worm_hole_state  = "fading";
      m_worm_hole_tstamp = curr_time;
      m_worm_hole_tag    = worm_hole_tag;
      m_inx = osx;
      m_iny = osy;
    }
  }

  // Case 3: Ownship is fading into a wormhole
  if(m_worm_hole_state == "fading") {
    double delta_time = curr_time - m_worm_hole_tstamp;
    if(delta_time >= (m_tunnel_time/2)) {
      m_worm_hole_state  = "emerging";
      m_worm_hole_tstamp = curr_time;
      m_transparency = 0;

      WormHole worm_hole = getWormHole(m_worm_hole_tag);
      string connection_type = worm_hole.getConnectionType();
      if(connection_type == "from_weber") {
	worm_hole.crossPositionWeberToMadrid(m_inx,m_iny, newx,newy);
	m_cooldown_active = true;
	m_land_x = newx;
	m_land_y = newy;
	// The vehicle's real position is about to jump straight to
	// (newx,newy) -- next tick's segment must start fresh from there,
	// not span the artificial teleport gap (which would otherwise look
	// like a huge, spurious crossing of everything in between).
	m_prev_x = newx;
	m_prev_y = newy;
	return(true);
      }
      if(connection_type == "from_madrid") {
	worm_hole.crossPositionMadridToWeber(m_inx,m_iny, newx,newy);
	m_cooldown_active = true;
	m_land_x = newx;
	m_land_y = newy;
	m_prev_x = newx;
	m_prev_y = newy;
	return(true);
      }
    }
    else {
      double pct = delta_time / (m_tunnel_time/2);
      m_transparency = 1- pct;
      return(false);
    }
  }

  return(false);
}


//----------------------------------------------------------------
// Procedure: segmentCrossesBand()
//   Purpose: does the segment from (x0,y0) to (x1,y1) pass through an
//            axis-aligned band poly (long in one dimension, thin in the
//            other -- exactly the shape of a wormhole's madrid/weber
//            poly)? Checked as: does the segment's extent along the
//            band's thin (wrap) axis overlap the band at all, AND does
//            either endpoint's lateral coordinate fall within the band's
//            lateral extent. This catches a crossing regardless of how
//            large the single-tick step is, unlike plain point-in-polygon
//            containment, which can be skipped over entirely if ownship
//            jumps clean across a band between two consecutive ticks
//            (confirmed happening in practice under MOOSTimeWarp=20).

static bool segmentCrossesBand(const XYPolygon& poly,
				double x0, double y0, double x1, double y1)
{
  double minx = poly.get_min_x();
  double maxx = poly.get_max_x();
  double miny = poly.get_min_y();
  double maxy = poly.get_max_y();

  double dx = maxx - minx;
  double dy = maxy - miny;
  if((dx <= 0) || (dy <= 0))
    return(false);

  if(dy <= dx) {
    // Band is long in x, thin in y -- y is the wrap axis.
    double lo = (y0 < y1) ? y0 : y1;
    double hi = (y0 < y1) ? y1 : y0;
    if((hi < miny) || (lo > maxy))
      return(false);
    if(((x0 >= minx) && (x0 <= maxx)) || ((x1 >= minx) && (x1 <= maxx)))
      return(true);
    return(false);
  }
  else {
    // Band is long in y, thin in x -- x is the wrap axis.
    double lo = (x0 < x1) ? x0 : x1;
    double hi = (x0 < x1) ? x1 : x0;
    if((hi < minx) || (lo > maxx))
      return(false);
    if(((y0 >= miny) && (y0 <= maxy)) || ((y1 >= miny) && (y1 <= maxy)))
      return(true);
    return(false);
  }
}


//----------------------------------------------------------------
// Procedure: findWormHoleEntry()

string WormHoleSet::findWormHoleEntry(double prev_x, double prev_y, bool has_prev,
				       double osx, double osy)
{
  // Sanity Check
  if(m_worm_holes.size() != m_worm_hole_tags.size())
    return("");

  // Check each worm_hole, and if ownship is in entry polygon, or the
  // segment from its previous position crossed through it, then return
  // the tag of that worm_hole.
  for(unsigned int i=0; i<m_worm_holes.size(); i++) {

    WormHole worm_hole = m_worm_holes[i];

    string connection_type = worm_hole.getConnectionType();
    if(connection_type == "from_weber") {
      XYPolygon weber_poly = worm_hole.getWeberPoly();
      if(weber_poly.contains(osx, osy))
	return(m_worm_hole_tags[i]);
      if(has_prev && segmentCrossesBand(weber_poly, prev_x, prev_y, osx, osy))
	return(m_worm_hole_tags[i]);
    }
    if(connection_type == "from_madrid") {
      XYPolygon madrid_poly = worm_hole.getMadridPoly();
      if(madrid_poly.contains(osx, osy))
	return(m_worm_hole_tags[i]);
      if(has_prev && segmentCrossesBand(madrid_poly, prev_x, prev_y, osx, osy))
	return(m_worm_hole_tags[i]);
    }
  }

  return("");
}






