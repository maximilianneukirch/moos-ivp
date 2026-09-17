/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: WormHoleSet.h                                        */
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

#ifndef WORM_HOLE_SET_HEADER
#define WORM_HOLE_SET_HEADER

#include <string>
#include <vector>
#include "WormHole.h"

class WormHoleSet
{
public:
  WormHoleSet();
  ~WormHoleSet() {}

 public: // Setters
  bool   addWormHoleConfig(std::string);
  void   setTunnelTime(double);
  void   setTransparency(double);
  void   setMinClearDist(double);

 public: // Getters
  WormHole  getWormHole(unsigned int);
  WormHole  getWormHole(std::string);

  double getTransparency() const {return(m_transparency);}
  double getTunnelTime() const   {return(m_tunnel_time);}
  
  unsigned int size() const {return(m_worm_holes.size());}

  std::vector<std::string> getConfigSummary() const;

 public: // Application  

  bool apply(double curr_time, double osx, double osy,
	     double& newx, double& newy);

 protected:
  std::string findWormHoleEntry(double prev_x, double prev_y, bool has_prev,
				 double osx, double osy);


 protected: // Config vars
  std::vector<WormHole>    m_worm_holes;
  std::vector<std::string> m_worm_hole_tags;
  double      m_tunnel_time;

 protected: // state vars (for particular vehicle)
  std::string m_worm_hole_state;
  std::string m_worm_hole_tag;
  double      m_worm_hole_tstamp;
  double      m_transparency;

  double      m_inx;
  double      m_iny;

  // Distance-based re-entry cooldown: a wormhole's landing (weber/madrid)
  // poly necessarily sits inside the *other* wormhole's own trigger poly
  // (they're mirror images spaced by the arena width), so immediately
  // after a transport, ownship is still literally standing inside another
  // trigger zone. findWormHoleEntry() only checks position, not heading,
  // so without this it can immediately re-trigger a reverse transport --
  // and whether that happens before or after ownship's own motion carries
  // it clear of the zone depends on speed vs. tick rate, which is exactly
  // why some vehicles reliably wrap and others get stuck oscillating or
  // escape as their speed happens to alias badly against the tick grid
  // (reproduced and confirmed standalone before this fix). A time-based
  // cooldown (wormhole_tunnel_time) has the same problem: it's still just
  // a fixed delay, so some speed will always alias against it. Gating on
  // *distance traveled since landing* instead is speed-independent: no
  // new wormhole trigger is considered until ownship has moved at least
  // m_min_clear_dist from where it landed.
  bool        m_cooldown_active;
  double      m_land_x;
  double      m_land_y;
  double      m_min_clear_dist;

  // Per-tick displacement can exceed the entry band's width -- at high
  // MOOSTimeWarp, or whenever real per-tick wall-clock time runs long
  // under CPU load (either one enlarges the *simulated* time elapsed
  // between consecutive Iterate() calls), a fast-enough vehicle can jump
  // clean over a band that was only checked via point-in-polygon
  // containment, landing on the far side without ever having been
  // "inside" it on any single sampled tick -- reproduced directly: nearly
  // every vehicle in a MOOSTimeWarp=20 test run escaped this way despite
  // the band-width/cooldown fixes above, which only address what happens
  // *after* a crossing is detected, not whether it's detected at all.
  // Tracking the previous tick's position and checking whether the
  // *segment* from there to the current position crosses a wormhole's
  // band (not just whether the current point lands inside it) is robust
  // regardless of step size, short of a single tick spanning more than
  // one full arena width (never realistic for any vehicle speed here).
  double      m_prev_x;
  double      m_prev_y;
  bool        m_has_prev;
};

#endif 






