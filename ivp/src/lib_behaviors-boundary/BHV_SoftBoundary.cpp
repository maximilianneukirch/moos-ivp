#include <cmath>
#include <vector>
#include <string>
#include "BHV_SoftBoundary.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "XYPolygon.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_Vector.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

/*// Register behavior
IvPBehaviorCreator(BHV_SoftBoundary);*/

//------------------------------------------------------------------
// Procedure: Constructor

BHV_SoftBoundary::BHV_SoftBoundary(IvPDomain domain)
    : IvPBehavior(domain) {

    // All distances are in meters, all speed in meters per second
    // Default values for configuration parameters 

    m_max_range   = 20.0;
    m_min_range   = 5.0;
    m_peak_width  = 20.0;
    m_boundary_var = "BOUNDARY_POLYGON";
    m_min_speed = 1.0;

    // Wind state initialization
    m_max_polar_speed = 0.0;
    m_last_polar_str = "";
    m_apparent_wind_heading = 0.0;
    m_wind_received = false;
    addInfoVars("NAV_WIND_DIR_APP", "POLAR_PLOT");

    // Subscribe to required variables
    addInfoVars("NAV_X, NAV_Y");
    addInfoVars("NAV_HEADING");
    addInfoVars(m_boundary_var);

}

//------------------------------------------------------------------
// Procedure: setParam - handle behavior configuration parameters

bool BHV_SoftBoundary::setParam(string param, string value) {
    // Convert the parameter to lower case for more general matching
    param = tolower(param);

    if (IvPBehavior::setParam(param, value)) return true;

    if (param == "polygon") {
        // parameter has the form: polygon = pts={-25,50:29,50:29,-50:-25,-50}
        XYPolygon parsed_poly = string2Poly(value);

        if (parsed_poly.size() > 0) {
            m_boundary_polygon.clear();
            for (unsigned int i = 0; i < parsed_poly.size(); i++) {
                m_boundary_polygon.emplace_back(parsed_poly.get_vx(i), parsed_poly.get_vy(i));
            }
            return true;
        } else {
            postWMessage("setParam failed to parse static polygon: " + value);
            return false;
        }
    }
    else if (param == "max_range") {
        m_max_range = atof(value.c_str());
        return true;
    }
    else if (param == "min_range") {
        m_min_range = atof(value.c_str());
        return true;
    }
    else if (param == "peak_width") {
        m_peak_width = atof(value.c_str());
        return true;
    }
    else if (param == "boundary_var") {
        m_boundary_var = value;
        addInfoVars(m_boundary_var);
        return true;
    }
    else if (param == "min_speed") {
        m_min_speed = atof(value.c_str());
        return true;
    }
    return false;
}

//---------------------------------------------------------------
// Procedure: parsePolarPlot()
bool BHV_SoftBoundary::parsePolarPlot(string str)
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

double BHV_SoftBoundary::getPolarMultiplier(double candidate_heading)
{
  if(m_polar_map.empty() || m_max_polar_speed == 0.0) {
    return(1.0); // Fail open if no valid polar plot exists
  }

  double rel_wind = candidate_heading - m_apparent_wind_heading;
  if(rel_wind < -180.0) rel_wind += 360.0;
  if(rel_wind >  180.0) rel_wind -= 360.0;
  rel_wind = fabs(rel_wind);

  if(m_polar_map.count(rel_wind)) {
    //return (m_polar_map[rel_wind] / m_max_polar_speed);
    double exact_spd = m_polar_map[rel_wind];
    double deadzone_threshold = 0.20; // 20% of max speed defines the deadzone boundary
    double raw_mult = exact_spd / m_max_polar_speed;
    
    if (raw_mult >= deadzone_threshold) return 1.0;
    return raw_mult / deadzone_threshold;
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

  //double utility_multiplier = interp_spd / m_max_polar_speed;
  double raw_multiplier = interp_spd / m_max_polar_speed;
  
  double deadzone_threshold = 0.20; 
  double utility_multiplier = 0.0;

  if (raw_multiplier >= deadzone_threshold) {
      // Flat top: Preserve 100% utility for all viable sailing angles
      utility_multiplier = 1.0; 
  } else {
      // Valley walls: Smoothly ramp down to 0 inside the deadzone to avoid sharp mathematical cliffs
      utility_multiplier = raw_multiplier / deadzone_threshold; 
  }

  if(utility_multiplier < 0.0) utility_multiplier = 0.0;
  if(utility_multiplier > 1.0) utility_multiplier = 1.0;

  return(utility_multiplier);
}

//------------------------------------------------------------------
// Procedure: onRunState - called every helm itertion

IvPFunction* BHV_SoftBoundary::onRunState() {
    // Get wind and polar data
    bool ok_polar = false;
    if (getBufferVarUpdated("POLAR_PLOT")) {
        string polar_str = getBufferStringVal("POLAR_PLOT", ok_polar);
        if(ok_polar && (polar_str != m_last_polar_str)) {
        if(parsePolarPlot(polar_str)) {
            m_last_polar_str = polar_str;
        } else {
            postWMessage("Failed to parse incoming POLAR_PLOT string");
        }
        }
    }

    if (getBufferVarUpdated("NAV_WIND_DIR_APP")) {
        bool ok_wind = false;
        m_apparent_wind_heading = getBufferDoubleVal("NAV_WIND_DIR_APP", ok_wind);
        if (ok_wind) {
        m_wind_received = true;
        }
    }

    if (!m_wind_received || m_polar_map.empty() || m_max_polar_speed == 0.0) {
        return(0); // Cannot safely navigate without wind data
    }

    // Update Position, Polygon, etc.
    if (!updateInfoIn()) {
        return nullptr;
    }

    if (m_boundary_polygon.empty()) {
        return nullptr;
    }

    // Get polygon from points
    XYPolygon polygon;
    for (const auto& point : m_boundary_polygon) {
        polygon.add_vertex(point.first, point.second);
    }

    // Visualization of border
    postViewPolygon();

    // Calculate distance and closest point on boundary
    double closest_x, closest_y;
    polygon.closest_point_on_poly(m_osx, m_osy, closest_x, closest_y);
    double dist_to_boundary = hypot(m_osx - closest_x, m_osy - closest_y);

    bool is_inside = polygon.contains(m_osx, m_osy);

    // Return nullptr if inside polygon and outside force-field max_range (far from border)
    if (is_inside && dist_to_boundary >= m_max_range) {
        postMessage("VIEW_VECTOR", "label=escape_heading_" + m_us_name + ",clear=true");
        return nullptr;
    }

    // Visualization of border
    //postViewPolygon();

    // Compute dynamic weight (priority) of behavior
    // 0 at max_range, linear up to configured pwt at min_range
    double weight = 0.0;
    double base_pwt = getPriorityWt();
    double escape_heading = 0.0;

    if (!is_inside) {
        // Boat already violated boundary and is outside -> highes pwt, inverted escape_heading
        weight = base_pwt;
        escape_heading = relAng(m_osx, m_osy, closest_x, closest_y);
    } else {
        if (dist_to_boundary <= m_min_range) {
            weight = base_pwt;
        } else {
            // TODO: Check for m_max_range = m_min_range
            double fraction = (m_max_range - dist_to_boundary) / (m_max_range - m_min_range);
            weight = base_pwt * fraction;
        }
        
        // Compute escape heading (absolute 360-deg heading from boundary point to vehocle pos)
        double inward_heading = relAng(closest_x, closest_y, m_osx, m_osy);

        double tangent_offset = 0.0;
        double opt1 = angle360(inward_heading + tangent_offset);
        double opt2 = angle360(inward_heading - tangent_offset);

        
        double util_opt1 = getPolarMultiplier(opt1);
        double util_opt2 = getPolarMultiplier(opt2);
        
        if (util_opt1 > util_opt2 + 0.1) {
            escape_heading = opt1;
        } else if (util_opt2 > util_opt1 + 0.1) {
            escape_heading = opt2;
        //if (1 == 0) {
        } else {
            double diff1 = abs(m_osh - opt1);
            if (diff1 > 180.0) diff1 = 360.0 - diff1;
    
            double diff2 = abs(m_osh - opt2);
            if (diff2 > 180.0) diff2 = 360.0 - diff2;
    
            if (diff1 <= diff2) {
                escape_heading = opt2;
            } else {
                escape_heading = opt1;
            }            
        }
    }

    postEscapeVector(escape_heading);


    // HEADING (wind-aware)
    //int crs_ix = m_domain.getIndex("course");
    //int crs_pts = m_domain.getVarPoints("course");
    //vector<double> domain_vec(crs_pts, 0.0);
    //vector<double> utility_vec(crs_pts, 0.0);
//
    //for(int i = 0; i < crs_pts; i++) {
    //    double h = m_domain.getVal(crs_ix, i);
    //    domain_vec[i] = h;
    //    
    //    // Base Utility
    //    // Drops from 100 at the escape_heading to 0 at +/- 90 degrees away
    //    double diff = fabs(angle180(h - escape_heading));
    //    double base_util = 0.0;
    //    if(diff <= 90.0) {
    //    base_util = 100.0 * (1.0 - (diff / 90.0));
    //    }
//
    //    // Wind Penalty Multiplier [0.0 to 1.0]
    //    double wind_mult = getPolarMultiplier(h);
//
    //    // Multiplication
    //    utility_vec[i] = base_util * wind_mult;
    //}
//
    //ZAIC_Vector crs_zaic(m_domain, "course");
    //crs_zaic.setDomainVals(domain_vec);
    //crs_zaic.setRangeVals(utility_vec);

    
    // Old ZAIC
    // COURSE: Build function with ZAIC
    ZAIC_PEAK crs_zaic(m_domain, "course");
    crs_zaic.setSummit(escape_heading);
    crs_zaic.setPeakWidth(m_peak_width);
    crs_zaic.setBaseWidth(180.0);           // 180 deg of degradation
    crs_zaic.setSummitDelta(0.0);
    crs_zaic.setValueWrap(true);            // ValueWrap: wrap 360 to 0
    
    if(crs_zaic.stateOK() == false) {
        string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
        postWMessage(warnings);
        return(0);
    }

    IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();
    if(!crs_ipf)
        postWMessage("Failure on the CRS ZAIC");

    //// SPEED: Build function with ZAIC
    //ZAIC_PEAK spd_zaic(m_domain, "speed");
    //spd_zaic.setSummit(m_min_speed);
    //spd_zaic.setPeakWidth(0.2);
    //spd_zaic.setBaseWidth(1.0);
    //spd_zaic.setSummitDelta(0.0);
    //spd_zaic.setValueWrap(false);
//
    //IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
    //if(!spd_ipf)
    //    postWMessage("Failure on the SPD ZAIC");
//
    //OF_Coupler coupler;
    //IvPFunction *ipf = coupler.couple(crs_ipf, spd_ipf, 0.5, 0.5);

    if(!crs_ipf) {
        postWMessage("Failure on the CRS_SPD COUPLER");
    } else {
        // Apply dynamic weight
        crs_ipf->setPWT(weight);
    }

    return crs_ipf;
}

//------------------------------------------------------------------
// updateInfoIn: Update internal data

bool BHV_SoftBoundary::updateInfoIn() {
    bool ok_x, ok_y, ok_poly, ok_h;
    string polygon_str;

    // Vehicle's position from InfoBuffer
    m_osx = getBufferDoubleVal("NAV_X", ok_x);
    m_osy = getBufferDoubleVal("NAV_Y", ok_y);
    m_osh = getBufferDoubleVal("NAV_HEADING", ok_h);

    if (m_boundary_polygon.empty()) {
        // Polygon from InfoBuffer
        polygon_str = getBufferStringVal(m_boundary_var, ok_poly);
    }

    if (ok_poly && !polygon_str.empty()) {
        XYPolygon parsed_poly = string2Poly(polygon_str);
        
        if (parsed_poly.size() > 0) {
            m_boundary_polygon.clear();
            for (unsigned int i = 0; i < parsed_poly.size(); i++) {
                m_boundary_polygon.emplace_back(parsed_poly.get_vx(i), parsed_poly.get_vy(i));
            }
        } else {
            postWMessage("Failed to parse polygon from string: " + polygon_str);
        }
    }
    
    if (!ok_x || !ok_y) {
        postWMessage("No ownship NAV_X/NAV_Y info in info_buffer.");
        return false; // No data, no steering
    }else if (!ok_h) {
        postWMessage("No ownship NAV_HEADING info in info_buffer.");
        return false; // No data, no steering
    } else if (m_boundary_polygon.empty()) {
        postWMessage("No boundary polygon configured in .bhv or received from DB.");
        return false;
    }

    return true;
}

//------------------------------------------------------------------
// Procedure: postViewPoint - Visualize edge in pMarineViewer (optionally)

void BHV_SoftBoundary::postViewPolygon() {
    if (!m_boundary_polygon.empty()) {
        string spec = "pts={";

        for (size_t i = 0; i < m_boundary_polygon.size(); i++) {
            spec += doubleToString(m_boundary_polygon[i].first, 2) + "," +
                    doubleToString(m_boundary_polygon[i].second, 2);

            if (i < m_boundary_polygon.size() - 1) {
                spec += ":";
            }
        }

        spec += "}, label=soft_boundary,edge_color=red,edge_size=2,vertex_color=white,vertex_size=2";

        postMessage("VIEW_POLYGON", spec);
    }
}

//------------------------------------------------------------------
// Procedure: postEscapeVector - Visualize the chosen escape heading

void BHV_SoftBoundary::postEscapeVector(double heading) {
    // Construct the vector string for pMarineViewer
    string spec = "x=" + doubleToString(m_osx, 2) + ",";
    spec += "y=" + doubleToString(m_osy, 2) + ",";
    spec += "ang=" + doubleToString(heading, 2) + ",";
    spec += "mag=20,"; // Length of the arrow in meters
    
    // Use m_us_name (inherited from IvPBehavior) to ensure each boat gets its own unique arrow label
    spec += "label=escape_heading_" + m_us_name + ",";
    spec += "color=green,";
    spec += "edge_size=2";

    // Post it to the MOOSDB
    postMessage("VIEW_VECTOR", spec);
}