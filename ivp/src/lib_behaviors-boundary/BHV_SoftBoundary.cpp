#include <cmath>
#include <vector>
#include <string>
#include "BHV_SoftBoundary.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "ZAIC_PEAK.h"
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
    m_lookahead_dist = 5.0;

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
            //m_boundary_polygon.clear();
            //for (unsigned int i = 0; i < parsed_poly.size(); i++) {
            //    m_boundary_polygon.emplace_back(parsed_poly.get_vx(i), parsed_poly.get_vy(i));
            //}
            m_boundary_polygon = parsed_poly;
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
    else if (param == "lookahead_dist") {
        m_lookahead_dist = atof(value.c_str());
        return true;
    }
    return false;
}

//------------------------------------------------------------------
// Procedure: onRunState - called every helm itertion

IvPFunction* BHV_SoftBoundary::onRunState() {
    // Update Position, Polygon, etc.
    if (!updateInfoIn()) {
        return nullptr;
    }

//    if (m_boundary_polygon.empty()) {
//        return nullptr;
//    }
//
//    // Get polygon from points
//    XYPolygon polygon;
//    for (const auto& point : m_boundary_polygon) {
//        polygon.add_vertex(point.first, point.second);
//    }
//
//    // Visualization of border
//    postViewPolygon();
//
//    // Calculate distance and closest point on boundary
//    double closest_x, closest_y;
//    polygon.closest_point_on_poly(m_osx, m_osy, closest_x, closest_y);
//    double dist_to_boundary = hypot(m_osx - closest_x, m_osy - closest_y);
//
//    bool is_inside = polygon.contains(m_osx, m_osy);

    if (!m_boundary_polygon.is_convex()) {
        m_boundary_polygon.determine_convexity();
    }

    //double closest_x, closest_y;
    //m_boundary_polygon.closest_point_on_poly(m_osx, m_osy, closest_x, closest_y);
    //double dist_to_boundary = hypot(m_osx - closest_x, m_osy - closest_y);

    double math_heading_rad = (90.0 - m_osh) * (M_PI / 180.0);

    double proj_x = m_osx + (m_lookahead_dist * cos(math_heading_rad));
    double proj_y = m_osy + (m_lookahead_dist * sin(math_heading_rad));

    // Distance to boundary from projected pos
    double proj_closest_x, proj_closest_y;
    m_boundary_polygon.closest_point_on_poly(proj_x, proj_y, proj_closest_x, proj_closest_y);
    double proj_dist_to_boundary = hypot(proj_x - proj_closest_x, proj_y - proj_closest_y);

    // Distance to boundary from true pos
    double true_closest_x, true_closest_y;
    m_boundary_polygon.closest_point_on_poly(m_osx, m_osy, true_closest_x, true_closest_y);
    double true_dist_to_boundary = hypot(m_osx - true_closest_x, m_osy - true_closest_y);

    bool is_inside = m_boundary_polygon.contains(m_osx, m_osy);
    //bool is_inside;
    //bool os_inside = m_boundary_polygon.contains(m_osx, m_osy);
    //bool proj_os_inside = m_boundary_polygon.contains(proj_x, proj_y);

    //if (os_inside && proj_os_inside) {
    //    is_inside = true;
    //} else {
    //    is_inside = false;
    //}

    // Return nullptr if inside polygon and outside force-field max_range (far from border)
    if (is_inside) {
        double heading_to_boundary = relAng(m_osx, m_osy, true_closest_x, true_closest_y);

        double hdg_diff = abs(m_osh - heading_to_boundary);
        if (hdg_diff > 180.0) {
            hdg_diff = 360.0 - hdg_diff;
        }

        if (hdg_diff > 120.0) {
            return nullptr;
        }

        if (proj_dist_to_boundary >= m_max_range) {
            return nullptr;
        }
    }

    // Visualization of border
    postViewPolygon();

    // Compute dynamic weight (priority) of behavior
    // 0 at max_range, linear up to configured pwt at min_range
    double weight = 0.0;
    double base_pwt = getPriorityWt();
    double escape_heading = 0.0;
    
    // V1: lets the botas outside the polygon drive in a smooth curve like inside
    //double outward_norm = relAng(closest_x, closest_y, m_osx, m_osy);

    //if (!is_inside) {
    //    weight = base_pwt;
//
    //    double inward_norm = angle360(outward_norm + 180.0);
//
    //    double tangent_offset = 70.0;
    //    double opt1 = angle360(inward_norm + tangent_offset);
    //    double opt2 = angle360(inward_norm - tangent_offset);
//
    //    double diff1 = abs(m_osh - opt1); if (diff1 > 180.0) diff1 = 360 - diff1;
    //    double diff2 = abs(m_osh - opt2); if (diff2 > 180.0) diff2 = 360 - diff2;
//
    //    escape_heading = (diff1 <= diff2) ? opt1 : opt2;
//
    //} else {
    //    if (dist_to_boundary <= m_min_range) {
    //        weight = base_pwt;
    //    } else {
    //        // TODO: Check for m_max_range = m_min_range
    //        double fraction = (m_max_range - dist_to_boundary) / (m_max_range - m_min_range);
    //        weight = base_pwt * fraction;
    //    }
//
    //    double tangent_offset = 70.0;
    //    double opt1 = angle360(outward_norm + tangent_offset);
    //    double opt2 = angle360(outward_norm - tangent_offset);
//
    //    double diff1 = abs(m_osh - opt1); if (diff1 > 180.0) diff1 = 360 - diff1;
    //    double diff2 = abs(m_osh - opt2); if (diff2 > 180.0) diff2 = 360 - diff2;
//
    //    escape_heading = (diff1 <= diff2) ? opt1 : opt2;
    //}

    // V2: lets the boats outside the polygon 
    if (!is_inside || (is_inside && true_dist_to_boundary < m_min_range)) {
        // Boat already violated boundary and is outside -> highes pwt, inverted escape_heading
        weight = base_pwt;
        escape_heading = relAng(m_osx, m_osy, true_closest_x, true_closest_y);
    } else {
        if (proj_dist_to_boundary <= m_min_range) {
            weight = base_pwt;
        } else {
            // TODO: Check for m_max_range = m_min_range
            double fraction = (m_max_range - proj_dist_to_boundary) / (m_max_range - m_min_range);
            weight = base_pwt * fraction;
        }
        
        // Compute escape heading (absolute 360-deg heading from boundary point to vehocle pos)
        double inward_heading = relAng(proj_closest_x, proj_closest_y, m_osx, m_osy);

        double tangent_offset = 70.0;
        double opt1 = angle360(inward_heading + tangent_offset);
        double opt2 = angle360(inward_heading - tangent_offset);

        double diff1 = abs(m_osh - opt1);
        if (diff1 > 180.0) diff1 = 360.0 - diff1;

        double diff2 = abs(m_osh - opt2);
        if (diff2 > 180.0) diff2 = 360.0 - diff2;

        if (diff1 <= diff2) {
            escape_heading = opt1;
        } else {
            escape_heading = opt2;
        }

        //escape_heading = opt1; // fixed wall-behavior (evading to the left)
    }

    // COURSE: Build function with ZAIC
    ZAIC_PEAK crs_zaic(m_domain, "course");
    crs_zaic.setSummit(escape_heading);
    crs_zaic.setPeakWidth(m_peak_width);
    crs_zaic.setBaseWidth(360.0);           // 360 deg of degradation
    crs_zaic.setSummitDelta(0.0);
    crs_zaic.setValueWrap(true);            // ValueWrap: wrap 360 to 0

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

    if (m_boundary_polygon.size() == 0) {
        // Polygon from InfoBuffer
        polygon_str = getBufferStringVal(m_boundary_var, ok_poly);
    }

    if (ok_poly && !polygon_str.empty()) {
        XYPolygon parsed_poly = string2Poly(polygon_str);
        
        if (parsed_poly.size() > 0) {
            //m_boundary_polygon.clear();
            //for (unsigned int i = 0; i < parsed_poly.size(); i++) {
            //    m_boundary_polygon.emplace_back(parsed_poly.get_vx(i), parsed_poly.get_vy(i));
            //}
            m_boundary_polygon = parsed_poly;
            if (!m_boundary_polygon.is_convex()) {
                m_boundary_polygon.determine_convexity();
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
    } else if (m_boundary_polygon.size() == 0) {
        postWMessage("No boundary polygon configured in .bhv or received from DB.");
        return false;
    }

    return true;
}

//------------------------------------------------------------------
// Procedure: postViewPoint - Visualize edge in pMarineViewer (optionally)

void BHV_SoftBoundary::postViewPolygon() {
    if (m_boundary_polygon.size() > 0) {
        string spec = "pts={";

        for (unsigned int i = 0; i < m_boundary_polygon.size(); i++) {
            spec += doubleToString(m_boundary_polygon.get_vx(i), 2) + "," +
                    doubleToString(m_boundary_polygon.get_vy(i), 2);

            if (i < m_boundary_polygon.size() - 1) {
                spec += ":";
            }
        }

        spec += "}, label=soft_boundary,edge_color=red,edge_size=2,vertex_color=white,vertex_size=2";

        postMessage("VIEW_POLYGON", spec);
    }
}