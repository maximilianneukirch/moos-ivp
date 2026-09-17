#include <cmath>
#include <vector>
#include <string>
#include "BHV_SoftBoundary.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "ZAIC_PEAK.h"
#include "XYFormatUtilsPoly.h"


using namespace std;

//------------------------------------------------------------------
// Procedure: Constructor

BHV_SoftBoundary::BHV_SoftBoundary(IvPDomain domain)
    : IvPBehavior(domain) {

    // All distances are in meters, all speed in meters per second
    // Default values for configuration parameters

    m_max_range   = 20.0;
    m_min_range   = 5.0;
    m_max_delta   = 91.0;   // Max corrective delta: 91 deg off the wall -> glides neatly alongside the boundary
    m_curve_power = 2.0;    // quadratic ramp: gentle until the boat is close, then turns in smoothly
    m_peak_width  = 15.0;
    m_base_width  = 170.0;
    m_summit_delta = 50.0;
    m_boundary_var = "BOUNDARY_POLYGON";
    m_min_speed = 1.0;

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
            m_boundary_polygon = parsed_poly;
            return true;
        }
        postWMessage("setParam failed to parse static polygon: " + value);
        return false;
    }
    else if (param == "max_range") {
        m_max_range = atof(value.c_str());
        return true;
    }
    else if (param == "min_range") {
        m_min_range = atof(value.c_str());
        return true;
    }
    else if (param == "max_delta") {
        m_max_delta = atof(value.c_str());
        return true;
    }
    else if (param == "curve_power") {
        m_curve_power = atof(value.c_str());
        return true;
    }
    else if (param == "peak_width") {
        m_peak_width = atof(value.c_str());
        return true;
    }
    else if (param == "base_width") {
        m_base_width = atof(value.c_str());
        return true;
    }
    else if (param == "summit_delta") {
        m_summit_delta = atof(value.c_str());
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

//------------------------------------------------------------------
// Procedure: onRunState - called every helm itertion

IvPFunction* BHV_SoftBoundary::onRunState() {
    // Update Position, Polygon, etc.
    if (!updateInfoIn()) {
        return nullptr;
    }

    if (!m_boundary_polygon.is_convex()) {
        m_boundary_polygon.determine_convexity();
    }

    // Distance from the boat's *real* position to the nearest boundary point.
    // (No lookahead projection: the gradually-increasing delta already provides
    //  the smooth response, and the real position keeps activation and distance
    //  consistent with where the boat actually is.)
    double closest_x, closest_y;
    m_boundary_polygon.closest_point_on_poly(m_osx, m_osy, closest_x, closest_y);
    double dist_to_boundary = hypot(m_osx - closest_x, m_osy - closest_y);

    // Visualization of border
    postViewPolygon();

    bool is_inside = m_boundary_polygon.contains(m_osx, m_osy);

    // Heading (0-360 deg) from the boat toward the nearest boundary point, i.e.
    // the direction of the wall. The safe interior lies on the other side.
    double to_wall = relAng(m_osx, m_osy, closest_x, closest_y);

    // How far is the current heading from pointing straight at the wall?
    //   0 deg  -> heading straight into the wall
    //  90 deg  -> heading tangent to the wall (gliding alongside)
    // 180 deg  -> heading straight away from the wall
    double ang_from_wall = fabs(angle180(m_osh - to_wall));

    // --- Decide whether the behavior is active ---
    // Deactivate whenever the boat is pointing in a direction that will carry it
    // out of the danger zone (tangent or away from the boundary) - even if it is
    // close to the edge. The single exception is a boat that has crossed the
    // boundary (outside the polygon): that case must always steer the boat back in.
    if (is_inside) {
        if (dist_to_boundary >= m_max_range) {
            return nullptr;  // out of the danger zone (far from the boundary)
        }
        if (ang_from_wall >= 90.0) {
            return nullptr;  // heading out of the danger zone (tangent or away)
        }
    }

    // --- Compute the corrective heading delta ---
    // The behavior reads the boat's current heading and applies a delta toward a
    // safe direction, instead of replacing it with an absolute escape heading.
    // The delta grows as the boat closes in on the boundary and is capped so the
    // commanded course never points more than m_max_delta (91 deg) from the wall -
    // letting the flock glide neatly alongside the boundary.
    double delta;
    if (is_inside) {
        // Aim for the tangent direction (m_max_delta off the wall) on the side
        // nearest the current heading, so we always turn the short way.
        double opt1 = angle360(to_wall + m_max_delta);
        double opt2 = angle360(to_wall - m_max_delta);
        double target = (fabs(angle180(m_osh - opt1)) <= fabs(angle180(m_osh - opt2))) ? opt1 : opt2;

        // Proximity factor in [0,1]: 0 at max_range, 1 at min_range. Shaped by
        // curve_power so the correction stays gentle until the boat is close.
        double p = (m_max_range - dist_to_boundary) / (m_max_range - m_min_range);
        if (p < 0.0) p = 0.0;
        if (p > 1.0) p = 1.0;
        double proximity = pow(p, m_curve_power);

        delta = angle180(target - m_osh) * proximity;
    } else {
        // Boat has crossed the boundary: steer it straight back into the interior.
        delta = angle180(to_wall - m_osh);
    }

    // Cap the corrective delta at the configured maximum.
    if (delta >  m_max_delta) delta =  m_max_delta;
    if (delta < -m_max_delta) delta = -m_max_delta;

    double desired_course = angle360(m_osh + delta);

    // Build a course-preference peak at the desired course. The weight is the
    // behavior's constant (maximum) priority weight - it is the DELTA, not the
    // weight, that encodes how close the boat is to the boundary.
    ZAIC_PEAK zaic(m_domain, "course");
    zaic.setSummit(desired_course);
    zaic.setBaseWidth(m_base_width);
    zaic.setPeakWidth(m_peak_width);
    zaic.setSummitDelta(m_summit_delta);
    zaic.setValueWrap(true);

    IvPFunction *crs_ipf = zaic.extractIvPFunction();
    if(!crs_ipf) {
        postWMessage("Failure building the boundary course ZAIC_PEAK");
        return nullptr;
    }

    string zaic_warnings = zaic.getWarnings();
    if(zaic_warnings != "")
        postWMessage(zaic_warnings);

    // Constant, maximum weight: this behavior dominates whenever it is active.
    crs_ipf->setPWT(getPriorityWt());
    return crs_ipf;
}

//------------------------------------------------------------------
// updateInfoIn: Update internal data

bool BHV_SoftBoundary::updateInfoIn() {
    bool ok_x, ok_y, ok_h;
    string polygon_str;

    // Vehicle's position from InfoBuffer
    m_osx = getBufferDoubleVal("NAV_X", ok_x);
    m_osy = getBufferDoubleVal("NAV_Y", ok_y);
    m_osh = getBufferDoubleVal("NAV_HEADING", ok_h);

    // Allow dynamic polygon updates whenever a fresh value is posted.
    if (getBufferVarUpdated(m_boundary_var)) {
        bool ok_poly = false;
        string polygon_str = getBufferStringVal(m_boundary_var, ok_poly);
        if (ok_poly && !polygon_str.empty()) {
            XYPolygon parsed_poly = string2Poly(polygon_str);
            if (parsed_poly.size() > 0) {
                m_boundary_polygon = parsed_poly;
                if (!m_boundary_polygon.is_convex())
                    m_boundary_polygon.determine_convexity();
            } else {
                postWMessage("Failed to parse polygon from string: " + polygon_str);
            }
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
