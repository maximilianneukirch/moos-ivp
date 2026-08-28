#include <cmath>
#include <vector>
#include <string>
#include "BHV_SoftBoundary.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "ZAIC_PEAK.h"
#include "ZAIC_Vector.h"
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
    m_peak_width  = 20.0;
    m_curve_power = 2.0;   // quadratic by default: flat top -> helm blends smoothly instead of snapping
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
    else if (param == "peak_width") {
        m_peak_width = atof(value.c_str());
        return true;
    }
    else if (param == "curve_power") {
        m_curve_power = atof(value.c_str());
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

    if (!m_boundary_polygon.is_convex()) {
        m_boundary_polygon.determine_convexity();
    }

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

    // Visualization of border
    postViewPolygon();

    // Return nullptr if inside polygon and outside force-field max_range (far from border)
    bool is_inside = m_boundary_polygon.contains(m_osx, m_osy);
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


    // Compute dynamic weight (priority) of behavior
    // 0 at max_range, linear up to configured pwt at min_range
    double weight = 0.0;
    double base_pwt = getPriorityWt();
    double escape_heading = 0.0;

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

    // COURSE: Build a GENTLE, flat-topped course-preference curve (quadratic by
    // default) that peaks (utility 100) at escape_heading and falls to 0 at
    // +/-180 deg, using a power curve instead of a linear ramp:
    //
    //     utility(d) = 100 * (1 - (d / 180)^power)      d = |heading - escape|
    //
    // A power curve has ZERO slope at the peak (flat top), unlike the old linear
    // ramp. Because the helm picks the single course that maximizes the SUM of all
    // behaviors' weighted utilities, a flat-topped boundary curve lets the other
    // behaviors (e.g. VisFlocking) keep influencing that optimum, so the resulting
    // course is a SMOOTH compromise that drifts toward escape_heading only as this
    // behavior's weight grows - instead of snapping to it (the hard turns / S-curves).
    //
    // curve_power: 1 = linear (old, snappy), 2 = quadratic (soft), higher = softer.
    double power = m_curve_power;
    if (power < 0.1) power = 0.1;
    double max_dist = 180.0;

    int crs_ix  = m_domain.getIndex("course");
    int crs_pts = m_domain.getVarPoints("course");
    std::vector<double> domain_vec(crs_pts, 0.0);
    std::vector<double> utility_vec(crs_pts, 0.0);
    for(int i = 0; i < crs_pts; i++) {
        double h    = m_domain.getVal(crs_ix, i);
        double diff = fabs(angle180(h - escape_heading));
        domain_vec[i] = h;
        if(diff <= max_dist) {
            double x = diff / max_dist;          // 0 at escape_heading, 1 at the antipode
            utility_vec[i] = 100.0 * (1.0 - pow(x, power));
        } else {
            utility_vec[i] = 0.0;
        }
    }

    ZAIC_Vector crs_zaic(m_domain, "course");
    crs_zaic.setDomainVals(domain_vec);
    crs_zaic.setRangeVals(utility_vec);

    IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();
    if(!crs_ipf) {
        postWMessage("Failure building the course ZAIC_Vector");
        return nullptr;
    }
    // Apply dynamic weight
    crs_ipf->setPWT(weight);
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
