#include <cmath>
#include <vector>
#include <string>
#include "BHV_SoftBoundary.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "XYPolygon.h"
#include "ZAIC_PEAK.h"

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

    // Parameters for IvP-function
    IvPFunction *new_function = new ZAIC_PEAK(
        "soft_boundary",  // Name
        0.0,              // Min utility (at border)
        100.0,            // Max utility (far from border)
        0.0,              // Current distance
        m_max_range,      // Peak-distance
        m_peak_width      // Width of the function peak
    );
    new_function->setPWT(100.0); // Standard-PWT
    setFunction(new_function);
}

//------------------------------------------------------------------
// Procedure: setParam - handle behavior configuration parameters

bool BHV_SoftBoundary::setParam(string param, string value) {
    // Convert the parameter to lower case for more general matching
    param = tolower(param);

    if (IvPBehavior::setParam(param, value)) return true;

    if (param == "polygon") {
        // Polygon from string (Format: "x1,y1:x2,y2:x3,y3:...")
        vector<string> points = parseString(value, ':');
        for (const auto& point : points) {
            vector<string> coords = parseString(point, ',');
            if (coords.size() == 2) {
                double x = atof(coords[0].c_str());
                double y = atof(coords[1].c_str());
                m_boundary_polygon.emplace_back(x, y);
            }
        }
        return true;
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
        return true;
    }
    return false;
}

//------------------------------------------------------------------
// Procedure: onRunState - called every helm itertion

IvPFunction* BHV_SoftBoundary::onRunState() {
    // Update Position, Polygon, etc.
    updateInfoIn();

    double dist_to_boundary = computeDistanceToBoundary();

    // Create/update IvP-Function for Repulsion
    ZAIC_PEAK *zaic = dynamic_cast<ZAIC_PEAK*>(getFunction());
    if (!zaic) {
        zaic = new ZAIC_PEAK(
            "soft_boundary",
            0.0, 100.0, dist_to_boundary, m_max_range, m_peak_width
        );
        setFunction(zaic);
    } else {
        zaic->setParam("x", dist_to_boundary);
    }

    // Visualization of border
    postViewPoint();

    return getFunction();
}

//------------------------------------------------------------------
// updateInfoIn: Update internal data

bool BHV_SoftBoundary::updateInfoIn() {
    bool ok = true;

    // Vehicle's position
    ok = ok && updateInfoIn("NAV_X", m_osx);
    ok = ok && updateInfoIn("NAV_Y", m_osy);

    // Polygon from MOOSDB
    string polygon_str;
    if (m_MissionReader->GetValue(m_boundary_var, polygon_str)) {
        m_boundary_polygon.clear();
        vector<string> points = parseString(polygon_str, ':');
        for (const auto& point : points) {
            vector<string> coords = parseString(point, ',');
            if (coords.size() == 2) {
                double x = atof(coords[0].c_str());
                double y = atof(coords[1].c_str());
                m_boundary_polygon.emplace_back(x, y);
            }
        }
    }

    if (!ok) {
        postWMessage("No ownship NAV_X/NAV_Y info in info_buffer.");
        return(false); // No data, no steering
    } else if (polygon_str.empty()) {
        postWMessage("No ownship polygon info in info_buffer.");
    }

    return ok;
}

//------------------------------------------------------------------
// Procedure: computeDistanceToBoundary - Calculate distance to next polygon edge

double BHV_SoftBoundary::computeDistanceToBoundary() {
    if (m_boundary_polygon.empty())
        return m_max_range + 1; // NO border -> no repulsion

    // Get polygon from points
    XYPolygon polygon;
    for (const auto& point : m_boundary_polygon) {
        polygon.add_vertex(point.first, point.second);
    }

    // Calculate distance to next edge
    double dist = polygon.dist_to_poly(m_osx, m_osy);

    // Restrict distance to [m_min_range, m_max_range]
    if (dist < m_min_range)
        return m_min_range;
    else if (dist > m_max_range)
        return m_max_range;
    else
        return dist;
}

//------------------------------------------------------------------
// Procedure: postViewPoint - Visualize edge in pMarineViewer (optionally)

void BHV_SoftBoundary::postViewPoint() {
    if (!m_boundary_polygon.empty()) {
        string spec = "label=soft_boundary,";
        spec += "type=string,";
        spec += "color=red,";
        spec += "msg=BOUNDARY_POLYGON,";
        spec += "x=" + doubleToString(m_osx, 2) + ",";
        spec += "y=" + doubleToString(m_osy, 2);

        // Add polygon points
        for (const auto& point : m_boundary_polygon) {
            spec += ":" + doubleToString(point.first, 2) + "," + doubleToString(point.second, 2);
        }

        postViewPoint(spec);
    }
}