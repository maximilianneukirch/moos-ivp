#ifndef FLOCK_EVALUATOR_H
#define FLOCK_EVALUATOR_H

#include "MOOS/libMOOS/App/MOOSApp.h"
#include <map>
#include <set>
#include <string>
#include <fstream>
#include <vector>
#include <utility>

// Hilfsstrukturen für Geometrie und Fahrzeugstatus
struct VehicleState {
    double x;
    double y;
    double heading;
};

struct Point {
    double x;
    double y;
};

class FlockEvaluator : public CMOOSApp {
public:
    FlockEvaluator();
    ~FlockEvaluator();

protected: // Standard MOOSApp Funktionen
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();
    void RegisterVariables();

protected: // Eigene Berechnungsfunktionen
    void updateCrashes();
    double calculateHeadingVariance();
    double calculateConvexArea();
    
    // Hilfsfunktion für den Jarvis March (Convex Hull)
    double crossProduct(Point p, Point q, Point r);

private:
    // Datenspeicher
    std::map<std::string, VehicleState> m_vehicles;
    
    // Set zur Speicherung aktuell kollidierender Paare (verhindert Mehrfachzählung)
    std::set<std::pair<std::string, std::string>> m_active_crashes;
    
    // Metriken
    int m_total_crashes;
    double m_crash_distance;      // Schwellenwert für Crash (z.B. 1.1m)
    double m_hysteresis_distance; // Schwellenwert zum Auflösen des Crashes (z.B. 1.3m)
    
    // Laufende Durchschnittsberechnung
    double m_sum_heading_variance;
    double m_sum_convex_area;
    long m_iterations_count;

    // Logging
    std::ofstream m_log_file;
    std::string m_run_id; // Kann optional per Parameter gesetzt werden
};

#endif