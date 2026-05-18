#include "FlockEvaluator.h"
#include "MBUtils.h"
#include "NodeRecordUtils.h"
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

// Konstruktor - Initialisierung der Variablen
FlockEvaluator::FlockEvaluator() {
    m_total_crashes = 0;
    m_crash_distance = 0.9;       // Länge der alpha-Boote
    m_hysteresis_distance = 1.3;  // Leicht größer, um "Flackern" zu vermeiden
    
    m_sum_heading_variance = 0.0;
    m_sum_convex_area = 0.0;
    m_iterations_count = 0;
    
    m_run_id = "run_default";
}

FlockEvaluator::~FlockEvaluator() {
    if(m_log_file.is_open()) {
        m_log_file.close();
    }
}

bool FlockEvaluator::OnStartUp() {
    m_log_file.open("flock_evaluation.csv", ios::app);
    if(m_log_file.tellp() == 0) {
        m_log_file << "RunID,TotalCrashes,AvgHeadingVar,AvgConvexArea,Iterations\n";
        m_log_file.flush(); // <--- DIESE ZEILE HINZUFÜGEN! Zwingt C++ zum Speichern.
    }

    m_MissionReader.GetConfigurationParam("RUN_ID", m_run_id);
    RegisterVariables();
    return true;
}

bool FlockEvaluator::OnConnectToServer() {
    RegisterVariables();
    return true;
}

void FlockEvaluator::RegisterVariables() {
    Register("NODE_REPORT", 0);
}

bool FlockEvaluator::OnNewMail(MOOSMSG_LIST &NewMail) {
    // Öffne eine temporäre Debug-Datei
    std::ofstream dlog("debug_moos.txt", std::ios::app);
    
    MOOSMSG_LIST::iterator p;
    for(p = NewMail.begin(); p != NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        
        if(msg.GetKey() == "NODE_REPORT" && msg.IsString()) {
            dlog << "Empfangen: " << msg.GetString() << "\n";
            
            // WICHTIG: Das 'true' (für strict mode) wurde hier entfernt!
            NodeRecord record = string2NodeRecord(msg.GetString()); 
            
            dlog << "Ist Valid? " << record.valid() << " | Name: " << record.getName() << "\n";
            
            if(record.valid() && record.getName() != "") {
                std::string vname = record.getName();
                m_vehicles[vname].x = record.getX();
                m_vehicles[vname].y = record.getY();
                m_vehicles[vname].heading = record.getHeading();
            }
        }
    }
    return true;
}

bool FlockEvaluator::Iterate() {
    if(m_vehicles.size() < 3) {
        return true; // Wir warten, bis genug Fahrzeuge im System sind
    }

    m_iterations_count++;

    // 1. Crashes berechnen
    updateCrashes();

    // 2. Heading Variance berechnen
    double current_var = calculateHeadingVariance();
    m_sum_heading_variance += current_var;

    // 3. Konvexe Fläche berechnen
    double current_area = calculateConvexArea();
    m_sum_convex_area += current_area;

    // Laufende Durchschnitte berechnen
    double avg_var = m_sum_heading_variance / m_iterations_count;
    double avg_area = m_sum_convex_area / m_iterations_count;

    // 4. In Datei loggen (Python liest später die letzte Zeile)
    m_log_file << m_run_id << "," 
               << m_total_crashes << "," 
               << avg_var << "," 
               << avg_area << "," 
               << m_iterations_count << "\n";
    m_log_file.flush();

    return true;
}

// ================================================================
// ALGORITHMEN
// ================================================================

void FlockEvaluator::updateCrashes() {
    // Konvertiere Map in einen Vector für einfacheren Paar-Vergleich
    vector<pair<string, VehicleState>> v_list(m_vehicles.begin(), m_vehicles.end());

    for(size_t i = 0; i < v_list.size(); i++) {
        for(size_t j = i + 1; j < v_list.size(); j++) {
            string v1 = v_list[i].first;
            string v2 = v_list[j].first;
            
            // Paarschlüssel immer alphabetisch sortieren, um Vertauschungen zu vermeiden
            if(v1 > v2) swap(v1, v2);
            pair<string, string> crash_pair = make_pair(v1, v2);

            double dx = v_list[i].second.x - v_list[j].second.x;
            double dy = v_list[i].second.y - v_list[j].second.y;
            double dist = sqrt(dx*dx + dy*dy);

            bool is_active = (m_active_crashes.find(crash_pair) != m_active_crashes.end());

            if(!is_active && dist < m_crash_distance) {
                // Neuer Crash!
                m_active_crashes.insert(crash_pair);
                m_total_crashes++;
            } 
            else if(is_active && dist > m_hysteresis_distance) {
                // Fahrzeuge sind wieder weit genug voneinander entfernt
                m_active_crashes.erase(crash_pair);
            }
        }
    }
}

double FlockEvaluator::calculateHeadingVariance() {
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    int n = m_vehicles.size();

    for(auto const& pair : m_vehicles) {
        const VehicleState& state = pair.second;
        
        // Umwandlung in Radiant
        double rad = state.heading * M_PI / 180.0;
        sum_sin += sin(rad);
        sum_cos += cos(rad);
    }

    double R = sqrt(sum_sin * sum_sin + sum_cos * sum_cos) / n;
    return 1.0 - R; // 0 = Perfekt parallel, 1 = Totales Chaos
}

// Hilfsfunktion: 2D Kreuzprodukt
double FlockEvaluator::crossProduct(Point p, Point q, Point r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
}

double FlockEvaluator::calculateConvexArea() {
    vector<Point> pts;

    for(auto const& pair : m_vehicles) {
        const VehicleState& state = pair.second;
        pts.push_back({state.x, state.y});
    }

    int n = pts.size();
    if(n < 3) return 0.0; // Keine Fläche möglich

    // 1. Jarvis March (Gift Wrapping) für die Convex Hull
    vector<Point> hull;
    
    // Finde den am weitesten links liegenden Punkt
    int l = 0;
    for(int i = 1; i < n; i++) {
        if(pts[i].x < pts[l].x) l = i;
    }

    int p = l, q;
    do {
        hull.push_back(pts[p]);
        q = (p + 1) % n;
        for(int i = 0; i < n; i++) {
            // Wenn Punkt 'i' weiter links (counter-clockwise) von der Linie pq liegt
            if(crossProduct(pts[p], pts[i], pts[q]) > 0) {
                q = i;
            }
        }
        p = q;
    } while(p != l);

    // 2. Shoelace-Formel für die Fläche des Polygons
    double area = 0.0;
    int j = hull.size() - 1;
    for(size_t i = 0; i < hull.size(); i++) {
        area += (hull[j].x + hull[i].x) * (hull[j].y - hull[i].y);
        j = i;
    }

    return abs(area / 2.0);
}