#ifndef FLOCK_EVALUATOR_H
#define FLOCK_EVALUATOR_H

#include "MOOS/libMOOS/App/MOOSApp.h"
#include <map>
#include <string>
#include <fstream>
#include <vector>
#include <utility>

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

protected: 
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();
    void RegisterVariables();

protected: 
    double calculateConvexArea();
    double crossProduct(Point p, Point q, Point r);

private:
    std::map<std::string, VehicleState> m_vehicles;
    
    // Evaluation Parameters
    double m_overlap_distance; 
    double m_cluster_distance; 
    
    // Running sums for the new metrics
    double m_sum_polarization;
    double m_sum_mean_distance;
    double m_sum_max_cluster_size;
    double m_sum_rca;
    long m_overlap_ticks; // Counts iterations where an overlap occurs
    
    long m_iterations_count;

    std::ofstream m_log_file;
    std::string m_run_id;
};

#endif