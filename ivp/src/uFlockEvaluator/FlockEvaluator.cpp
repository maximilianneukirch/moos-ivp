#include "FlockEvaluator.h"
#include "MBUtils.h"
#include "NodeRecordUtils.h"
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

FlockEvaluator::FlockEvaluator() {
    // Physical parameters 
    m_overlap_distance = 0.9; // Distance to count as "Overlap"
    m_cluster_distance = 5.0; // Max distance between agents to be in the same cluster
    
    // Initialize running sums
    m_sum_polarization = 0.0;
    m_sum_mean_distance = 0.0;
    m_sum_max_cluster_size = 0.0;
    m_sum_rca = 0.0;
    m_overlap_ticks = 0;
    
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
        // Updated Header matching Python analysis script
        m_log_file << "RunID,PolarizationOrder,MeanDistance,MaxClusterSize,AreaToCircleRatio,OverlapRatio,Iterations\n";
        m_log_file.flush(); 
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
    MOOSMSG_LIST::iterator p;
    for(p = NewMail.begin(); p != NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        
        if(msg.GetKey() == "NODE_REPORT" && msg.IsString()) {
            NodeRecord record = string2NodeRecord(msg.GetString()); 
            
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
    int n = m_vehicles.size();
    if(n < 3) return true; // Wait for vehicles to deploy

    m_iterations_count++;
    
    vector<VehicleState> states;
    for(const auto& pair : m_vehicles) states.push_back(pair.second);

    // --- 1. Distances, Overlap (R^sim_o), and Clusters (N^max_clus) ---
    double sum_dist = 0.0;
    double max_dist = 0.0001; 
    bool is_overlapping_now = false;
    int pair_count = 0;

    // Union-Find structure for clustering
    vector<int> parent(n);
    for(int i=0; i<n; i++) parent[i] = i;
    
    auto find_root = [&](int i) {
        int root = i;
        while(parent[root] != root) root = parent[root];
        return root;
    };
    
    auto unite = [&](int i, int j) {
        int root_i = find_root(i);
        int root_j = find_root(j);
        if(root_i != root_j) parent[root_i] = root_j;
    };

    for(int i = 0; i < n; i++) {
        for(int j = i + 1; j < n; j++) {
            double dx = states[i].x - states[j].x;
            double dy = states[i].y - states[j].y;
            double dist = sqrt(dx*dx + dy*dy);
            
            sum_dist += dist;
            if(dist > max_dist) max_dist = dist;
            if(dist < m_overlap_distance) is_overlapping_now = true;
            if(dist < m_cluster_distance) unite(i, j);
            
            pair_count++;
        }
    }

    double current_mean_dist = sum_dist / std::max(1, pair_count);
    if(is_overlapping_now) m_overlap_ticks++;

    // Calculate max cluster size
    vector<int> cluster_sizes(n, 0);
    for(int i = 0; i < n; i++) {
        cluster_sizes[find_root(i)]++;
    }
    int current_max_cluster = *std::max_element(cluster_sizes.begin(), cluster_sizes.end());


    // --- 2. Polarization Order (P) ---
    double sum_sin = 0.0, sum_cos = 0.0;
    for(int i = 0; i < n; i++) {
        double rad = states[i].heading * M_PI / 180.0;
        sum_sin += sin(rad);
        sum_cos += cos(rad);
    }
    double current_polarization = sqrt(sum_sin * sum_sin + sum_cos * sum_cos) / n;


    // --- 3. Area-to-Circle Ratio (RCA) ---
    double convex_area = calculateConvexArea();
    double circle_area = std::max(0.0001, M_PI * std::pow(max_dist / 2.0, 2));
    double current_rca = convex_area / circle_area;


    // --- 4. Accumulate and Calculate Averages ---
    m_sum_polarization += current_polarization;
    m_sum_mean_distance += current_mean_dist;
    m_sum_max_cluster_size += current_max_cluster;
    m_sum_rca += current_rca;

    double avg_pol = m_sum_polarization / m_iterations_count;
    double avg_dist = m_sum_mean_distance / m_iterations_count;
    double avg_cluster = m_sum_max_cluster_size / m_iterations_count;
    double avg_rca = m_sum_rca / m_iterations_count;
    double overlap_ratio = (double)m_overlap_ticks / m_iterations_count;

    // Log the current overall averages
    m_log_file << m_run_id << "," 
               << avg_pol << "," 
               << avg_dist << "," 
               << avg_cluster << "," 
               << avg_rca << ","
               << overlap_ratio << ","
               << m_iterations_count << "\n";
    m_log_file.flush();

    return true;
}

// ----------------------------------------------------------------
// Geometric Helper Functions
// ----------------------------------------------------------------

double FlockEvaluator::crossProduct(Point p, Point q, Point r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
}

double FlockEvaluator::calculateConvexArea() {
    vector<Point> pts;
    for(auto const& pair : m_vehicles) {
        pts.push_back({pair.second.x, pair.second.y});
    }

    int n = pts.size();
    if(n < 3) return 0.0; 

    // Jarvis March (Gift Wrapping)
    vector<Point> hull;
    int l = 0;
    for(int i = 1; i < n; i++) {
        if(pts[i].x < pts[l].x) l = i;
    }

    int p = l, q;
    do {
        hull.push_back(pts[p]);
        q = (p + 1) % n;
        for(int i = 0; i < n; i++) {
            if(crossProduct(pts[p], pts[i], pts[q]) > 0) {
                q = i;
            }
        }
        p = q;
    } while(p != l);

    // Shoelace Formula
    double area = 0.0;
    int j = hull.size() - 1;
    for(size_t i = 0; i < hull.size(); i++) {
        area += (hull[j].x + hull[i].x) * (hull[j].y - hull[i].y);
        j = i;
    }

    return abs(area / 2.0);
}