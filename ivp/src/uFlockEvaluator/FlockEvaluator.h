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
    std::vector<Point> computeHull(const std::vector<Point>& pts) const;
    double hullArea(const std::vector<Point>& hull) const;
    double hullMaxNonAdjacentDiagonal(const std::vector<Point>& hull) const;
    double crossProduct(Point p, Point q, Point r) const;
    double wrappedDelta(double d) const;
    double pairDistance(const VehicleState& a, const VehicleState& b) const;
    std::vector<Point> unwrapPositions() const;
    int computeLargestCluster(const std::vector<VehicleState>& states,
                               const std::vector<std::vector<double> >& pos_dist) const;

private:
    std::map<std::string, VehicleState> m_vehicles;

    // Gates all metric accumulation/logging in Iterate() until the fleet has
    // actually been deployed -- DEPLOY_ALL is posted directly to shoreside's
    // own MOOSDB (by pMarineViewer's DEPLOY button in visflocking_test, or
    // by uTimerScript's scripted event in visflocking_optimize), so
    // subscribing to it here needs no extra routing. Latches true forever
    // once seen (this app is restarted fresh for every mission/sweep run),
    // so a stray RETURN afterwards doesn't blank out already-collected data.
    bool m_deployed;

    // Overlap threshold: defaults to AGENT_DIAMETER (i.e. two agents'
    // circular bodies actually touching, matching ABM's
    // calculate_collision_time's 2*RADIUS_AGENT criterion). An explicit
    // OVERLAP_DISTANCE mission-file param still overrides this, for
    // back-compat with missions that tuned it directly.
    double m_overlap_distance;
    bool m_overlap_distance_explicit;
    double m_agent_diameter;

    // Cluster detection: Ward-linkage hierarchical clustering on a composite
    // distance combining normalized inter-agent distance and heading
    // (polarization) dissimilarity, cut at m_cluster_threshold -- matching
    // ABM's calculate_clustering()/return_clustering_distnace(). See
    // computeLargestCluster()/FlockEvaluator.cpp for the formula.
    double m_cluster_threshold;

    // Torus support: when TOROIDAL=true, every pairwise-distance-based
    // metric (MeanDistance, MaxClusterSize, OverlapRatio, the convex hull
    // used for AreaToCircleRatio) uses minimum-image distance instead of
    // plain Euclidean distance, and the convex hull for RCA is computed on
    // positions unwrapped into one contiguous patch first. ARENA_WIDTH must
    // match the *full* period of the wraparound (e.g. the visflocking_optimize
    // torus mission wraps a 300m square via uSimMarineV23 wormholes at
    // x,y=+/-150 -- see meta_vehicle.moos -- so ARENA_WIDTH=300 there, not
    // 150). ARENA_WIDTH/ARENA_HEIGHT (the latter defaults to ARENA_WIDTH if
    // unset, i.e. assume a square arena) also set the arena-diagonal
    // normalization used by the cluster distance metric above, independent
    // of whether TOROIDAL is enabled.
    bool m_toroidal;
    double m_arena_width;
    double m_arena_height;

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