#include "FlockEvaluator.h"
#include "MBUtils.h"
#include "NodeRecordUtils.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

using namespace std;

FlockEvaluator::FlockEvaluator() {
    // Physical parameters
    m_agent_diameter = 1.1;             // agent hull diameter, meters
    m_overlap_distance = m_agent_diameter; // default: 2*radius, overridable via OVERLAP_DISTANCE
    m_overlap_distance_explicit = false;

    m_cluster_threshold = 0.275; // Ward dendrogram cut, matches ABM's tuned default

    m_toroidal = false;    // must be explicitly enabled to match a wormhole/torus mission
    m_arena_width = 300.0; // full wrap period, e.g. visflocking_optimize's 300m torus
    m_arena_height = m_arena_width; // assume square arena unless ARENA_HEIGHT overrides

    // Initialize running sums
    m_sum_polarization = 0.0;
    m_sum_mean_distance = 0.0;
    m_sum_max_cluster_size = 0.0;
    m_sum_rca = 0.0;
    m_overlap_ticks = 0;
    
    m_iterations_count = 0;
    m_run_id = "run_default";

    m_deployed = false;
    m_deploy_time = 0.0;
    m_warmup_secs = 0.0;
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
        // The first five metric columns are running averages over the
        // post-warmup run; the *_inst columns are this tick's instantaneous
        // values, so any averaging window can be re-derived afterwards (and
        // convergence can be seen at all). Appended at the end so existing
        // readers that select by column name are unaffected.
        m_log_file << "RunID,PolarizationOrder,MeanDistance,MaxClusterSize,AreaToCircleRatio,OverlapRatio,Iterations,"
                   << "P_inst,Dist_inst,Cluster_inst,RCA_inst\n";
        m_log_file.flush(); 
    }

    m_MissionReader.GetConfigurationParam("RUN_ID", m_run_id);
    m_MissionReader.GetConfigurationParam("TOROIDAL", m_toroidal);
    m_MissionReader.GetConfigurationParam("ARENA_WIDTH", m_arena_width);
    m_arena_height = m_arena_width;
    m_MissionReader.GetConfigurationParam("ARENA_HEIGHT", m_arena_height);
    m_MissionReader.GetConfigurationParam("AGENT_DIAMETER", m_agent_diameter);
    m_MissionReader.GetConfigurationParam("CLUSTER_THRESHOLD", m_cluster_threshold);
    m_MissionReader.GetConfigurationParam("WARMUP_SECONDS", m_warmup_secs);

    // OVERLAP_DISTANCE, if explicitly set in the mission file, still wins
    // over the AGENT_DIAMETER-derived default (2*radius).
    m_overlap_distance_explicit =
        m_MissionReader.GetConfigurationParam("OVERLAP_DISTANCE", m_overlap_distance);
    if(!m_overlap_distance_explicit) {
        m_overlap_distance = m_agent_diameter;
    }

    RegisterVariables();
    return true;
}

bool FlockEvaluator::OnConnectToServer() {
    RegisterVariables();
    return true;
}

void FlockEvaluator::RegisterVariables() {
    Register("NODE_REPORT", 0);
    // Posted directly to shoreside's own MOOSDB -- by pMarineViewer's DEPLOY
    // button in visflocking_test, or by uTimerScript's scripted event in
    // visflocking_optimize -- so no extra pShare routing is needed to see it
    // here. Gates Iterate() below so metrics aren't collected/averaged while
    // the fleet is still sitting stationary pre-deploy.
    Register("DEPLOY_ALL", 0);
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
        else if(msg.GetKey() == "DEPLOY_ALL") {
            // Posted as a string ("true"/"false") by both pMarineViewer's
            // button handler and uTimerScript (neither treats "true" as a
            // number) -- but also accept a nonzero double, in case some
            // other trigger ever posts it numerically instead.
            bool val = false;
            if(msg.IsString()) val = (tolower(msg.GetString()) == "true");
            else if(msg.IsDouble()) val = (msg.GetDouble() != 0.0);

            if(val && !m_deployed) {
                m_deployed = true; // latches -- a later RETURN shouldn't blank collected data
                m_deploy_time = MOOSTime();
            }
        }
    }
    return true;
}

bool FlockEvaluator::Iterate() {
    if(!m_deployed) return true; // fleet hasn't been deployed yet -- ignore stationary pre-deploy ticks

    // Discard the post-deploy transient (MOOS time is warped consistently, so
    // this is sim seconds).
    if((MOOSTime() - m_deploy_time) < m_warmup_secs) return true;

    int n = m_vehicles.size();
    if(n < 3) return true; // Wait for vehicles to deploy

    m_iterations_count++;

    vector<VehicleState> states;
    for(const auto& pair : m_vehicles) states.push_back(pair.second);

    // --- 1. Pairwise distances, Overlap ratio, Mean distance ---
    // Overlap threshold (m_overlap_distance) defaults to AGENT_DIAMETER,
    // i.e. two agents' circular hulls actually touching -- matches ABM's
    // calculate_collision_time() (dist < 2*RADIUS_AGENT).
    vector<vector<double> > pos_dist(n, vector<double>(n, 0.0));
    double sum_dist = 0.0;
    bool is_overlapping_now = false;
    int pair_count = 0;

    for(int i = 0; i < n; i++) {
        for(int j = i + 1; j < n; j++) {
            double dist = pairDistance(states[i], states[j]);
            pos_dist[i][j] = pos_dist[j][i] = dist;

            sum_dist += dist;
            if(dist < m_overlap_distance) is_overlapping_now = true;

            pair_count++;
        }
    }

    double current_mean_dist = sum_dist / std::max(1, pair_count);
    if(is_overlapping_now) m_overlap_ticks++;

    // --- 2. Largest cluster (Ward hierarchical clustering) ---
    int current_max_cluster = computeLargestCluster(states, pos_dist);


    // --- 3. Polarization Order (P) ---
    double sum_sin = 0.0, sum_cos = 0.0;
    for(int i = 0; i < n; i++) {
        double rad = states[i].heading * M_PI / 180.0;
        sum_sin += sin(rad);
        sum_cos += cos(rad);
    }
    double current_polarization = sqrt(sum_sin * sum_sin + sum_cos * sum_cos) / n;


    // --- 4. Area-to-Circle Ratio (RCA) ---
    // On a torus, positions must be unwrapped into one contiguous patch
    // before hulling -- Jarvis March on raw wrapped (x,y) would otherwise
    // treat a group straddling a wormhole seam as spanning the whole arena.
    vector<Point> hull_pts;
    if(m_toroidal) {
        hull_pts = unwrapPositions();
    } else {
        for(const auto& s : states) hull_pts.push_back({s.x, s.y});
    }
    vector<Point> hull = computeHull(hull_pts);
    double convex_area = hullArea(hull);
    // Reference circle diameter = longest non-adjacent hull-vertex diagonal
    // (matches ABM's plot_convex_hull_in_current_t calc_longest_d), not the
    // max pairwise distance across all agents.
    double hull_diam = hullMaxNonAdjacentDiagonal(hull);
    double circle_area = std::max(0.0001, M_PI * std::pow(hull_diam / 2.0, 2));
    double current_rca = convex_area / circle_area;


    // --- 5. Accumulate and Calculate Averages ---
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
               << m_iterations_count << ","
               << current_polarization << ","
               << current_mean_dist << ","
               << current_max_cluster << ","
               << current_rca << "\n";
    m_log_file.flush();

    return true;
}

// ----------------------------------------------------------------
// Geometric Helper Functions
// ----------------------------------------------------------------

double FlockEvaluator::crossProduct(Point p, Point q, Point r) const {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
}

// --- Torus helpers ---

// Signed shortest displacement for a 1-D periodic coordinate of period
// m_arena_width (minimum-image convention): brings d into (-width/2, width/2].
double FlockEvaluator::wrappedDelta(double d) const {
    if(!m_toroidal || m_arena_width <= 0.0) return d;
    double w = m_arena_width;
    d = fmod(d, w);
    if(d > w / 2.0) d -= w;
    if(d < -w / 2.0) d += w;
    return d;
}

double FlockEvaluator::pairDistance(const VehicleState& a, const VehicleState& b) const {
    double dx = wrappedDelta(a.x - b.x);
    double dy = wrappedDelta(a.y - b.y);
    return sqrt(dx*dx + dy*dy);
}

// Unwraps every vehicle's position relative to the first vehicle (arbitrary
// reference) by the minimum-image displacement, producing one contiguous
// patch of positions suitable for a plain (non-periodic) convex hull -- the
// same trick data_loader.py's on_torus branch relies on.
vector<Point> FlockEvaluator::unwrapPositions() const {
    vector<Point> pts;
    if(m_vehicles.empty()) return pts;
    const VehicleState& ref = m_vehicles.begin()->second;
    for(auto const& pair : m_vehicles) {
        const VehicleState& v = pair.second;
        pts.push_back({ref.x + wrappedDelta(v.x - ref.x),
                        ref.y + wrappedDelta(v.y - ref.y)});
    }
    return pts;
}

vector<Point> FlockEvaluator::computeHull(const vector<Point>& pts) const {
    int n = pts.size();
    vector<Point> hull;
    if(n < 3) return hull;

    // Jarvis March (Gift Wrapping)
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

    return hull;
}

double FlockEvaluator::hullArea(const vector<Point>& hull) const {
    if(hull.size() < 3) return 0.0;

    // Shoelace Formula
    double area = 0.0;
    int j = hull.size() - 1;
    for(size_t i = 0; i < hull.size(); i++) {
        area += (hull[j].x + hull[i].x) * (hull[j].y - hull[i].y);
        j = i;
    }

    return abs(area / 2.0);
}

// Longest distance between two hull vertices that are not consecutive in
// hull-boundary order, matching ABM's plot_convex_hull_in_current_t
// (calc_longest_d): only (i, i+1) pairs for i in [0, size-2] are excluded as
// "adjacent" -- the wrap-around pair (last, first) is deliberately NOT
// excluded there, and this mirrors that exactly for parity with ABM's
// reported values.
double FlockEvaluator::hullMaxNonAdjacentDiagonal(const vector<Point>& hull) const {
    int m = hull.size();
    double best = 0.0;
    for(int i = 0; i < m; i++) {
        for(int j = i + 1; j < m; j++) {
            if(j == i + 1) continue;
            double dx = hull[i].x - hull[j].x;
            double dy = hull[i].y - hull[j].y;
            double d = sqrt(dx*dx + dy*dy);
            if(d > best) best = d;
        }
    }
    return best;
}

// Ward-linkage agglomerative clustering on a composite distance combining
// normalized inter-agent distance and heading (polarization) dissimilarity,
// cut at m_cluster_threshold. Mirrors ABM's return_clustering_distnace() +
// calculate_clustering()/calculate_largest_subcluster_size(): merging the
// dendrogram at height m_cluster_threshold is equivalent to ABM's
// dendrogram-color cut (unmerged leaves become singleton clusters, matching
// that code's per-leaf remap of "color C0" leaves to distinct new ids).
int FlockEvaluator::computeLargestCluster(const vector<VehicleState>& states,
                                           const vector<vector<double> >& pos_dist) const {
    int n = states.size();
    if(n == 0) return 0;
    if(n == 1) return 1;

    // Median of all pairwise position distances this tick.
    vector<double> upper;
    upper.reserve(n * (n - 1) / 2);
    for(int i = 0; i < n; i++)
        for(int j = i + 1; j < n; j++)
            upper.push_back(pos_dist[i][j]);

    sort(upper.begin(), upper.end());
    size_t m = upper.size();
    double median_iid = (m % 2 == 1) ? upper[m / 2]
                                      : (upper[m / 2 - 1] + upper[m / 2]) / 2.0;

    double max_dist_norm = sqrt(m_arena_width * m_arena_width +
                                 m_arena_height * m_arena_height) / 2.0;
    if(max_dist_norm <= 0.0) max_dist_norm = 1.0;

    // Composite distance matrix: (heading-dissimilarity + normalized
    // position-deviation) / 2, matching ABM's return_clustering_distnace().
    vector<vector<double> > D(n, vector<double>(n, 0.0));
    for(int i = 0; i < n; i++) {
        double ux_i = cos(states[i].heading * M_PI / 180.0);
        double uy_i = sin(states[i].heading * M_PI / 180.0);
        for(int j = i + 1; j < n; j++) {
            double ux_j = cos(states[j].heading * M_PI / 180.0);
            double uy_j = sin(states[j].heading * M_PI / 180.0);

            double ux_sum = ux_i + ux_j;
            double uy_sum = uy_i + uy_j;
            double pm = sqrt(ux_sum*ux_sum + uy_sum*uy_sum) / 2.0;
            double dist_pm = 1.0 - pm;

            double niidm = fabs(median_iid - pos_dist[i][j]) / max_dist_norm;

            double d = (dist_pm + niidm) / 2.0;
            D[i][j] = D[j][i] = d;
        }
    }

    // Ward agglomeration (Lance-Williams update), merging while the closest
    // remaining pair of clusters is below m_cluster_threshold.
    vector<int> csize(n, 1);
    vector<bool> active(n, true);

    while(true) {
        int bi = -1, bj = -1;
        double best = std::numeric_limits<double>::max();
        for(int i = 0; i < n; i++) {
            if(!active[i]) continue;
            for(int j = i + 1; j < n; j++) {
                if(!active[j]) continue;
                if(D[i][j] < best) { best = D[i][j]; bi = i; bj = j; }
            }
        }
        if(bi == -1 || best >= m_cluster_threshold) break;

        for(int k = 0; k < n; k++) {
            if(!active[k] || k == bi || k == bj) continue;
            double ni = csize[bi], nj = csize[bj], nk = csize[k];
            double dik = D[bi][k], djk = D[bj][k], dij = D[bi][bj];
            double d = sqrt(((ni+nk)*dik*dik + (nj+nk)*djk*djk - nk*dij*dij) /
                            (ni+nj+nk));
            D[bi][k] = D[k][bi] = d;
        }
        csize[bi] += csize[bj];
        active[bj] = false;
    }

    int max_cluster = 0;
    for(int i = 0; i < n; i++) {
        if(active[i] && csize[i] > max_cluster) max_cluster = csize[i];
    }
    return max_cluster;
}