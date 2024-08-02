/**
 * File:   visis_program.h
 *
 * Date:   11.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_VISIS_PROGRAM_H_
#define VISIS_VISIS_PROGRAM_H_

#include <string>
#include <limits>
#include <vector>
#include <random>

namespace visis {

/**
 * All program option variables and their default values should be defined here.
 * For each variable, there should be an option added in AddProgramOptions.
 */
struct ProgramOptions {

    bool irace = false;

    std::string map_full_path;
    std::string map_name = "undefined";
    std::string map_extension = ".txt";
    std::string map_dir = ".";
    std::string out_dir = ".";
    std::string out_file;
    double map_scale = std::numeric_limits<double>::quiet_NaN();

    double vis_radius = std::numeric_limits<double>::infinity();
    double robustness_radius = 0.0;
    double robustness_sample_dist = std::numeric_limits<double>::quiet_NaN();
    double sample_arc_edges_angle = M_PI / 36.0;
    double sample_arc_edges_dist = std::numeric_limits<double>::quiet_NaN();

    double rob_radius = 0.0;

    double min_coverage_ratio = 0.999;

    double target_bucket_ratio = 0.1;
    double target_bucket_size = std::numeric_limits<double>::quiet_NaN();

    double weight_bucket_ratio = 0.1;
    double weight_bucket_size = std::numeric_limits<double>::quiet_NaN();

    bool route_cyclic = true;
    std::vector<double> route_anchor_init;
    std::vector<double> route_anchor_exit;

    double vis_bucket_size = 1.0;
    bool vis_optimize_buckets = false;

    std::vector<std::string> generate_guards;
    /// === rand ===
    unsigned guards_random_n = 100;
    unsigned guards_random_seed = std::random_device{}();
    /// === reflex ===
    double guards_reflex_deg = 0.0;
    double guards_reflex_dist = 0.0;
    unsigned guards_reflex_n_max = std::numeric_limits<unsigned>::max();
    unsigned guards_reflex_rand_seed = std::random_device{}();
    /// === ccdt ===
    unsigned guards_ccdt_n_max = std::numeric_limits<unsigned>::max();
    unsigned guards_ccdt_rand_seed = std::random_device{}();
    /// === ka ===
    std::string guards_ka_order = "ears_small";
    std::string guards_ka_rule = "best";
    unsigned guards_ka_n_max = std::numeric_limits<unsigned>::max();
    unsigned guards_ka_rand_seed = std::random_device{}();
    /// === cov_rand ===
    double guards_cov_rand_ratio = 0.999;
    double guards_cov_rand_seed = std::random_device{}();
    unsigned guards_cov_rand_n_max = std::numeric_limits<unsigned>::max();
    /// === cov_inf ===
    double guards_cov_inf_ratio = 0.999;
    double guards_cov_inf_seed = std::random_device{}();
    unsigned guards_cov_inf_n_max = std::numeric_limits<unsigned>::max();
    /// === cov_dual ===
    double guards_cov_dual_ratio = 0.999;
    double guards_cov_dual_seed = std::random_device{}();
    unsigned guards_cov_dual_n_max = std::numeric_limits<unsigned>::max();
    unsigned guards_cov_dual_samp = 10;
    double guards_cov_dual_samp_sq = std::numeric_limits<double>::quiet_NaN();

    bool precompute_reflex_paths = false;
    bool precompute_cities_paths = false;

    bool solve_agp = false;
    bool solve_agp_tsp = false;

    std::string solve_d_search;
    std::string d_search_greedy_utility = "w/t";
    bool d_search_greedy_lookahead = false;
    double d_search_greedy_tree_size_ratio = 1.0;

    bool filter_coverage = false;

    double drawer_resolution = 0.025;
    double drawer_relative_frame_width = 0.02;
    std::string draw_map;
    std::string draw_mesh;
    std::string draw_target_regions;
    std::string draw_target_buckets;
    std::string draw_guards;
    std::string draw_coverage;
    std::string draw_uncovered_region;
    std::string draw_reflex_vis_graph;
    std::string draw_cities_vis_graph;
    std::string draw_reflex_shortest_paths;
    std::string draw_cities_shortest_paths;
    std::string draw_cities_path;

    bool record_map_properties = false;
    int record_vis_properties = -1;
    bool record_guards_stats = false;
    bool record_guards_coverage = false;

    std::string terminate;

    bool test_pathfinding = false;
};

/**
 *
 * Parses arguments and initializes logging.
 *
 * @param argc Number of arguments.
 * @param argv Array of arguments.
 * @param po
 * @return Character 'e' if an exception occurred, 'h' if --help option, '0' else.
 */
char ParseProgramOptions(
    int argc,
    const char *const *argv,
    visis::ProgramOptions &po
);

/**
 *
 *  ##########################################
 *  ## THIS IS THE MAIN BODY OF THE PROGRAM ##
 *  ##########################################
 *
 * @param po ~ program option variables
 * @return exit code
 */
int VisisProgram(const ProgramOptions &po);

}

#endif //VISIS_VISIS_PROGRAM_H_
