/**
 * File:   visis_program.cc
 *
 * Date:   11.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/visis_program.h"

#include <iostream>

// === BOOST INCLUDES ===

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

// === VISIS INCLUDES ===

#include "visis/visis.h"

// === TRIVIS INCLUDES ===

#include "trivis/trivis.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace visis;

void AddProgramOptions(
    po::options_description &options_description,
    visis::ProgramOptions &po,
    bool &verbosity_silent,
    bool &verbosity_quiet,
    bool &verbosity_verbose
) {
    options_description.add_options()
        ("help,h", "Produce this help message. \n (*) Overwrites options: all.")
        ("verbose",
         po::bool_switch(&verbosity_verbose)->default_value(verbosity_verbose),
         "Log messages: fatal, error, warning, buckets, debug.")
        ("quiet",
         po::bool_switch(&verbosity_quiet)->default_value(verbosity_quiet),
         "Log messages: fatal, error, warning. \n (*) Overwrites options: verbose.")
        ("silent",
         po::bool_switch(&verbosity_silent)->default_value(verbosity_silent),
         "Log messages: fatal, error. \n (*) Overwrites options: verbose, quiet.")
        ("irace",
         po::bool_switch(&po.irace)->default_value(po.irace),
         "A special switch to tell the program it is being tuned by irace.")
        ("map_name",
         po::value(&po.map_name)->default_value(po.map_name),
         "Map name.")
        ("map_ext",
         po::value(&po.map_extension)->default_value(po.map_extension),
         "Map file extension.")
        ("map_dir",
#ifdef DEFAULT_MAP_DIR
         po::value(&po.map_dir)->default_value(DEFAULT_MAP_DIR),
#elif
        po::value(&po.map_dir)->default_value(po.map_dir);
#endif
         "Directory where the program will search for the map.")
        ("map_full_path",
         po::value(&po.map_full_path)->default_value(po.map_full_path),
         "Map full path. \n (*) Overwrites options: map_name, map_ext, map_dir.")
        ("map_scale",
         po::value(&po.map_scale)->default_value(po.map_scale),
         "Map scale.")
        ("out_dir",
#ifdef DEFAULT_OUT_DIR
         po::value(&po.out_dir)->default_value(DEFAULT_OUT_DIR),
#elif
        po::value(&po.out_dir)->default_value(po.out_dir);
#endif
         "Output directory.")
        ("out_file",
         po::value(&po.out_file)->default_value(po.out_file),
         "Output file.")
        ("vis_radius",
         po::value(&po.vis_radius)->default_value(po.vis_radius),
         "Visibility radius of the visibility model.")
        ("robustness_radius",
         po::value(&po.robustness_radius)->default_value(po.robustness_radius),
         "Robustness radius of the visibility model.")
        ("robustness_sample_dist",
         po::value(&po.robustness_sample_dist)->default_value(po.robustness_sample_dist),
         "Robustness sample distance of the visibility model.")
        ("sample_arc_edges_angle",
         po::value(&po.sample_arc_edges_angle)->default_value(po.sample_arc_edges_angle),
         "Sampling angle used in computing vis. region approximations.")
        ("sample_arc_edges_dist",
         po::value(&po.sample_arc_edges_dist)->default_value(po.sample_arc_edges_dist),
         "Sampling distance used in computing vis. region approximations.")
        ("rob_radius",
         po::value(&po.rob_radius)->default_value(po.rob_radius),
         "Radius of the robot's base.")
        ("cycle",
         po::value(&po.route_cyclic)->default_value(po.route_cyclic),
         "Set if the route is cyclic or not.")
        ("init",
         po::value(&po.route_anchor_init)->multitoken()->default_value(po.route_anchor_init, ""),
         "Set the route init anchor. Format: 'x-coord y-coord'.")
        ("exit",
         po::value(&po.route_anchor_exit)->multitoken()->default_value(po.route_anchor_exit, ""),
         "Set the route exit anchor. Format: 'x-coord y-coord'.")
        ("vis_bucket_size",
         po::value(&po.vis_bucket_size)->default_value(po.vis_bucket_size),
         "Size of the buckets used when triangle-localizing arbitrary points in O(1).")
        ("vis_optimize_buckets",
         po::bool_switch(&po.vis_optimize_buckets)->default_value(po.vis_optimize_buckets),
         "If used, then the buckets are optimized to slightly improve the mean triangle-localizing performance.")
        ("coverage",
         po::value(&po.min_coverage_ratio)->default_value(po.min_coverage_ratio),
         "Minimal coverage ratio: how much of the regions of interest must be covered.")
        ("target_bucket_ratio",
         po::value(&po.target_bucket_ratio)->default_value(po.target_bucket_ratio),
         "Determines the size of the buckets used to speed up polygon clipping. The size is computed as bucket_ratio * max(width, height).")
        ("target_bucket_size",
         po::value(&po.target_bucket_size)->default_value(po.target_bucket_size),
         "Size of the buckets used to speed up polygon clipping.")
        ("weight_bucket_ratio",
         po::value(&po.weight_bucket_ratio)->default_value(po.weight_bucket_ratio),
         "Determines the size of the buckets used to speed up polygon clipping. The size is computed as bucket_ratio * max(width, height).")
        ("weight_bucket_size",
         po::value(&po.weight_bucket_size)->default_value(po.weight_bucket_size),
         "Size of the buckets used to speed up polygon clipping.")
        ("guards",
         po::value(&po.generate_guards)->multitoken()->default_value(po.generate_guards, ""),
         "Instruction to generate the guards. The argument is a space-separated list of methods (order matters).")
        ("guards_random_n",
         po::value(&po.guards_random_n)->default_value(po.guards_random_n),
         "Param of 'rand': How many guards should be generated.")
        ("guards_random_seed",
         po::value(&po.guards_random_seed)->default_value(po.guards_random_seed),
         "Param of 'rand': Random seed used to generate the guards. The default is generated by std::random_device{}().")
        ("guards_reflex_deg",
         po::value(&po.guards_reflex_deg)->default_value(po.guards_reflex_deg),
         "Param of 'reflex*': The tolerance in degrees used to determine the reflex map vertices. Vertices whose angle is closer to 180 deg than the tolerance are rejected.")
        ("guards_reflex_dist",
         po::value(&po.guards_reflex_dist)->default_value(po.guards_reflex_dist),
         "Param of 'reflex*': The distance of the reflex guards from the corresponding reflex vertices (small positive value).")
        ("guards_reflex_n_max",
         po::value(&po.guards_reflex_n_max)->default_value(po.guards_reflex_n_max),
         "Param of 'reflex*': The maximum number of guards that is generated.")
        ("guards_reflex_rand_seed",
         po::value(&po.guards_reflex_rand_seed)->default_value(po.guards_reflex_rand_seed),
         "Param of 'reflex-rand': Random seed used to generate the guards. The default is generated by std::random_device{}().")
        ("guards_ccdt_n_max",
         po::value(&po.guards_ccdt_n_max)->default_value(po.guards_ccdt_n_max),
         "Param of 'ccdt*': The maximum number of guards that is generated.")
        ("guards_ccdt_rand_seed",
         po::value(&po.guards_ccdt_rand_seed)->default_value(po.guards_ccdt_rand_seed),
         "Param of 'ccdt-rand': Random seed used to generate the guards. The default is generated by std::random_device{}().")
        ("guards_ka_order",
         po::value(&po.guards_ka_order)->default_value(po.guards_ka_order),
         "Param of 'ka*': The ordering type used when merging triangles to convex polygons. \n -> Available options: default, small, large, ears, ears_small, ears_large, open, open_small, open_large.")
        ("guards_ka_rule",
         po::value(&po.guards_ka_rule)->default_value(po.guards_ka_rule),
         "Param of 'ka*': The selection rule used when merging triangles to convex polygons. \n -> Available options: first, best.")
        ("guards_ka_n_max",
         po::value(&po.guards_ka_n_max)->default_value(po.guards_ka_n_max),
         "Param of 'ka*': The maximum number of guards that is generated.")
        ("guards_ka_rand_seed",
         po::value(&po.guards_ka_rand_seed)->default_value(po.guards_ka_rand_seed),
         "Param of 'ka-rand': Random seed used to generate the guards. The default is generated by std::random_device{}().")
        ("guards_cov_rand_ratio",
         po::value(&po.guards_cov_rand_ratio)->default_value(po.guards_cov_rand_ratio),
         "Param of 'cov-rand': How much of the regions of interest must be covered.")
        ("guards_cov_rand_seed",
         po::value(&po.guards_cov_rand_seed)->default_value(po.guards_cov_rand_seed),
         "Param of 'cov-rand': Random seed used to generate the guards. The default is generated by std::random_device{}().")
        ("guards_cov_rand_n_max",
         po::value(&po.guards_cov_rand_n_max)->default_value(po.guards_cov_rand_n_max),
         "Param of 'cov-rand': The maximum number of guards that is generated.")
        ("guards_cov_inf_ratio",
         po::value(&po.guards_cov_inf_ratio)->default_value(po.guards_cov_inf_ratio),
         "Param of 'cov-inf': How much of the regions of interest must be covered.")
        ("guards_cov_inf_seed",
         po::value(&po.guards_cov_inf_seed)->default_value(po.guards_cov_inf_seed),
         "Param of 'cov-inf': Random seed used to generate the guards. The default is generated by std::random_device{}().")
        ("guards_cov_inf_n_max",
         po::value(&po.guards_cov_inf_n_max)->default_value(po.guards_cov_inf_n_max),
         "Param of 'cov-inf': The maximum number of guards that is generated.")
        ("guards_cov_dual_ratio",
         po::value(&po.guards_cov_dual_ratio)->default_value(po.guards_cov_dual_ratio),
         "Param of 'cov-dual': How much of the regions of interest must be covered.")
        ("guards_cov_dual_seed",
         po::value(&po.guards_cov_dual_seed)->default_value(po.guards_cov_dual_seed),
         "Param of 'cov-dual': Random seed used to generate the guards. The default is generated by std::random_device{}().")
        ("guards_cov_dual_n_max",
         po::value(&po.guards_cov_dual_n_max)->default_value(po.guards_cov_dual_n_max),
         "Param of 'cov-dual': The maximum number of guards that is generated.")
        ("guards_cov_dual_samp",
         po::value(&po.guards_cov_dual_samp)->default_value(po.guards_cov_dual_samp),
         "Param of 'cov-dual': The number of dual samples used in each iteration of the algorithm.")
        ("guards_cov_dual_samp_sq",
         po::value(&po.guards_cov_dual_samp_sq)->default_value(po.guards_cov_dual_samp_sq),
         "Param of 'cov-dual': The number of dual samples used in each square unit. An alternative to 'guards_cov_dual_samp'.")
        ("filter",
         po::bool_switch(&po.filter_coverage)->default_value(po.filter_coverage),
         "Will filter the coverage.")
        ("precomp_reflex",
         po::bool_switch(&po.precompute_reflex_paths)->default_value(po.precompute_reflex_paths),
         "Precomputes the reflex shortest paths.")
        ("precomp_cities",
         po::bool_switch(&po.precompute_cities_paths)->default_value(po.precompute_cities_paths),
         "Precomputes the cities shortest paths.")
        ("agp",
         po::bool_switch(&po.solve_agp)->default_value(po.solve_agp),
         "Solves the art gallery problem.")
        ("agp_tsp",
         po::bool_switch(&po.solve_agp_tsp)->default_value(po.solve_agp_tsp),
         "Solves the art gallery problem followed by the traveling salesperson problem.")
        ("d_search",
         po::value(&po.solve_d_search)->default_value(po.solve_d_search),
         "Solves the discrete search using the provided method. \n -> Available options: greedy.")
        ("d_search_greedy_utility",
         po::value(&po.d_search_greedy_utility)->default_value(po.d_search_greedy_utility),
         "Utility used by the greedy algorithm. \n -> Available options: w, -t, -T, w/t, w/T, -wt, -wT, U+w, U-t, U-T, U+w/t, U+w/T, U-wt, U-wT.")
        ("d_search_greedy_lookahead",
         po::bool_switch(&po.d_search_greedy_lookahead)->default_value(po.d_search_greedy_lookahead),
         "If used, the greedy algorithm looks several steps ahead.")
        ("d_search_greedy_tree_size_ratio",
         po::value(&po.d_search_greedy_tree_size_ratio)->default_value(po.d_search_greedy_tree_size_ratio),
         "Limits the size of the search tree when looking ahead.")
        ("drawer_resolution",
         po::value(&po.drawer_resolution)->default_value(po.drawer_resolution),
         "1/drawer_resolution = Number of pixel divisions per 1 metric unit.")
        ("drawer_relative_frame_width",
         po::value(&po.drawer_relative_frame_width)->default_value(po.drawer_relative_frame_width),
         "A frame of the map is created with width = drawer_relative_frame_width * max(map's metric width, map's metric height).")
        ("draw_map",
         po::value(&po.draw_map)->default_value(po.draw_map),
         "Draws the map. Available formats: PDF, PNG.")
        ("draw_mesh",
         po::value(&po.draw_mesh)->default_value(po.draw_mesh),
         "Draws the mesh. Available formats: PDF, PNG.")
        ("draw_target_regions",
         po::value(&po.draw_target_regions)->default_value(po.draw_target_regions),
         "Draws the regions of interest. Available formats: PDF, PNG.")
        ("draw_target_buckets",
         po::value(&po.draw_target_buckets)->default_value(po.draw_target_buckets),
         "Draws the buckets. Available formats: PDF, PNG.")
        ("draw_guards",
         po::value(&po.draw_guards)->default_value(po.draw_guards),
         "Draws the guards. Available formats: PDF, PNG.")
        ("draw_coverage",
         po::value(&po.draw_coverage)->default_value(po.draw_coverage),
         "Draws the coverage. Available formats: PDF, PNG.")
        ("draw_uncovered_region",
         po::value(&po.draw_uncovered_region)->default_value(po.draw_uncovered_region),
         "Draws the uncovered_region. Available formats: PDF, PNG.")
        ("draw_reflex_vis_graph",
         po::value(&po.draw_reflex_vis_graph)->default_value(po.draw_reflex_vis_graph),
         "Draws the reflex visibility graph. Available formats: PDF, PNG.")
        ("draw_cities_vis_graph",
         po::value(&po.draw_cities_vis_graph)->default_value(po.draw_cities_vis_graph),
         "Draws the visibility graph. Available formats: PDF, PNG.")
        ("draw_reflex_shortest_paths",
         po::value(&po.draw_reflex_shortest_paths)->default_value(po.draw_reflex_shortest_paths),
         "Draws the reflex shortest paths. Available formats: PDF, PNG.")
        ("draw_cities_shortest_paths",
         po::value(&po.draw_cities_shortest_paths)->default_value(po.draw_cities_shortest_paths),
         "Draws the cities shortest paths. Available formats: PDF, PNG.")
        ("draw_cities_path",
         po::value(&po.draw_cities_path)->default_value(po.draw_cities_path),
         "Draws the cities path. Available formats: PDF, PNG.")
        ("record_map_properties",
         po::bool_switch(&po.record_map_properties)->default_value(po.record_map_properties),
         "If used, then map properties are evaluated and recorded.")
        ("record_vis_properties",
         po::value(&po.record_vis_properties)->default_value(po.record_vis_properties),
         "If positive integer value is provided, then the TriVis properties are evaluated and recorded. \n (*) The value is the number of random points during the evaluation.")
        ("record_guards_stats",
         po::bool_switch(&po.record_guards_stats)->default_value(po.record_guards_stats),
         "Records some detailed statistics during guards' postprocessing.")
        ("record_guards_coverage",
         po::bool_switch(&po.record_guards_coverage)->default_value(po.record_guards_coverage),
         "Records some detailed statistics during guards' postprocessing.")
        ("terminate",
         po::value(&po.terminate)->default_value(po.terminate),
         "Tells the program when to terminate. Options: record_map_properties")
        ("test_pathfinding",
         po::bool_switch(&po.test_pathfinding)->default_value(po.test_pathfinding),
         "Tests pathfinding algorithms and terminates the program.");
}

visis::log::severity_level GetSeverity(
    bool verbosity_silent,
    bool verbosity_quiet,
    bool verbosity_verbose
) {
    using namespace visis::log;
    if (verbosity_silent) {
        return severity_level::error;
    } else if (verbosity_quiet) {
        return severity_level::warning;
    } else if (verbosity_verbose) {
        return severity_level::debug;
    } else {
        return severity_level::info;
    }
}

char visis::ParseProgramOptions(
    int argc,
    const char *const *argv,
    visis::ProgramOptions &po
) {
    using namespace visis::log;
    if (argc < 2) {
        InitLogging(severity_level::info);
        LOGF_INF("\n" << kVisisASCIIArt);
        LOGF_FTL("No arguments provided. Use --help to see available options.");
        return 'e';
    }
    po::variables_map vm;
    po::options_description command_line_options;
    po::options_description options_description("General options");
    bool verbosity_silent = false;
    bool verbosity_quiet = false;
    bool verbosity_verbose = false;
    AddProgramOptions(options_description, po, verbosity_silent, verbosity_quiet, verbosity_verbose);
    try {
        // Parse the command line arguments.
        command_line_options.add(options_description);
        po::store(po::parse_command_line(argc, argv, command_line_options), vm);
        if (vm.count("help")) {
            // If '-h' or '--help' option, print the options and return 'h'.
            command_line_options.print(std::cout, 80);
            return 'h';
        }
        po::notify(vm);
    } catch (const std::exception &e) {
        // If exception, log it and return 'e'.
        InitLogging(severity_level::info);
        LOGF_INF("\n" << kVisisASCIIArt);
        LOGF_FTL("Error in parsing arguments: " << e.what() << ".");
        return 'e';
    }
    if (po.irace) verbosity_silent = true;
    InitLogging(GetSeverity(verbosity_silent, verbosity_quiet, verbosity_verbose));
    LOGF_INF("\n" << kVisisASCIIArt);
    return '0';
}

bool LoadMap(const ProgramOptions &po, Visis &visis) {
    std::optional<double> map_scale = std::isnan(po.map_scale) ? std::nullopt : std::make_optional(po.map_scale);
    if (!po.map_full_path.empty()) {
        return visis.LoadMap(po.map_full_path, std::nullopt, std::nullopt, map_scale);
    } else {
        return visis.LoadMap(po.map_name, po.map_extension, po.map_dir, map_scale);
    }
}

int visis::VisisProgram(const ProgramOptions &po) {

    double irace_solution_cost = 0.0;
    double irace_time = 0.0;
    trivis::utils::SimpleClock clock_total;

    Visis visis;

    if (!fs::exists(po.out_dir)) {
        if (!fs::create_directories(po.out_dir)) {
            LOGF_FTL("Could not create directory " << po.out_dir << ".");
            return EXIT_FAILURE;
        }
    }

    if (!LoadMap(po, visis)) {
        return EXIT_FAILURE;
    }

    if (po.record_map_properties) {
        visis.RecordMapProperties();
        if (po.terminate == "record_map_properties") {
            visis.PrintRecords();
            if (!po.out_file.empty()) {
                if (!visis.WriteRecords(po.out_dir + "/" + po.out_file)) {
                    return EXIT_FAILURE;
                }
            }
            return EXIT_SUCCESS;
        }
    }

    { // visis.SetVisibilityModel
        auto vis_radius = (std::isfinite(po.vis_radius) && po.vis_radius > 0.0) ? std::make_optional(po.vis_radius) : std::nullopt;
        auto robustness_radius = (std::isfinite(po.robustness_radius) && po.robustness_radius > 0.0) ? std::make_optional(po.robustness_radius) : std::nullopt;
        auto robustness_sample_dist = (std::isfinite(po.robustness_sample_dist) && po.robustness_sample_dist > 0.0) ? std::make_optional(po.robustness_sample_dist) : std::nullopt;
        auto sample_arc_edges_angle = (std::isfinite(po.sample_arc_edges_angle) && po.sample_arc_edges_angle > 0.0) ? std::make_optional(po.sample_arc_edges_angle) : std::nullopt;
        auto sample_arc_edges_dist = (std::isfinite(po.sample_arc_edges_dist) && po.sample_arc_edges_dist > 0.0) ? std::make_optional(po.sample_arc_edges_dist) : std::nullopt;
        if (!visis.SetVisibilityModel(vis_radius, robustness_radius, robustness_sample_dist, sample_arc_edges_angle, sample_arc_edges_dist)) {
            return EXIT_FAILURE;
        }
    }

    if (!visis.SetRobotRadius(po.rob_radius)) {
        return EXIT_FAILURE;
    }

    if (!visis.InitDrawer(po.drawer_resolution, po.drawer_relative_frame_width)) {
        return EXIT_FAILURE;
    }

    if (!po.draw_map.empty()) {
        visis.DrawMap(po.out_dir + "/" + po.draw_map);
    }

    if (!visis.InitVis(po.vis_bucket_size, po.vis_optimize_buckets)) {
        return EXIT_FAILURE;
    }

    if (!po.draw_mesh.empty()) {
        visis.DrawMesh(po.out_dir + "/" + po.draw_mesh);
    }

    if (po.record_vis_properties >= 0) {
        visis.RecordVisProperties(po.record_vis_properties);
    }

    if (!visis.ConstructReflexVisibilityGraph()) {
        return EXIT_FAILURE;
    }

    if (!po.draw_reflex_vis_graph.empty()) {
        visis.DrawReflexVisibilityGraph(po.out_dir + "/" + po.draw_reflex_vis_graph);
    }

    if (po.precompute_reflex_paths) {
        if (!visis.PrecomputeReflexShortestPaths()) {
            return EXIT_FAILURE;
        }
    }

    if (!po.draw_reflex_shortest_paths.empty()) {
        visis.DrawReflexShortestPaths(po.out_dir + "/" + po.draw_reflex_shortest_paths);
    }

    std::optional<std::vector<trivis::geom::FPolygons>> target_regions = std::nullopt;
    // target_regions = std::vector<trivis::geom::FPolygons>{};
    // target_regions->push_back({{{100.0, 100.0}, {400.0, 100.0}, {400.0, 400.0}, {100.0, 400.0}}});
    if (!visis.SetTargetRegions(po.min_coverage_ratio, target_regions)) {
        return EXIT_FAILURE;
    }

    { // visis.BucketizeTargetRegions
        auto bucket_ratio = (std::isnan(po.target_bucket_ratio) || po.target_bucket_ratio <= 0.0) ? std::nullopt : std::make_optional(po.target_bucket_ratio);
        auto bucket_size = (std::isnan(po.target_bucket_size) || po.target_bucket_size <= 0.0) ? std::nullopt : std::make_optional(po.target_bucket_size);
        if (!visis.BucketizeTargetRegions(bucket_ratio, bucket_size)) {
            return EXIT_FAILURE;
        }
    }

    std::optional<std::vector<trivis::geom::FPolygons>> weight_regions = std::nullopt;
    std::optional<std::vector<double>> weights = std::nullopt;
    // weight_regions = target_regions;
    // weight_regions->push_back({{{150.0, 150.0}, {1000.0, 150.0}, {1000.0, 1000.0}, {150.0, 1000.0}}});
    // weight_regions->push_back({{{200.0, 200.0}, {1000.0, 200.0}, {1000.0, 1000.0}, {200.0, 1000.0}}});
    // weight_regions->push_back({{{250.0, 250.0}, {1000.0, 250.0}, {1000.0, 1000.0}, {250.0, 1000.0}}});
    // weights = std::vector<double>{};
    // weights->push_back(2);
    // weights->push_back(2);
    // weights->push_back(4);
    // weights->push_back(8);
    if (!visis.SetWeightRegions(weight_regions, weights)) {
        return EXIT_FAILURE;
    }

    { // visis.BucketizeWeightRegions
        auto bucket_ratio = (std::isnan(po.weight_bucket_ratio) || po.weight_bucket_ratio <= 0.0) ? std::nullopt : std::make_optional(po.weight_bucket_ratio);
        auto bucket_size = (std::isnan(po.weight_bucket_size) || po.weight_bucket_size <= 0.0) ? std::nullopt : std::make_optional(po.weight_bucket_size);
        if (!visis.BucketizeWeightRegions(bucket_ratio, bucket_size)) {
            return EXIT_FAILURE;
        }
    }

    if (!po.draw_target_regions.empty()) {
        visis.DrawTargetRegions(po.out_dir + "/" + po.draw_target_regions);
    }

    if (!po.draw_target_buckets.empty()) {
        visis.DrawTargetBuckets(po.out_dir + "/" + po.draw_target_buckets);
    }

    { // visis.SetRouteProperties
        std::optional<trivis::geom::FPoint> anchor_start, anchor_end;
        for (const auto &anchor: {std::make_pair(&po.route_anchor_init, &anchor_start), std::make_pair(&po.route_anchor_exit, &anchor_end)}) {
            if (anchor.first->size() == 2) {
                *anchor.second = trivis::geom::MakePoint(anchor.first->operator[](0), anchor.first->operator[](1));
            } else if (!anchor.first->empty()) {
                LOGF_FTL("Incorrect number of anchor coordinates!");
                return EXIT_FAILURE;
            }
        }
        if (!visis.SetRouteProperties(po.route_cyclic, std::move(anchor_start), std::move(anchor_end))) {
            return EXIT_FAILURE;
        }
    }

    if (!po.generate_guards.empty()) {
        for (const auto &method: po.generate_guards) {
            Visis::GuardsParam param;
            param.record_vis_reg_stats = po.record_guards_stats;
            param.record_coverage = po.record_guards_coverage;
            auto method_lower = boost::algorithm::to_lower_copy(method);
            if (method_lower == "random") {
                param.n_max = po.guards_random_n;
                param.random_seed = po.guards_random_seed;
                if (!visis.GenerateGuards("random", param)) {
                    return EXIT_FAILURE;
                }
                continue;
            }
            if (method_lower.rfind("reflex", 0) != std::string::npos) {
                param.reflex_tolerance = po.guards_reflex_deg;
                param.reflex_dist_from_vertex = po.guards_reflex_dist;
                if (po.guards_reflex_n_max != std::numeric_limits<unsigned>::max()) {
                    param.n_max = po.guards_reflex_n_max;
                }
                if (method_lower.find("-rand") != std::string::npos) {
                    param.random_seed = po.guards_reflex_rand_seed;
                }
                if (!visis.GenerateGuards("reflex", param)) {
                    return EXIT_FAILURE;
                }
                continue;
            }
            if (method_lower.rfind("ccdt", 0) != std::string::npos) {
                if (po.guards_ccdt_n_max != std::numeric_limits<unsigned>::max()) {
                    param.n_max = po.guards_ccdt_n_max;
                }
                if (method_lower.find("-rand") != std::string::npos) {
                    param.random_seed = po.guards_ccdt_rand_seed;
                }
                if (!visis.GenerateGuards("ccdt", param)) {
                    return EXIT_FAILURE;
                }
                continue;
            }
            if (method_lower.rfind("ka", 0) != std::string::npos) {
                param.ka_ordering = po.guards_ka_order;
                param.ka_selection_rule = po.guards_ka_rule;
                if (po.guards_ka_n_max != std::numeric_limits<unsigned>::max()) {
                    param.n_max = po.guards_ka_n_max;
                }
                if (method_lower.find("-rand") != std::string::npos) {
                    param.random_seed = po.guards_ka_rand_seed;
                }
                if (!visis.GenerateGuards("ka", param)) {
                    return EXIT_FAILURE;
                }
                continue;
            }
            if (method_lower == "cov-rand") {
                param.random_seed = po.guards_cov_rand_seed;
                param.min_coverage_ratio = po.guards_cov_rand_ratio;
                if (po.guards_cov_rand_n_max != std::numeric_limits<unsigned>::max()) {
                    param.n_max = po.guards_cov_rand_n_max;
                }
                if (!visis.GenerateGuards("cov-rand", param)) {
                    return EXIT_FAILURE;
                }
                continue;
            }
            if (method_lower == "cov-inf") {
                param.random_seed = po.guards_cov_inf_seed;
                param.min_coverage_ratio = po.guards_cov_inf_ratio;
                if (po.guards_cov_inf_n_max != std::numeric_limits<unsigned>::max()) {
                    param.n_max = po.guards_cov_inf_n_max;
                }
                if (!visis.GenerateGuards("cov-inf", param)) {
                    return EXIT_FAILURE;
                }
                continue;
            }
            if (method_lower == "cov-dual") {
                param.random_seed = po.guards_cov_dual_seed;
                param.min_coverage_ratio = po.guards_cov_dual_ratio;
                if (po.guards_cov_dual_n_max != std::numeric_limits<unsigned>::max()) {
                    param.n_max = po.guards_cov_dual_n_max;
                }
                if (std::isfinite(po.guards_cov_dual_samp_sq) && po.guards_cov_dual_samp_sq > 0.0) {
                    param.n_dual_samples_per_square_unit = po.guards_cov_dual_samp_sq;
                    if (po.guards_cov_dual_samp != 0) {
                        param.n_dual_samples = po.guards_cov_dual_samp;
                    }
                } else {
                    param.n_dual_samples = po.guards_cov_dual_samp;
                }
                if (!visis.GenerateGuards("cov-dual", param)) {
                    return EXIT_FAILURE;
                }
                continue;
            }
            LOGF_ERR("Unknown method for generating guards: '" << method << "'.");
        }
    }

    if (!visis.ProcessCoverage(po.filter_coverage)) {
        return EXIT_FAILURE;
    }

    if (!po.draw_guards.empty()) {
        visis.DrawGuards(po.out_dir + "/" + po.draw_guards);
    }

    if (!po.draw_coverage.empty()) {
        visis.DrawCoverage(po.out_dir + "/" + po.draw_coverage);
    }

    if (!po.draw_uncovered_region.empty()) {
        visis.DrawUncoveredRegion(po.out_dir + "/" + po.draw_uncovered_region);
    }

    if (!visis.ApplyFilter()) { // Must be called even if po.filter_coverage is false.
        return EXIT_FAILURE;
    }

    if (po.test_pathfinding) {
        visis.TestPathfinding();
        return EXIT_SUCCESS;
    }

    if (!visis.ConstructCitiesVisibilityGraph()) {
        return EXIT_FAILURE;
    }

    if (!po.draw_cities_vis_graph.empty()) {
        visis.DrawCitiesVisibilityGraph(po.out_dir + "/" + po.draw_cities_vis_graph);
    }

    if (po.precompute_cities_paths) {
        if (!visis.PrecomputeCitiesShortestPaths()) {
            return EXIT_FAILURE;
        }
    }

    if (!po.draw_cities_shortest_paths.empty()) {
        visis.DrawCitiesShortestPaths(po.out_dir + "/" + po.draw_cities_shortest_paths);
    }

    if (po.solve_agp) {
        const auto &rec = visis.records();
        // Get the number of guards.
        int agp_n = rec.GetChild("process_coverage").Get<int>("n_after_filtering");
        // Get the time to generate the guards (collect only necessary times).
        double agp_time = 0.0;
        agp_time += rec.GetChild("vis").Get<double>("time_total");
        agp_time += rec.GetChild("target_regions").Get<double>("time");
        agp_time += rec.GetChild("target_regions_bucketing").Get<double>("time");
        for (int i = 0; i < po.generate_guards.size(); ++i) {
            agp_time += rec.GetChild("guards_" + std::to_string(i + 1)).Get<double>("time_total");
        }
        agp_time += rec.GetChild("process_coverage").Get<double>("time");
        // Write the metrics down.
        visis.records_writable().Put("agp_n", agp_n);
        visis.records_writable().Put("agp_time", agp_time);
        LOGF_INF("AGP number of guards: " << agp_n << ".");
        LOGF_INF("AGP time: " << agp_time << ".");
    }

    double time_total = clock_total.TimeInSeconds();

    LOGF_INF("Time total: " << time_total << " s.");

    return EXIT_SUCCESS;
}
