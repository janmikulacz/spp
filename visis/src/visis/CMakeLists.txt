# include boost:
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost 1.65 REQUIRED COMPONENTS program_options system filesystem log_setup log)

add_library(Visis
        core/coverage/approx_vis_region.cc
        core/coverage/buckets.cc
        core/coverage/coverage.cc
        core/coverage/filter_coverage.cc
        core/coverage/sample_coverage.cc
        core/coverage/target_regions.cc
        core/pathfinding/astar.cc
        core/pathfinding/boost_graph.cc
        core/pathfinding/dijkstra.cc
        core/pathfinding/path_finder.cc
        core/pathfinding/visibility_planner.cc
        core/guards/guards_ccdt.cc
        core/guards/guards_ka.cc
        core/guards/guards_random.cc
        core/guards/guards_reflex.cc
        core/guards/merge_triangles.cc
        core/guards/process_guards.cc
        core/guards/reflex_vertex.cc
        core/visibility/visibility_model.cc
        data/find_file.cc
        data/load_map.cc
        data/load_mesh.cc
        data/load_visibility_region.cc
        data/ptree_geom.cc
        drawing/drawing.cc
        drawing/fancy_drawing.cc
        drawing/random_colors.cc
        log/log.cc
        statistics/evaluate_vis.cc
        statistics/statistics.cc
        visis.cc
        visis_program.cc
        )

target_include_directories(Visis PUBLIC ${Visis_INCLUDE_DIR})

target_link_libraries(Visis
        Trivis
        ${Boost_LIBRARIES}
        cairo
        )

target_compile_options(Visis PUBLIC -std=c++17)

target_compile_definitions(Visis PRIVATE
        DEFAULT_MAP_DIR="${PROJECT_SOURCE_DIR}/../data"
        DEFAULT_OUT_DIR="${PROJECT_SOURCE_DIR}/.."
        )
