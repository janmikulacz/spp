/**
 * File:   process.cc
 *
 * Date:   25.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/guards/process_guards.h"

#include "visis/log/log.h"

#include "trivis/trivis.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::guards;

using namespace trivis;

using Clock = utils::SimpleClock;

void guards::RemoveDuplicates(
    Guards &guards
) {
    if (guards.empty()) {
        return;
    }
    auto n_with_duplicates = static_cast<int>(guards.size());
    // Create a vector of sensing location points and indices.
    std::vector<std::pair<int, geom::FPoint>> guards_simple(n_with_duplicates);
    for (int i = 0; i < n_with_duplicates; ++i) {
        guards_simple[i].first = i;
        guards_simple[i].second = guards[i].point;
    }
    // Sort the vector to locate the duplicates.
    std::sort(guards_simple.begin(), guards_simple.end(), [](const std::pair<int, geom::FPoint> &a, std::pair<int, geom::FPoint> &b) {
        return a.second.y < b.second.y || (a.second.y == b.second.y && a.second.x < b.second.x);
    });
    // Go over the sorted vector and add indices to remove if duplicates are detected.
    std::vector<int> indices_to_remove;
    for (int i = 0; i < n_with_duplicates; ++i) {
        int idx_first = guards_simple[i].first;
        const auto &point_first = guards_simple[i].second;
        for (int j = i + 1; j < n_with_duplicates; ++j, ++i) {
            if (guards_simple[j].second != point_first) {
                break;
            }
            // Location duplicate is detected!
            int idx = guards_simple[j].first;
            const auto &guard = guards[guards_simple[j].first];
            if (guard.is_anchor_0 || guard.is_anchor_n) {
                // If the duplicate is an anchor, remove the first location instead.
                const auto &guard_first = guards[idx_first];
                if (guard_first.is_anchor_0 || guard_first.is_anchor_n) {
                    // If the first location is also an anchor, then do not remove any location.
                    continue;
                }
                indices_to_remove.push_back(idx_first);
                continue;
            }
            // Remove the location duplicate.
            indices_to_remove.push_back(idx);
        }
    }
    // Actually remove the duplicates from the original vector.
    if (!indices_to_remove.empty()) {
        // Remove the elements in descending order of the indices.
        std::sort(indices_to_remove.begin(), indices_to_remove.end());
        std::reverse(indices_to_remove.begin(), indices_to_remove.end());
        for (int i: indices_to_remove) {
            guards.erase(guards.begin() + i);
        }
    }
}

inline RadialVisibilityRegion ComputeVisibilityRegion(
    const Trivis &vis,
    const std::optional<int> &id_v,
    const std::optional<geom::FPoint> &point,
    const std::optional<int> &point_tri_id,
    const visibility::VisibilityModel &vis_model,
    Trivis::ExpansionStats *stats
) {
    assert(id_v.has_value() || point.has_value());
    assert(point.has_value() == point_tri_id.has_value());
    std::optional<RadialVisibilityRegion> vis_reg;
    if (id_v.has_value()) {
        vis_reg = vis_model.VisibilityRegion(vis, id_v.value(), stats);
    } else {
        vis_reg = vis_model.VisibilityRegion(vis, point.value(), point_tri_id.value(), stats);
    }
    if (!vis_reg.has_value()) {
        return RadialVisibilityRegion{.radius = 0.0, .seed_id = id_v, .seed = id_v.has_value() ? vis.mesh().point(id_v.value()) : point.value()};
    }
    return vis_reg.value();
}

VisibilityRegionsWithStatistics guards::ComputeVisibilityRegions(
    Guards &guards,
    const Trivis &vis,
    const visibility::VisibilityModel &vis_model
) {
    if (guards.empty()) {
        return {};
    }
    Guards guards_out;
    VisibilityRegionsWithStatistics ret;
    for (int i = 0; i < guards.size(); ++i) {
        auto &guard = guards[i];
        if (guard.id_v.has_value() && vis.mesh().point(guard.id_v.value()) == guard.point) {
            // Case 1: The guard is known to be located at a vertex.
            VisRegionWithStats vis_reg_with_stat;
            Clock clock;
            vis_reg_with_stat.region = ComputeVisibilityRegion(vis, guard.id_v, std::nullopt, std::nullopt, vis_model, &vis_reg_with_stat.stats);
            vis_reg_with_stat.time = clock.TimeInSeconds();
            guards_out.push_back(std::move(guard));
            ret.push_back(std::move(vis_reg_with_stat));
            continue;
        }
        // Cases 2-5: The guard is not known to be located at a vertex.
        Clock clock;
        auto guard_pl = vis.LocatePoint(guard.point);
        double time_pl = clock.TimeInSeconds();
        if (!guard_pl.has_value() && (guard.is_anchor_0 || guard.is_anchor_n)) {
            // Case 2: The guard is an anchor and could not be located.
            LOGF_WRN("Could not compute visibility region from anchor [" << i << "] = " << guard.point << ". The visibility 'region' will be just a single point.");
            VisRegionWithStats vis_reg_with_stat;
            vis_reg_with_stat.region.seed = guard.point;
            vis_reg_with_stat.region.radius = 0.0;
            vis_reg_with_stat.time = time_pl;
            guards_out.push_back(std::move(guard));
            ret.push_back(std::move(vis_reg_with_stat));
            continue;
        }
        // Cases 3-5: The guard is not simultaneously an anchor and could not be located.
        if (!guard_pl.has_value()) {
            // Case 3: The guard could not be located.
            LOGF_WRN("Could not compute visibility region from guard [" << i << "] = " << guard.point << ". The guard wil be removed.");
            continue;
        }
        // Cases 4-5: The guard was located.
        if (guard_pl->snap_to_vertices.empty()) {
            // Case 4: The guard cannot be snapped to any vertex.
            VisRegionWithStats vis_reg_with_stat;
            clock.Restart();
            vis_reg_with_stat.region = ComputeVisibilityRegion(vis, std::nullopt, guard.point, guard_pl->tri_id, vis_model, &vis_reg_with_stat.stats);
            vis_reg_with_stat.time = time_pl + clock.TimeInSeconds();
            guard.id_t = guard_pl->tri_id;
            guards_out.push_back(std::move(guard));
            ret.push_back(std::move(vis_reg_with_stat));
            continue;
        }
        // Case 5: The guard can be snapped to one or more vertices.
        if (guard_pl->snap_to_vertices.size() > 1) {
            LOGF_WRN("Guard [" << i << "] = " << guard.point << " will be split into " << guard_pl->snap_to_vertices.size() << " guards.");
        }
        for (int ver_id: guard_pl->snap_to_vertices) {
            VisRegionWithStats vis_reg_with_stat;
            clock.Restart();
            vis_reg_with_stat.region = ComputeVisibilityRegion(vis, ver_id, std::nullopt, std::nullopt, vis_model, &vis_reg_with_stat.stats);
            vis_reg_with_stat.time = time_pl + clock.TimeInSeconds();
            Guard new_guard = guard;
            new_guard.id_v = ver_id;
            guards_out.push_back(std::move(new_guard));
            ret.push_back(std::move(vis_reg_with_stat));
        }
    }
    guards = std::move(guards_out);
    assert(ret.size() == guards.size()); // This is always satisfied unless this function itself is buggy.
    return ret;
}
