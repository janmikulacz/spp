/**
 * File:   visibility_model.h
 *
 * Date:   21.05.2024
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_CORE_VISIBILITY_VISIBILITY_MODEL_H_
#define VISIS_CORE_VISIBILITY_VISIBILITY_MODEL_H_

#include "trivis/trivis.h"

namespace visis::core::visibility {

class VisibilityModel {
public:

    explicit VisibilityModel(
        std::optional<double> vis_radius = std::nullopt,
        std::optional<double> robustness_radius = std::nullopt,
        std::optional<double> robustness_sample_dist = std::nullopt,
        std::optional<double> sample_arc_edges_angle = std::nullopt,
        std::optional<double> sample_arc_edges_dist = std::nullopt
    );

    [[nodiscard]] bool CheckValidity() const;

    [[nodiscard]] std::optional<trivis::RadialVisibilityRegion> VisibilityRegion(
        const trivis::Trivis &vis,
        const trivis::geom::FPoint &p,
        const trivis::Trivis::PointLocationResult &pl,
        trivis::Trivis::ExpansionStats *stats = nullptr
    ) const;

    [[nodiscard]] std::optional<trivis::RadialVisibilityRegion> VisibilityRegion(
        const trivis::Trivis &vis,
        const trivis::geom::FPoint &p,
        trivis::Trivis::ExpansionStats *stats = nullptr
    ) const;

    [[nodiscard]] std::optional<trivis::RadialVisibilityRegion> VisibilityRegion(
        const trivis::Trivis &vis,
        const trivis::geom::FPoint &p,
        int p_tri_id,
        trivis::Trivis::ExpansionStats *stats = nullptr
    ) const;

    [[nodiscard]] std::optional<trivis::RadialVisibilityRegion> VisibilityRegion(
        const trivis::Trivis &vis,
        int ver_id,
        trivis::Trivis::ExpansionStats *stats = nullptr
    ) const;

    [[nodiscard]] const auto &vis_radius() const {
        return _vis_radius;
    }

    [[nodiscard]] const auto &robustness_radius() const {
        return _robustness_radius;
    }

    [[nodiscard]] const auto &robustness_sample_dist() const {
        return _robustness_sample_dist;
    }

    [[nodiscard]] const auto &sample_arc_edges_angle() const {
        return _sample_arc_edges_angle;
    }

private:
    std::optional<double> _vis_radius;
    std::optional<double> _robustness_radius;
    std::optional<double> _robustness_sample_dist;
    std::optional<double> _sample_arc_edges_angle;
};

}

#endif //VISIS_CORE_VISIBILITY_VISIBILITY_MODEL_H_
