/**
 * File:   coverage.cc
 *
 * Date:   25.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/core/coverage/coverage.h"

using namespace visis;
using namespace visis::core;
using namespace visis::core::coverage;

using namespace trivis;

Clipper2Lib::Paths64 coverage::Difference(
    const Clipper2Lib::Paths64 &subject,
    const Coverage &clip,
    int cluster_size
) {
    Clipper2Lib::Paths64 ret = subject;
    Clipper2Lib::Paths64 clips_cluster;
    Clipper2Lib::Clipper64 clipper;
    int n = static_cast<int>(clip.size());
    for (int k = 0; k < n; ++k) {
        clips_cluster.push_back(clip[k].approx.clipper);
        if (clips_cluster.size() == cluster_size || k + 1 == n) {
            clipper.AddSubject(ret);
            clipper.AddClip(clips_cluster);
            clipper.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, ret);
            clipper.Clear();
            clips_cluster.clear();
            if (ret.empty()) {
                break;
            }
        }
    }
    return ret;
}
