/**
 * File:   statistics.h
 *
 * Date:   12.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_STATISTICS_STATISTICS_H_
#define VISIS_STATISTICS_STATISTICS_H_

#include <vector>
#include <limits>
#include <algorithm>
#include <numeric>
#include <valarray>

#include <boost/property_tree/ptree.hpp>

namespace visis::statistics {

template<typename T>
struct VectorStatistics {
    int n = 0;
    T min = std::numeric_limits<T>::max();
    T max = -std::numeric_limits<T>::max();
    T sum = 0.0;
    T mean = 0.0;
    T sd = 0.0;
};

template<typename T>
VectorStatistics<T> ComputeVectorStatistics(const std::vector<T> &vec) {
    VectorStatistics<T> ret;
    ret.n = static_cast<int>(vec.size());
    for (const auto &v: vec) {
        if (v > ret.max) ret.max = v;
        if (v < ret.min) ret.min = v;
        ret.sum += v;
    }
    ret.mean = ret.sum / ret.n;
    std::vector<T> diff(ret.n);
    std::transform(vec.begin(), vec.end(), diff.begin(), [ret](T x) { return x - ret.mean; });
    T sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    ret.sd = std::sqrt(sq_sum / ret.n);
    return ret;
}

template<typename T>
boost::property_tree::ptree ToPTree(const VectorStatistics<T> &stat) {
    boost::property_tree::ptree ret;
    ret.put("n", stat.n);
    ret.put("min", stat.min);
    ret.put("max", stat.max);
    ret.put("sum", stat.sum);
    ret.put("mean", stat.mean);
    ret.put("sd", stat.sd);
    return ret;
}

}

#endif //VISIS_STATISTICS_STATISTICS_H_
