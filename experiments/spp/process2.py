#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# python3 process2.py -c val -i data/val.csv
# python3 process2.py -c test -i data/test.csv
# python3 process2.py -c extra -i data/extra.csv

import argparse

import math
import copy as cp

import scipy as sp
import numpy as np

import datatable as dt
from datatable import f

from pathlib import Path

import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


def args_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--context', type=str, help='Context: {val,test}')
    parser.add_argument('-i', '--input_csv_file', type=str, help='Input table in CSV format.')
    return parser


def compute_tab_summaries_per_vis_radius(
        tab: dt.Frame,
):
    tab_summaries = {}
    full_tables = {}
    cost_key = 'agp_n'
    time_key = 'agp_time'
    vis_radii = dt.unique(tab[:, 'vis_radius']).to_list()[0]
    vis_radii.sort(reverse=True)
    for d in vis_radii:
        tab_d = tab[f.vis_radius == d, :]
        print(f'vis_radius: {d:5}, num_rows: {tab_d.nrows:4}')
        # print method types and exit
        min_cost_per_map = tab_d[:, {cost_key: dt.min(f[cost_key])}, dt.by('map.name')].to_tuples()
        min_time_per_map = tab_d[:, {time_key: dt.min(f[time_key])}, dt.by('map.name')].to_tuples()
        max_time_per_map = dict(tab_d[:, {time_key: dt.max(f[time_key])}, dt.by('map.name')].to_tuples())
        tab_d['gap'] = math.inf
        for map_name, min_cost in min_cost_per_map:
            tab_d[f['map.name'] == map_name, 'cost'] = f[cost_key]
            tab_d[f['map.name'] == map_name, 'gap'] = 100.0 * (f[cost_key] - min_cost) / min_cost
        tab_d['norm_time'] = f[time_key]
        for map_name, min_time in min_time_per_map:
            max_time = float(max_time_per_map[map_name])
            tab_d[f['map.name'] == map_name, 'time'] = f[time_key]
            tab_d[f['map.name'] == map_name, 'norm_time'] = 100.0 * (f[time_key] - min_time) / (max_time - min_time)
        full_tables[d] = tab_d
        summary_tables = {}
        group_by = {
            'map': ['map.name'],
            'method': ['method_type', 'method', 'method_full', 'dual_samp', 'filtered', 'bucketed', 'filtered_bucketed'],
        }
        group_by['map_method'] = group_by['map'] + group_by['method']
        for key, group_by_list in group_by.items():
            summary_tables[key] = tab_d[:, {
                                               'cost_min': dt.min(f.cost),
                                               'cost_max': dt.max(f.cost),
                                               'cost_median': dt.median(f.cost),
                                               'cost_mean': dt.mean(f.cost),
                                               'cost_std': dt.sd(f.cost),
                                               'time_min': dt.min(f.time),
                                               'time_median': dt.median(f.time),
                                               'time_max': dt.max(f.time),
                                               'time_mean': dt.mean(f.time),
                                               'time_std': dt.sd(f.time),
                                               'gap_min': dt.min(f.gap),
                                               'gap_median': dt.median(f.gap),
                                               'gap_max': dt.max(f.gap),
                                               'gap_mean': dt.mean(f.gap),
                                               'gap_std': dt.sd(f.gap),
                                               'norm_time_min': dt.min(f.norm_time),
                                               'norm_time_median': dt.median(f.norm_time),
                                               'norm_time_max': dt.max(f.norm_time),
                                               'norm_time_mean': dt.mean(f.norm_time),
                                               'norm_time_std': dt.sd(f.norm_time),
                                           }, dt.by(group_by_list)]
        summary_tables['method_mean_over_maps'] = summary_tables['map_method'][:, {
                                                                                      'cost_min': dt.mean(f.cost_min),
                                                                                      'cost_max': dt.mean(f.cost_max),
                                                                                      'cost_median': dt.mean(f.cost_median),
                                                                                      'cost_mean': dt.mean(f.cost_mean),
                                                                                      'cost_std': dt.mean(f.cost_std),
                                                                                      'time_min': dt.mean(f.time_min),
                                                                                      'time_median': dt.mean(f.time_median),
                                                                                      'time_max': dt.mean(f.time_max),
                                                                                      'time_mean': dt.mean(f.time_mean),
                                                                                      'time_std': dt.mean(f.time_std),
                                                                                      'gap_min': dt.mean(f.gap_min),
                                                                                      'gap_median': dt.mean(f.gap_median),
                                                                                      'gap_max': dt.mean(f.gap_max),
                                                                                      'gap_mean': dt.mean(f.gap_mean),
                                                                                      'gap_std': dt.mean(f.gap_std),
                                                                                      'norm_time_min': dt.mean(f.norm_time_min),
                                                                                      'norm_time_median': dt.mean(f.norm_time_median),
                                                                                      'norm_time_max': dt.mean(f.norm_time_max),
                                                                                      'norm_time_mean': dt.mean(f.norm_time_mean),
                                                                                      'norm_time_std': dt.mean(f.norm_time_std),
                                                                                  }, dt.by(group_by['method'])]
        tab_summaries[d] = summary_tables
    return tab_summaries, vis_radii, full_tables


def create_color_palette(
        tab: dt.Frame,
):
    palette_dict = {}
    palette_names_dict = {
        'cov': 'autumn',
        'ccdt': 'summer',
        'ka': 'winter',
        'reflex': 'spring',
    }
    methods_ordered = []
    for method_type, palette_name in palette_names_dict.items():
        methods_of_type = dt.unique(dt.unique(tab[method_type == f['method_type'], 'method'])).to_list()[0]
        if method_type == 'cov':
            methods_of_type.sort(reverse=True)
        secondary_palette = sns.color_palette(palette_name, n_colors=len(methods_of_type))
        for j, method in enumerate(methods_of_type):
            palette_dict[method] = secondary_palette[j]
        methods_ordered.extend(methods_of_type)
    return palette_dict, methods_ordered


def get_graph_limits():
    norm_time_lin = (0.0, 100.0)
    gap_lin = (0.0, 30.0)
    norm_time_log = (1e-1, 100.0)
    gap_log = (1e-1, 1e4)
    return norm_time_lin, gap_lin, norm_time_log, gap_log


def convex_hull(
        tab_d_summary: dt.Frame,
        x_label,
        y_label,
        x_lim_lin,
        y_lim_lin,
        x_lim_log,
        y_lim_log,
):
    tab_dict = tab_d_summary.to_dict()
    points = np.array([tab_dict[x_label], tab_dict[y_label]]).T
    border_points = np.array([
        [min(tab_dict[x_label]), max(max(tab_dict[y_label]), y_lim_lin[1], y_lim_log[1]) + 10.0],  # left-top
        [max(max(tab_dict[x_label]), x_lim_lin[1], x_lim_log[1]) + 10.0, max(max(tab_dict[y_label]), y_lim_lin[1], y_lim_log[1]) + 10.0],  # right-top
        [max(max(tab_dict[x_label]), x_lim_lin[1], x_lim_log[1]) + 10.0, min(tab_dict[y_label])],  # right-bottom
    ])
    points_hull = np.concatenate((points, border_points))
    hull = sp.spatial.ConvexHull(points_hull)
    hull_vertices = hull.vertices.tolist()
    hull_vertices.append(hull_vertices[0])
    hull_vertices_no_border = np.array([i for i in hull_vertices if i < len(points)])
    return points_hull, hull_vertices, hull_vertices_no_border


def subplot(
        tab,
        ax,
        methods_ordered,
        x_label,
        y_label,
        x_lim,
        y_lim,
        palette=None,
        conv_hull=None,
        print_dual_samp=False,
):
    sns.scatterplot(
        tab.to_pandas(),
        x=x_label,
        y=y_label,
        hue='method',
        hue_order=methods_ordered,
        style='filtered_bucketed',
        zorder=3,
        ax=ax,
        palette=palette,
        edgecolor='black',
        s=75,
    )
    for where in ['top', 'right', 'bottom', 'left']:
        ax.spines[where].set_visible(True)
    if conv_hull is not None:
        points_hull, hull_vertices, hull_vertices_no_border = conv_hull
        ax.plot(
            points_hull[hull_vertices_no_border, 0],
            points_hull[hull_vertices_no_border, 1],
            color='none',
            zorder=2.5,
            marker='o',
            markersize=14.0,
            markerfacecolor='none',
            markeredgewidth=1.0,
            markeredgecolor='deeppink',
        )
    if x_lim is not None:
        ax.set_xlim(x_lim)
    if y_lim is not None:
        ax.set_ylim(y_lim)
    if print_dual_samp:
        for i in range(tab.nrows):
            row = tab[i, :]
            method = row['method'].to_list()[0][0]
            if not (method == 'cov-dual-n' or method == 'cov-dual-d'):
                continue
            x = row[x_label].to_list()[0][0]
            y = row[y_label].to_list()[0][0]
            if x < x_lim[0] or x > x_lim[1] or y < y_lim[0] or y > y_lim[1]:
                continue
            dual_samp = row['dual_samp'].to_list()[0][0]
            if method == 'cov-dual-d':
                dual_samp = dual_samp * 100.0
            dual_samp = int(dual_samp)
            ax.text(x, y, f'{dual_samp}', fontsize=3.5, fontweight='bold', ha='center', va='center', color='black', zorder=5)


def plot(
        d: float,
        tab_d_summary: dt.Frame,
        x_label,
        y_label,
        palette_dict,
        methods_ordered,
        legend_labels=None,
        fig_axes=None,
        i=None,
):
    sns.set_theme(style='ticks')
    sns.set_context('paper')
    x_min_log = dt.min(tab_d_summary[x_label]).to_list()[0][0]
    x_max_log = dt.max(tab_d_summary[x_label]).to_list()[0][0]
    y_min_log = dt.min(tab_d_summary[y_label]).to_list()[0][0]
    y_max_log = dt.max(tab_d_summary[y_label]).to_list()[0][0]
    x_lim_log = (10 ** np.floor(np.log10(x_min_log)), 10 ** np.ceil(np.log10(x_max_log)))
    y_lim_log = (10 ** np.floor(np.log10(y_min_log)), 10 ** np.ceil(np.log10(y_max_log)))
    tab_scaled_down = dt.Frame()
    tab_scaled_down = dt.rbind(tab_scaled_down, tab_d_summary[f.filtered, :])
    tab_scaled_down = dt.rbind(tab_scaled_down, tab_d_summary[f.method == 'cov-dual-n', :])
    x_min_lin = dt.min(tab_scaled_down[x_label]).to_list()[0][0]
    x_max_lin = dt.max(tab_scaled_down[x_label]).to_list()[0][0]
    x_diff_lin = x_max_lin - x_min_lin
    y_min_lin = dt.min(tab_scaled_down[y_label]).to_list()[0][0]
    y_max_lin = dt.max(tab_scaled_down[y_label]).to_list()[0][0]
    y_diff_lin = y_max_lin - y_min_lin
    x_lim_lin = (np.floor(x_min_lin - 0.05 * x_diff_lin), np.ceil(x_max_lin + 0.05 * x_diff_lin))
    y_lim_lin = (np.floor(y_min_lin - 0.05 * y_diff_lin), np.ceil(y_max_lin + 0.05 * y_diff_lin))
    ch = convex_hull(tab_d_summary, x_label, y_label, x_lim_lin, y_lim_lin, x_lim_log, y_lim_log)
    if fig_axes is None:
        fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(11, 4))
    else:
        fig, (ax1, ax2) = fig_axes
    rect_color = plt.Rectangle((x_lim_lin[0], y_lim_lin[0]), x_lim_lin[1] - x_lim_lin[0], y_lim_lin[1] - y_lim_lin[0], facecolor='deepskyblue', alpha=0.1, zorder=2)
    rect_bound = plt.Rectangle((x_lim_lin[0], y_lim_lin[0]), x_lim_lin[1] - x_lim_lin[0], y_lim_lin[1] - y_lim_lin[0], edgecolor='deepskyblue', facecolor='none', zorder=2)
    subplot(tab_d_summary, ax1, methods_ordered, x_label, y_label, x_lim=x_lim_log, y_lim=y_lim_log, palette=palette_dict, conv_hull=ch)
    grid_props = dict(which='major', axis='both', color='#262626', linestyle='--', linewidth=0.5, alpha=0.2)
    grid_props_minor = dict(which='minor', axis='both', color='#262626', linestyle='--', linewidth=0.25, alpha=0.1)
    ax1.grid(True, **grid_props)
    ax1.grid(True, **grid_props_minor)
    ax2.grid(True, **grid_props)
    ax2.grid(True, **grid_props_minor)
    ax1.set(xscale='log', yscale='log')
    ax1.set_xticks([0.01, 0.1, 1, 10, 100, 1000], labels=['0.01', '0.1', '1', '10', '100', '1,000'])
    ax1.set_yticks([100, 1000, 10000, 100000], labels=['100', '1,000', '10,000', '100,000'])
    ax1.set_xlim(x_lim_log)
    ax1.set_ylim(y_lim_log)
    subplot(tab_d_summary, ax2, methods_ordered, x_label, y_label, x_lim=x_lim_lin, y_lim=y_lim_lin, palette=palette_dict, conv_hull=ch)
    ax1.add_patch(rect_color)
    ax1.add_patch(rect_bound)
    ax1.legend().remove()
    ax1.set_xlabel('Mean Runtime [s]')
    ax1.set_ylabel('Mean Number of Guards [1]', labelpad=-2)
    ax2.set_xlabel('Mean Runtime [s]')
    ax2.set_ylabel('')
    d_str = f'{"Unlimited Visibility" if d == float("inf") else f"Limited Visibility Radius: {int(d)}m"}'
    if i is not None:
        d_str = f'({chr(97 + i)}) {d_str}'
    ax1.set_title(d_str, fontweight='bold', loc='left')
    ax1.set_title('i. Log Scale', loc='right')
    # ax2.set_title(d_str, fontweight='bold', loc='left')
    ax2.set_title('ii. Linear Scale, Zoomed In', loc='right')
    if legend_labels is not None:
        handles, _ = ax2.get_legend_handles_labels()
        if d != float('inf') and 'RV' in legend_labels:
            # Remove RV from the legend.
            rv_id = legend_labels.index('RV')
            handles = handles[:rv_id] + handles[rv_id + 1:]
            legend_labels = legend_labels[:rv_id] + legend_labels[rv_id + 1:]
        ax2.legend(
            handles=handles,
            labels=legend_labels,
            bbox_to_anchor=(1.0, 1.125),
            loc='upper left',
            frameon=False,
        )
    if fig_axes is None:
        fig.suptitle(
            f'{"Unlimited Visibility" if d == float("inf") else f"Limited Visibility Radius: {int(d)} m"}',
            y=0.99,
            fontweight='bold',
        )
        # fig.tight_layout()
        fig.tight_layout(rect=[-0.01, -0.04, 1.018, 1.04])


def summary_plot(
        save_dir: str,
        full_tables: dict[float, dt.Frame],
        methods: list,
):
    # Create a color palette.
    palette = {
        'ka': sns.color_palette('winter', n_colors=4)[0],
        'cov-inf': sns.color_palette('autumn', n_colors=4)[0],
        'cov-dual-n16': sns.color_palette('autumn', n_colors=4)[1],
        'cov-dual-n64': sns.color_palette('autumn', n_colors=4)[2],
        'cov-dual-n256': sns.color_palette('autumn', n_colors=4)[3],
        'ka+reflex-f': sns.color_palette('summer', n_colors=4)[0],
        'ka+reflex-f-b': sns.color_palette('summer', n_colors=4)[2],
        'ka+ccdt+reflex-f': sns.color_palette('cool', n_colors=4)[3],
        'ka+ccdt+reflex-f-b': sns.color_palette('cool', n_colors=4)[2],
    }
    for method in methods:
        assert method in palette, f'Method {method} does not have a color assigned.'
    # Create the summary data table.
    tab = dt.Frame()
    for d, tab_d in full_tables.items():
        if d == float('inf'):
            tab_d['vis_radius'] = 256
        else:
            tab_d['vis_radius'] = int(d)
        tab = dt.rbind([tab, tab_d])
    # Create a subset of methods without bucketing.
    methods_no_buck = cp.deepcopy(methods)
    for method in methods_no_buck:
        if method.endswith('-b'):
            methods_no_buck.remove(method)
    # Set common properties.
    sns.set_theme(style='ticks')
    sns.set_context('paper')
    x_label = 'vis_radius'
    hue_label = 'method_full'
    grid_props = dict(which='major', axis='both', color='#262626', linestyle='--', linewidth=0.5, alpha=0.2)
    grid_props_minor = dict(which='minor', axis='both', color='#262626', linestyle='--', linewidth=0.25, alpha=0.1)
    common_boxplot_props = dict(x=x_label, hue=hue_label, palette=palette, native_scale=True, width=0.8)
    common_line_plot_props = dict(x=x_label, hue=hue_label, palette=palette, estimator='mean', errorbar=None)
    x_ticks = [4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 256]
    x_tick_labels = ['4', '6', '8', '12', '16', '24', '32', '48', '64', '96', '128', '∞']
    x_axis_label = 'Limited Visibility Radius [m] in Log Scale + ∞ for Unlimited Visibility'
    x_lim = (3.5, 295)
    # Create the figure and axes.
    fig, (ax1, ax2) = plt.subplots(nrows=2, figsize=(12, 8))
    # Plot solution quality.
    ax = ax1
    y_label = 'gap'
    lin_thresh = 20.0
    ax.set_title(f'(a) Solution Quality Comparison of Selected Methods', fontweight='bold', loc='left')
    ax.set_title(f'i. Linear-Log Scale with Threshold at {lin_thresh:.0f}% on Vertical Axis', loc='right')
    ax.axhline(lin_thresh, color='#262626', linewidth=1.0)
    ax.fill_between([1, 400], lin_thresh, 1e6, color='azure', zorder=0, facecolor='deepskyblue', alpha=0.1)
    ax.grid(True, **grid_props)
    ax.grid(True, **grid_props_minor)
    tab_no_buck = tab[~f.bucketed, :]
    p_tab_no_buck = tab_no_buck.to_pandas()
    tab_no_buck_mean = tab_no_buck[:, {y_label: dt.mean(f[y_label])}, dt.by(hue_label, x_label)]
    for method in methods_no_buck:
        line = tab_no_buck_mean[f[hue_label] == method, y_label].to_list()[0]
        for i in range(1, len(line)):
            value_prev, value = line[i - 1], line[i]
            if (value_prev < lin_thresh < value) or (value_prev > lin_thresh > value):
                x_prev, x = tab_no_buck_mean[f[hue_label] == method, x_label].to_list()[0][i - 1], tab_no_buck_mean[f[hue_label] == method, x_label].to_list()[0][i]
                y_prev, y = value_prev, value
                x_cross = x_prev + (lin_thresh - y_prev) * (x - x_prev) / (y - y_prev)
                tab_no_buck_mean = dt.rbind([tab_no_buck_mean, dt.Frame([{'method_full': method, 'vis_radius': x_cross, y_label: lin_thresh}])])
    p_tab_no_buck_mean = tab_no_buck_mean.to_pandas()
    sns.lineplot(p_tab_no_buck_mean, ax=ax, y=y_label, **common_line_plot_props, hue_order=methods_no_buck, legend=False, linewidth=2)
    sns.boxplot(p_tab_no_buck, ax=ax, y=y_label, hue_order=methods_no_buck, **common_boxplot_props, linewidth=0.0,
                showfliers=True, fill=False, flierprops={'marker': 'o', 'markersize': 2.0}, legend=False, log_scale=(True, False))
    sns.boxplot(p_tab_no_buck, ax=ax, y=y_label, hue_order=methods_no_buck, **common_boxplot_props, linewidth=1.0, linecolor='#262626', showfliers=False, log_scale=(True, False))
    handles, _ = ax.get_legend_handles_labels()
    ax.legend(labels=['KA', 'IRS', 'DSk-16', 'DSk-64', 'DSk-256', 'HFH(B)-KA,RV', 'HFH(B)-KA,CCDT,RV'],
              handles=handles, bbox_to_anchor=(1.0, 1.0), loc='upper left', frameon=False)
    ax.set_xticks(x_ticks, labels=x_tick_labels)
    ax.set_yscale('symlog', linthresh=lin_thresh, linscale=3.0)
    ax.set_yticks([2e1, 3e1, 4e1, 5e1, 6e1, 7e1, 8e1, 9e1, 1e2, 2e2, 3e2, 4e2, 5e2, 6e2, 7e2, 8e2, 9e2, 1e3, 2e3, 3e3, 4e3, 5e3, 6e3, 7e3, 8e3, 9e3, 1e4], minor=True)
    ax.set_yticks([0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 50, 100, 500, 1000],
                  labels=['0', '2', '4', '6', '8', '10', '12', '14', '16', '18', '20', '50', '100', '500', '1,000'], fontsize=7)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_ylim(-1, 1300)
    ax.set_xticks([], minor=True)
    ax.set_xlabel(x_axis_label)
    ax.set_ylabel('Best-Known Solution Gap [%]', labelpad=-2)
    # Plot runtime.
    ax = ax2
    y_label = 'time'
    lin_thresh = 20.0
    ax.set_title(f'(b) Runtime Comparison of Selected Methods', fontweight='bold', loc='left')
    ax.set_title(f'i. Linear-Log Scale with Threshold at {lin_thresh:.0f}s on Vertical Axis', loc='right')
    ax.axhline(lin_thresh, color='#262626', linewidth=1.0)
    ax.fill_between([1, 400], lin_thresh, 1e6, color='azure', zorder=0, facecolor='deepskyblue', alpha=0.1)
    ax.grid(True, **grid_props)
    ax.grid(True, **grid_props_minor)
    tab_mean = tab[:, {y_label: dt.mean(f[y_label])}, dt.by(hue_label, x_label)]
    for method in methods:
        line = tab_mean[f[hue_label] == method, y_label].to_list()[0]
        for i in range(1, len(line)):
            value_prev, value = line[i - 1], line[i]
            if (value_prev < lin_thresh < value) or (value_prev > lin_thresh > value):
                x_prev, x = tab_mean[f[hue_label] == method, x_label].to_list()[0][i - 1], tab_mean[f[hue_label] == method, x_label].to_list()[0][i]
                y_prev, y = value_prev, value
                x_cross = x_prev + (lin_thresh - y_prev) * (x - x_prev) / (y - y_prev)
                tab_mean = dt.rbind([tab_mean, dt.Frame([{'method_full': method, 'vis_radius': x_cross, y_label: lin_thresh}])])
    p_tab_mean = tab_mean.to_pandas()
    sns.lineplot(p_tab_mean, ax=ax, y=y_label, **common_line_plot_props, hue_order=methods, linewidth=2)
    handles, _ = ax.get_legend_handles_labels()
    ax.legend(labels=['KA', 'IRS', 'DSk-16', 'DSk-64', 'DSk-256', 'HFH-KA,RV', 'HFHB-KA,RV', 'HFH-KA,CCDT,RV', 'HFHB-KA,CCDT,RV'],
              handles=handles, bbox_to_anchor=(1.0, 1.0), loc='upper left', frameon=False)
    ax.set(xscale='log')
    ax.set_xticks(x_ticks, labels=x_tick_labels)
    ax.set_yscale('symlog', linthresh=lin_thresh, linscale=3.0)
    ax.set_yticks([2e1, 3e1, 4e1, 5e1, 6e1, 7e1, 8e1, 9e1, 1e2, 2e2, 3e2, 4e2, 5e2, 6e2, 7e2, 8e2, 9e2, 1e3, 2e3, 3e3, 4e3, 5e3, 6e3, 7e3, 8e3, 9e3, 1e4], minor=True)
    ax.set_yticks([0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 50, 100, 500, 1000],
                  labels=['0', '2', '4', '6', '8', '10', '12', '14', '16', '18', '20', '50', '100', '500', '1,000'], fontsize=7)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_ylim(-1, 130)
    ax.set_xticks([], minor=True)
    ax.set_xlabel(x_axis_label)
    ax.set_ylabel('Mean Runtime [s]', labelpad=0)
    # #
    # ax = ax3
    # ax.set_title(f'ii. Log Scale', loc='right')
    # sns.boxplot(tab.to_pandas(), ax=ax, y=y_label, hue_order=methods, **common_boxplot_props, linewidth=0.0,
    #             showfliers=True, fill=False, flierprops={'marker': 'o', 'markersize': 2.0}, legend=False, log_scale=True)
    # sns.boxplot(tab.to_pandas(), ax=ax, y=y_label, hue_order=methods, **common_boxplot_props, linewidth=1.0, linecolor='#262626', showfliers=False, log_scale=True)
    # ax.grid(True, **grid_props)
    # ax.grid(True, **grid_props_minor)
    # handles, _ = ax.get_legend_handles_labels()
    # ax.legend(labels=['KA', 'IRS', 'DSk-16', 'DSk-64', 'DSk-256', 'HFH-KA,RV', 'HFHB-KA,RV', 'HFH-KA,CCDT,RV', 'HFHB-KA,CCDT,RV'],
    #           handles=handles, bbox_to_anchor=(1.0, 1.0), loc='upper left', frameon=False)
    # ax.set_xticks(x_ticks, labels=x_tick_labels)
    # ax.set_xlim(x_lim[0], x_lim[1])
    # ax.set_xticks([], minor=True)
    # ax.set_xlabel(x_axis_label)
    # ax.set_yticks([1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4], labels=['0.01', '0.1', '1', '10', '100', '1,000', '10,000'], fontsize=7)
    # ax.set_ylim(1e-2, 1e4)
    # ax.set_ylabel('Runtime [s]', labelpad=-4)

    # Save the figure.
    fig.tight_layout(pad=0.1, h_pad=1.0, w_pad=0.0, rect=[0.0, 0.0, 1.005, 1.0])
    # fig.tight_layout(rect=[-0.01, -0.01, 1.018, 1.01])
    plt.savefig(f'{save_dir}/final-results.pdf', dpi=300)


def process_val(
        tab: dt.Frame,
):
    tab = tab[~((f.method == 'cov-dual-n') & (f.dual_samp == 512)), :]
    tab_summaries, vis_radii, _ = compute_tab_summaries_per_vis_radius(tab)
    palette, methods = create_color_palette(tab)
    labels = [
        '',
        'RS',
        'IRS',
        'DSk-{2,4,...,256}',
        'DSρ-{2,4,...,256}×.01',
        'CCDT',
        'CCDT,RV',
        'KA',
        'KA,CCDT',
        'KA,CCDT,RV',
        'KA,RV',
        'RV',
        '',
        'Baseline M',
        'HFH-M',
        'HFHB-M',
    ]
    save_dir = f'figs/val'
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(nrows=len(vis_radii), ncols=2, figsize=(12, 15))
    for i, d in enumerate(vis_radii):
        tab_d_method = tab_summaries[d]['method_mean_over_maps']
        plot(d, tab_d_method, x_label='time_mean', y_label='cost_mean', palette_dict=palette, methods_ordered=methods, legend_labels=labels, fig_axes=(fig, axes[i]), i=i)
    fig.tight_layout(pad=0.1, h_pad=-2.5, w_pad=-0.5, rect=[0.0, -0.02, 1.005, 1.0])
    # fig.tight_layout(rect=[-0.01, -0.008, 1.015, 1.005])
    plt.savefig(f'{save_dir}/preliminary-results.pdf', dpi=300)


def set_last(name, names):
    if name in names:
        names.remove(name)
        names = names + [name]
    return names


def plot_speedup(
        save_dir,
        tab: dt.Frame,
):
    # Create a new fig.
    fig = plt.figure(figsize=(6, 3))
    ax = plt.gca()
    ax.set_xscale('log')
    sns.set_theme(style='ticks')
    sns.set_context('paper')
    palette = {
        'ka+reflex': sns.color_palette('summer', n_colors=4)[2],
        'ka+ccdt+reflex': sns.color_palette('cool', n_colors=4)[2],
    }
    x_label = 'd'
    y_label = 'speedup'
    hue_label = 'method'
    grid_props = dict(which='major', axis='both', color='#262626', linestyle='--', linewidth=0.5, alpha=0.2)
    grid_props_minor = dict(which='minor', axis='both', color='#262626', linestyle='--', linewidth=0.25, alpha=0.1)
    tab[f.d == float('inf'), 'd'] = 256
    sns.lineplot(tab.to_pandas(), x=x_label, y=y_label, hue=hue_label, palette=palette, linewidth=2)
    y_ticks = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    y_tick_labels = ['0', '10', '20', '30', '40', '50', '60', '70', '80', '90', '100']
    x_ticks = [4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 256]
    x_tick_labels = ['4', '6', '8', '12', '16', '24', '32', '48', '64', '96', '128', '∞']
    x_axis_label = 'Limited Visibility Radius [m] in Log Scale + ∞ for Unlimited Visibility'
    x_lim = (3.5, 295)
    ax.grid(True, **grid_props)
    ax.grid(True, **grid_props_minor)
    ax.set_yticks(y_ticks, labels=y_tick_labels)
    ax.set_xticks(x_ticks, labels=x_tick_labels)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_xticks([], minor=True)
    ax.set_xlabel(x_axis_label)
    ax.set_ylim(0, 100)
    ax.set_ylabel('Average Runtime Reduction [%]')
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles=handles, labels=['HFHB-KA,RV', 'HFHB-KA,CCDT,RV'], framealpha=1.0, fancybox=False, edgecolor='black')
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 1.0])
    plt.savefig(f'{save_dir}/speedup.pdf', dpi=300)


def process_test(
        tab: dt.Frame,
):
    tab_methods = dt.unique(tab[:, ['method_full']]).to_list()[0]
    tab_methods = set_last('ka', tab_methods)
    tab_methods = set_last('cov-inf', tab_methods)
    tab_methods = set_last('cov-dual-n16', tab_methods)
    tab_methods = set_last('cov-dual-n64', tab_methods)
    tab_methods = set_last('cov-dual-n256', tab_methods)
    tab_methods = set_last('ka+reflex-f', tab_methods)
    tab_methods = set_last('ka+reflex-f-b', tab_methods)
    tab_methods = set_last('ka+ccdt+reflex-f', tab_methods)
    tab_methods = set_last('ka+ccdt+reflex-f-b', tab_methods)
    tab_summaries, vis_radii, full_tables = compute_tab_summaries_per_vis_radius(tab)
    palette, methods = create_color_palette(tab)
    labels = [
        '',
        'IRS',
        'DSk-{16,64,256}',
        'KA',
        'KA,CCDT,RV',
        'KA,RV',
        '',
        'Baseline M',
        'HFH-M',
        'HFHB-M',
    ]
    save_dir = f'figs/test'
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    tab_speedup = dt.Frame()
    for i, d in enumerate(vis_radii):
        tab_d_method = tab_summaries[d]['method_mean_over_maps']
        plot(d, tab_d_method, x_label='time_mean', y_label='cost_mean', palette_dict=palette, methods_ordered=methods, legend_labels=labels)
        plt.savefig(f'{save_dir}/test{i + 1:02}-d{f"{int(d):03}" if d != float("inf") else "inf"}.pdf', dpi=300)
        time_hfhb_kr = tab_d_method[(f.method == 'ka+reflex') & f.bucketed, 'time_mean'].to_list()[0][0]
        time_hfhb_kcr = tab_d_method[(f.method == 'ka+ccdt+reflex') & f.bucketed, 'time_mean'].to_list()[0][0]
        gap_hfhb_kr = tab_d_method[(f.method == 'ka+reflex') & f.bucketed, 'gap_mean'].to_list()[0][0]
        gap_hfhb_kcr = tab_d_method[(f.method == 'ka+ccdt+reflex') & f.bucketed, 'gap_mean'].to_list()[0][0]
        time_increase = 100 * (time_hfhb_kcr - time_hfhb_kr) / time_hfhb_kr
        gap_decrease = gap_hfhb_kr - gap_hfhb_kcr
        for m in ['ka+reflex', 'ka+ccdt+reflex']:
            time_hfh = tab_d_method[(f.method == m) & ~f.bucketed, 'time_mean'].to_list()[0][0]
            time_hfhb = tab_d_method[(f.method == m) & f.bucketed, 'time_mean'].to_list()[0][0]
            speedup = 100 * time_hfhb / time_hfh
            tab_speedup = dt.rbind([tab_speedup, dt.Frame([{'d': d, 'method': m, 'time_hfh': time_hfh, 'time_hfhb': time_hfhb, 'speedup': speedup}])])
    summary_plot(save_dir, full_tables, tab_methods)
    plot_speedup(save_dir, tab_speedup)


def process_extra(
        tab: dt.Frame,
):
    tab[f.vis_radius == float('inf'), 'vis_radius'] = 256
    sns.set_theme(style='ticks')
    sns.set_context('paper')
    save_dir = f'figs/extra'
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=4, figsize=(12, 12))
    tab[:, 'process_coverage.covered_ratio'] = 100.0 * f['process_coverage.covered_ratio']
    color = sns.color_palette('summer', n_colors=4)[2]
    color_darker = mcolors.to_hex((0.4 * mcolors.to_rgb(color)[0], 0.4 * mcolors.to_rgb(color)[1], 0.4 * mcolors.to_rgb(color)[2]))
    palette = sns.blend_palette([color, color_darker], n_colors=6)
    x_label = 'vis_radius'
    y_label = 'process_coverage.covered_ratio'
    hue_label = 'robustness_radius'
    grid_props = dict(which='major', axis='both', color='#262626', linestyle='--', linewidth=0.5, alpha=0.2)
    grid_props_minor = dict(which='minor', axis='both', color='#262626', linestyle='--', linewidth=0.25, alpha=0.1)
    common_boxplot_props = dict(x=x_label, hue=hue_label, palette=palette, native_scale=True, width=0.8)
    x_ticks = [4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 256]
    x_tick_labels = ['4', '6', '8', '12', '16', '24', '32', '48', '64', '96', '128', '∞']
    x_axis_label = 'Limited Visibility Radius [m] in Log Scale + ∞ for Unlimited Visibility'
    x_lim = (3.5, 295)
    #
    ax = ax1
    ax.set_title('(a) Covered Ratio of the HFHB-KA,RV Method for Different Visibility Models', fontweight='bold', loc='left')
    ax.set_title('i. Linear Scale from 99.2% to 100% on Vertical Axis', loc='right')
    coverage_goal = ax.axhline(99.9, color='red', linestyle='--')
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, fill=False, showfliers=True, flierprops={'marker': 'o', 'markersize': 2.0}, legend=False, log_scale=True)
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, showfliers=False, log_scale=True)
    ax.grid(True, **grid_props)
    ax.set_ylabel('Covered Ratio [%]')
    ax.set_yticks([99.0, 99.1, 99.2, 99.3, 99.4, 99.5, 99.6, 99.7, 99.8, 99.9, 100.0],
                  labels=['99', '99.1', '99.2', '99.3', '99.4', '99.5', '99.6', '99.7', '99.8', '99.9', '100'])
    ax.set_ylim(99.2, 100)
    ax.set_xticks(x_ticks, labels=x_tick_labels)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_xticks([], minor=True)
    ax.set_xlabel(x_axis_label)
    handles, _ = ax.get_legend_handles_labels()
    empty_line1 = plt.Line2D([0], [0], color='none')
    empty_line2 = plt.Line2D([0], [0], color='none')
    handles = [empty_line1] + handles + [empty_line2, coverage_goal]
    uncertainty_label = 'Uncertainty:'
    legend = ax.legend(labels=[uncertainty_label, 'None', '10cm', '20cm', '40cm', '80cm', '1.6m', '', '99.9% Goal'],
                       handles=handles, bbox_to_anchor=(1.0, 1.0), loc='upper left', frameon=False)
    for text in legend.get_texts():
        if text.get_text() == uncertainty_label:
            text.set_position((-26, 0))
    #
    ax = ax2
    ax.set_title('ii. Zoomed in Between 99.895% and 99.915%', loc='right')
    coverage_goal = ax.axhline(99.9, color='red', linestyle='--')
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, fill=False, showfliers=True, flierprops={'marker': 'o', 'markersize': 2.0}, legend=False,
                log_scale=(True, False))
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, showfliers=False, log_scale=(True, False))
    ax.grid(True, **grid_props)
    ax.set_ylabel('Covered Ratio [%]')
    ax.set_yticks([99.895, 99.9, 99.905, 99.91, 99.915],
                  labels=['99.895', '99.9', '99.905', '99.91', '99.915'])
    ax.set_ylim(99.895, 99.915)
    ax.set_xticks(x_ticks, labels=x_tick_labels)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_xticks([], minor=True)
    ax.set_xlabel(x_axis_label)
    handles, _ = ax.get_legend_handles_labels()
    empty_line1 = plt.Line2D([0], [0], color='none')
    empty_line2 = plt.Line2D([0], [0], color='none')
    handles = [empty_line1] + handles + [empty_line2, coverage_goal]
    uncertainty_label = 'Uncertainty:'
    legend = ax.legend(labels=[uncertainty_label, 'None', '10cm', '20cm', '40cm', '80cm', '1.6m', '', '99.9% Goal'],
                       handles=handles, bbox_to_anchor=(1.0, 1.0), loc='upper left', frameon=False)
    for text in legend.get_texts():
        if text.get_text() == uncertainty_label:
            text.set_position((-26, 0))
    #
    ax = ax3
    y_label = 'agp_n'
    ax.set_title('(b) Number of Guards of the HFHB-KA,RV Method for Different Visibility Models', fontweight='bold', loc='left')
    ax.set_title('i. Log Scale on Vertical Axis', loc='right')
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, fill=False, showfliers=True, flierprops={'marker': 'o', 'markersize': 2.0}, legend=False, log_scale=True)
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, showfliers=False, log_scale=True)
    ax.grid(True, **grid_props)
    ax.grid(True, **grid_props_minor)
    ax.set_ylabel('Number of Guards [1]', labelpad=-2)
    ax.set_yticks([100, 1000, 10000, 100000], labels=['100', '1,000', '10,000', '100,000'])
    ax.set_ylim(50, 100000)
    ax.set_xticks(x_ticks, labels=x_tick_labels)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_xticks([], minor=True)
    ax.set_xlabel(x_axis_label)
    handles, _ = ax.get_legend_handles_labels()
    empty_line1 = plt.Line2D([0], [0], color='none')
    handles = [empty_line1] + handles
    uncertainty_label = 'Uncertainty:'
    legend = ax.legend(labels=[uncertainty_label, 'None', '10cm', '20cm', '40cm', '80cm', '1.6m'],
                       handles=handles, bbox_to_anchor=(1.0, 1.0), loc='upper left', frameon=False)
    for text in legend.get_texts():
        if text.get_text() == uncertainty_label:
            text.set_position((-26, 0))
    #
    ax = ax4
    y_label = 'agp_time'
    ax.set_title('(c) Computational Time of the HFHB-KA,RV Method for Different Visibility Models', fontweight='bold', loc='left')
    ax.set_title('i. Log Scale on Vertical Axis', loc='right')
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, fill=False, showfliers=True, flierprops={'marker': 'o', 'markersize': 2.0}, legend=False, log_scale=True)
    sns.boxplot(tab.to_pandas(), y=y_label, **common_boxplot_props, ax=ax, showfliers=False, log_scale=True)
    ax.grid(True, **grid_props)
    ax.grid(True, **grid_props_minor)
    ax.set_ylabel('Computational Time [s]', labelpad=-1)
    ax.set_yticks([0.1, 1, 10, 100, 1000, 10000], labels=['0.1', '1', '10', '100', '1,000', '10,000'])
    ax.set_ylim(0.1, 30000)
    ax.set_xticks(x_ticks, labels=x_tick_labels)
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_xticks([], minor=True)
    ax.set_xlabel(x_axis_label)
    handles, _ = ax.get_legend_handles_labels()
    empty_line1 = plt.Line2D([0], [0], color='none')
    handles = [empty_line1] + handles
    uncertainty_label = 'Uncertainty:'
    legend = ax.legend(labels=[uncertainty_label, 'None', '10cm', '20cm', '40cm', '80cm', '1.6m'],
                       handles=handles, bbox_to_anchor=(1.0, 1.0), loc='upper left', frameon=False)
    for text in legend.get_texts():
        if text.get_text() == uncertainty_label:
            text.set_position((-26, 0))
    fig.tight_layout(pad=0.8, h_pad=1.0, w_pad=0, rect=[-0.008, -0.008, 1.015, 1.008])
    plt.savefig('figs/extra/additional-results.pdf', dpi=300)


def main(args):
    tab = dt.fread(args.input_csv_file)
    if args.context == 'val':
        process_val(tab)
    elif args.context == 'test':
        process_test(tab)
    elif args.context == 'extra':
        process_extra(tab)
    else:
        raise Exception(f'Unknown context: {args.context}.')


if __name__ == '__main__':
    main(args_parser().parse_args())
