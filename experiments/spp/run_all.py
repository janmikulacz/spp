#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# python3 run_all.py -c val
# python3 run_all.py -c test
# python3 run_all.py -c extra

import run

import argparse

no_bucketing_args = '--target_bucket_ratio 1.0 --weight_bucket_ratio 1.0'
sample_arg_edges_args = '--sample_arc_edges_angle nan --sample_arc_edges_dist 2.0943951024'
robustness_param = '--robustness_sample_dist 0.15707963268'


def args_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--context', type=str, help='Context: {val,test,extra}')
    parser.add_argument('-d', '--run_deterministic', type=bool, default=True, help='Run deterministic methods.')
    parser.add_argument('-s', '--run_stochastic', type=bool, default=True, help='Run stochastic methods.')
    return parser


n_iter = 10

vis_radii_val = [
    float('inf'),
    64,
    32,
    16,
    8,
]

vis_radii_all = [
    float('inf'),
    128,
    96,
    64,
    48,
    32,
    24,
    16,
    12,
    8,
    6,
    4,
]

robustness_radii = [
    0.0,
    0.1,
    0.2,
    0.4,
    0.8,
    1.6,
]


def go(args: str, method: str):
    run.main(f'{args} --method {method.replace(" ", "|")}')


def main(options):
    general_args = ''
    general_args += f'--run_type {options.context}'
    robustness_radii_used = [None]
    if options.context == 'extra':
        robustness_radii_used = robustness_radii
    for robustness_radius in robustness_radii_used:
        vis_radii_used = vis_radii_val if options.context == 'val' else vis_radii_all
        for vis_radius in vis_radii_used:
            args_radius = f'{general_args} --vis_radius {vis_radius}'
            if robustness_radius is not None:
                args_radius += f' --robustness_radius {robustness_radius}'
            robustness_param_used = robustness_param if robustness_radius is not None else ''
            for i in range(n_iter):
                args = f'{args_radius} --iter {i + 1}'
                if i == 0 and options.run_deterministic:
                    # === Deterministic methods. ===
                    if options.context in ['val'] and vis_radius == float('inf'):
                        go(args, f'reflex {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'reflex {no_bucketing_args} --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'reflex --filter {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'ka {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val']:
                        go(args, f'ka {no_bucketing_args} --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ka --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ka reflex {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'ka reflex {no_bucketing_args} --filter {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test', 'extra']:
                        go(args, f'ka reflex --filter {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val']:
                        go(args, f'ccdt {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ccdt {no_bucketing_args} --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ccdt --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ccdt reflex {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ccdt reflex {no_bucketing_args} --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ccdt reflex --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ka ccdt {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ka ccdt {no_bucketing_args} --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ka ccdt --filter {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'ka ccdt reflex {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'ka ccdt reflex {no_bucketing_args} --filter {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'ka ccdt reflex --filter {sample_arg_edges_args} {robustness_param_used}')
                if options.run_stochastic:
                    # === Stochastic methods. ===
                    if options.context in ['val']:
                        go(args, f'cov-rand {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'cov-inf {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val']:
                        go(args, f'cov-dual --guards_cov_dual_samp   2 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   4 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   8 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'cov-dual --guards_cov_dual_samp  16 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val']:
                        go(args, f'cov-dual --guards_cov_dual_samp  32 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'cov-dual --guards_cov_dual_samp  64 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val']:
                        go(args, f'cov-dual --guards_cov_dual_samp 128 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val', 'test']:
                        go(args, f'cov-dual --guards_cov_dual_samp 256 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                    if options.context in ['val']:
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 0.02 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 0.04 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 0.08 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 0.16 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 0.32 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 0.64 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 1.28 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')
                        go(args, f'cov-dual --guards_cov_dual_samp   0 --guards_cov_dual_samp_sq 2.56 {no_bucketing_args} {sample_arg_edges_args} {robustness_param_used}')


if __name__ == '__main__':
    main(args_parser().parse_args())
