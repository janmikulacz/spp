#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import argparse
import random

args_parser = argparse.ArgumentParser()
args_parser.add_argument('--exec', type=str, default='../../build-Release/visis',
                         help='Path to the evaluate_mesh executable.')
args_parser.add_argument('--method', type=str, required=True,
                         help='Method name and arguments.')
args_parser.add_argument('--iter', type=int, default=None,
                         help='The iteration id (for stochastic methods).')
args_parser.add_argument('--run_type', type=str, required=True,
                         help='Options: val, test, extra.')
args_parser.add_argument('--vis_radius', type=float, default=math.inf,
                         help='Visibility radius.')
args_parser.add_argument('--robustness_radius', type=float, default=None,
                         help='Robustness radius.')


def main(args, parsed=False):
    if not parsed:
        args = args_parser.parse_args(args.split(' '))
        method = args.method.replace('|', ' ')
    else:
        method = args.method
    run_type = args.run_type
    if run_type == 'extra':
        inst_file_path = f'./maps-test.txt'
    else:
        inst_file_path = f'./maps-{run_type}.txt'
    vis_radius = args.vis_radius
    vis_radius_str = str(int(round(100.0 * vis_radius))) + "cm" if vis_radius != math.inf else str(math.inf)
    robustness_radius_str = str(int(round(100.0 * args.robustness_radius))) + "cm" if args.robustness_radius is not None else 'None'
    iteration = args.iter
    random.seed(iteration)
    coverage_ratio_str = '0.999'
    visis_args = ' --agp'
    visis_args += ' --record_guards_coverage'
    visis_args += f' --coverage {coverage_ratio_str}'
    visis_args += ' --guards_cov_rand_ratio'   f' {coverage_ratio_str}'
    visis_args += ' --guards_cov_inf_ratio'    f' {coverage_ratio_str}'
    visis_args += ' --guards_cov_dual_ratio'   f' {coverage_ratio_str}'
    visis_args += f' --vis_radius {vis_radius}'
    if args.robustness_radius is not None:
        robustness_radius = args.robustness_radius
        visis_args += f' --robustness_radius {robustness_radius}'
    visis_args += f' --guards {method}'
    with open(inst_file_path, 'r') as inst_file:
        for inst in inst_file.readlines():
            inst = inst.strip()
            if inst.startswith('#'):
                continue  # Ignore commented out instances.
            out_dir = f'data/{run_type}/{inst}/radius_{vis_radius_str}'
            if args.robustness_radius is not None:
                out_dir += f'/robustness_{robustness_radius_str}'
            out_file = f'{method.replace(" ", "_").replace("_-", "-").replace("_--", "--")}--it_{iteration:03}.json'
            program_args = f'{visis_args} --map_name {inst} --out_dir {out_dir} --out_file {out_file}'
            output_exists = False
            while not output_exists:
                random_seeds_args = ''
                random_seeds_args += ' --guards_random_seed'      f' {random.randint(0, 2 ** 31 - 1)}'
                random_seeds_args += ' --guards_reflex_rand_seed' f' {random.randint(0, 2 ** 31 - 1)}'
                random_seeds_args += ' --guards_ccdt_rand_seed'   f' {random.randint(0, 2 ** 31 - 1)}'
                random_seeds_args += ' --guards_ka_rand_seed'     f' {random.randint(0, 2 ** 31 - 1)}'
                random_seeds_args += ' --guards_cov_rand_seed'    f' {random.randint(0, 2 ** 31 - 1)}'
                random_seeds_args += ' --guards_cov_inf_seed'     f' {random.randint(0, 2 ** 31 - 1)}'
                random_seeds_args += ' --guards_cov_dual_seed'    f' {random.randint(0, 2 ** 31 - 1)}'
                cmd = f'timeout 21600 {args.exec} {program_args} {random_seeds_args}'  # 6 hours timeout
                print('-' * 80)
                print(f'>>> {cmd}')
                print()
                os.system(f'{cmd}')
                print()
                output_exists = os.path.exists(f'{out_dir}/{out_file}')
                if not output_exists:
                    print('Output does not exist. Retrying with different random seeds.')
                else:
                    print(f'<<< {cmd}')
                    print('-' * 80)
                    print()


if __name__ == '__main__':
    main(args_parser.parse_args(), True)
