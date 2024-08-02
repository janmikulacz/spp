#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse

args_parser = argparse.ArgumentParser()
args_parser.add_argument('--exec', type=str, default='../../build-Release/visis',
                         help='Path to the evaluate_mesh executable.')
args_parser.add_argument('--instances', type=str, required=True,
                         help='Path to the file with the instances names.')
args_parser.add_argument('--subset', type=str, default=None,
                         help='Subset name.')


def main(
        args: argparse.Namespace,
):
    instances = args.instances
    subset = args.subset
    out_dir = f'data'
    if subset is not None:
        out_dir += f'/{subset}'
    with open(instances, 'r') as inst_file:
        for inst in inst_file.readlines():
            inst = inst.strip()
            if inst.startswith('#'):
                continue  # Ignore commented out instances.
            out_file = f'{inst}.json'
            program_args = f'--record_map_properties --terminate record_map_properties --map_name {inst} --out_dir {out_dir} --out_file {out_file}'
            os.system(f'{args.exec} {program_args}')


if __name__ == '__main__':
    main(args_parser.parse_args())
