#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# python3 process1.py -i data/val -o data/val.csv
# python3 process1.py -i data/test -o data/test.csv

import json
import argparse

from pathlib import Path

from termcolor import colored

import datatable as dt
from datatable import f

from json_normalize import json_normalize


def args_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_data_dir', type=str, help='Directory with experimental data.')
    parser.add_argument('-o', '--output_csv_file', type=str, help='Output table in CSV format.')
    parser.add_argument('--append', default=False, action='store_true', help="Append to the end of the CSV table.")
    return parser


def process_results(
        data_dir: str,
):
    print(f'PROCESSING DATA: {data_dir} ...')
    files = [str(path.resolve()) for path in Path(data_dir).rglob('*.json')]
    n_files = len(files)
    print(f'N files: {n_files}')
    cnt = 0
    tab_main = dt.Frame()
    for file in files:
        cnt = cnt + 1
        print(f'File {cnt} / {n_files}: {file}')
        if not Path(file).is_file():
            print(colored(f'File {file} does not exist!', 'red'))
        tab = dt.Frame([{'file': file}])
        with open(file, 'r') as f:
            data = json.load(f)
            data = json_normalize(data)
            data = list(data)[0]
            for key, value in data.items():
                tab[key] = value
        tab_main = dt.rbind([tab_main, tab], force=True)
    return tab_main


def preprocess_table(tab):
    tab['method'] = ''
    tab['dual_samp'] = -1.0
    tab['filtered'] = False
    tab['bucketed'] = False
    tab['method_full'] = ''
    tab['method_type'] = ''
    tab['filtered_bucketed'] = '--'
    for i in range(tab.nrows):
        tab[i, 'method'] = f['guards_1.method']
        if 'guards_2.method' in tab.names and tab[i, 'guards_2.method'] is not None:
            tab[i, 'method'] += '+' + f['guards_2.method']
        if 'guards_3.method' in tab.names and tab[i, 'guards_3.method'] is not None:
            tab[i, 'method'] += '+' + f['guards_3.method']
        if 'guards_1.param.n_dual_samples' in tab.names and tab[i, 'guards_1.param.n_dual_samples'] is not None:
            tab[i, 'method'] += '-n'
        if 'guards_1.param.n_dual_samples_per_square_unit' in tab.names and tab[i, 'guards_1.param.n_dual_samples_per_square_unit'] is not None:
            tab[i, 'method'] += '-d'
        tab[i, 'method_full'] = f['method']
        if 'guards_1.param.n_dual_samples' in tab.names and tab[i, 'guards_1.param.n_dual_samples'] is not None:
            tab[i, 'dual_samp'] = dt.float64(f['guards_1.param.n_dual_samples'])
            tab[i, 'method_full'] += dt.int64(f['dual_samp'])
        if 'guards_1.param.n_dual_samples_per_square_unit' in tab.names and tab[i, 'guards_1.param.n_dual_samples_per_square_unit'] is not None:
            tab[i, 'dual_samp'] = dt.float64(f['guards_1.param.n_dual_samples_per_square_unit'])
            if 'guards_1.param.n_dual_samples' in tab.names and tab[i, 'guards_1.param.n_dual_samples'] is not None:
                tab[i, 'method_full'] += '-'
            tab[i, 'method_full'] += f['dual_samp']
        if tab[i, 'process_coverage.filtered'] == 'true':
            tab[i, 'filtered'] = True
            tab[i, 'filtered_bucketed'] = 'f' + tab[i, 'filtered_bucketed'][1]
            tab[i, 'method_full'] += '-f'
        if float(tab[i, 'target_regions_bucketing.bucket_ratio_in']) < 1.0:
            tab[i, 'bucketed'] = True
            tab[i, 'filtered_bucketed'] = tab[i, 'filtered_bucketed'][0] + 'b'
            tab[i, 'method_full'] += '-b'
        if tab[i, 'method'].startswith('reflex'):
            tab[i, 'method_type'] = 'reflex'
        elif tab[i, 'method'].startswith('ka'):
            tab[i, 'method_type'] = 'ka'
        elif tab[i, 'method'].startswith('ccdt'):
            tab[i, 'method_type'] = 'ccdt'
        elif tab[i, 'method'].startswith('cov'):
            tab[i, 'method_type'] = 'cov'


def main(args):
    tab = process_results(args.input_data_dir)
    preprocess_table(tab)  # Modifies the table in-place.
    tab.to_csv(args.output_csv_file, append=args.append)


if __name__ == '__main__':
    main(args_parser().parse_args())
