#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# python3 process1.py -i data/val  --flag val  -o data/maps.csv
# python3 process1.py -i data/test --flag test -o data/maps.csv --append

import json
import argparse

from pathlib import Path

from termcolor import colored

import datatable as dt

from json_normalize import json_normalize


def args_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_data_dir', type=str, help='Directory with experimental data.')
    parser.add_argument('-o', '--output_csv_file', type=str, help='Output table in CSV format.')
    parser.add_argument('--append', default=False, action='store_true', help="Append to the end of the CSV table.")
    parser.add_argument('--flag', type=str, help='Flag to be added to the table.')
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


def main(args):
    tab = process_results(args.input_data_dir)
    if args.flag:
        tab['flag'] = args.flag
    tab.to_csv(args.output_csv_file, append=args.append)


if __name__ == '__main__':
    main(args_parser().parse_args())
