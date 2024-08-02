#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# python3 process2.py data/maps.csv

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


def args_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input_csv_file', type=str, help='Input table in CSV format.')
    return parser


def latex_tab(
        tab: dt.Frame,
):
    checkmark = '\\checkmark'
    ret = f''
    ret += f'\\documentclass[border=5pt]{{standalone}}\n'
    ret += f'\\usepackage{{booktabs}}\n'
    ret += f'\\usepackage{{threeparttable}}\n'
    ret += f'\\usepackage{{amssymb}}\n'
    ret += f'\\usepackage{{ragged2e}}\n'
    ret += f'\\usepackage{{caption}}\n'
    ret += f'\\begin{{document}}\n'
    ret += f'  \\begin{{threeparttable}}\n'
    ret += f'    \\centering\n'
    ret += f'    \\footnotesize\n'
    ret += f'    \\caption*{{Map Properties.}}\n'
    ret += f'    \\begin{{tabular}}{{*{{8}}{{r}}}}\n'
    ret += f'      \\toprule\n'
    ret += f'      {"Map":7} & {"$n$":4} & {"$h$":3} & {"$x$":3} & {"$y$":3} & {"$xy$":7} & {"$a$":7} & {"Pre":10} \\\\\n'
    ret += f'      \\midrule\n'
    for i in range(tab.nrows):
        row = tab[i, :].to_dict()
        map_name = row["map.name"][0]
        map_name = map_name.replace('_smoothed', '')
        ret += f'      '
        ret += f'{map_name:7}'
        ret += f' & {row["map_info.n_points"][0]:4}'
        ret += f' & {row["map_info.n_holes"][0]:3}'
        ret += f' & {row["map_info.lim_x_max"][0]:3.0f}'
        ret += f' & {row["map_info.lim_y_max"][0]:3.0f}'
        ret += f' & {row["map_info.lim_x_max"][0] * row["map_info.lim_y_max"][0]:7,.0f}'
        ret += f' & {row["map_info.area"][0]:7,.0f}'
        ret += f' & {"" if row["flag"][0] == "test" else checkmark:10}'
        ret += f' \\\\\n'
    ret += f'      \\bottomrule\n'
    ret += f'    \\end{{tabular}}\n'
    ret += f'    \\smallskip\\footnotesize\\justifying\n'
    ret += f'    \\noindent Map: map name'
    ret += f'; $n$: no. vertices'
    ret += f'; $h$: no. holes'
    ret += f'; $x$: width'
    ret += f'; $y$: height'
    ret += f'; $xy$: bounding box area'
    ret += f'; $a$: map area'
    ret += f'; F: used in the final evaluation'
    ret += f'.\n'
    ret += f'  \\end{{threeparttable}}\n'
    ret += f'\\end{{document}}\n'
    return ret


def main(args):
    tab = dt.fread(args.input_csv_file)
    del tab[:, 'file']
    del tab[:, 'map.path']
    tab = tab[:, :, dt.sort(f['map.name'])]
    lt = latex_tab(tab)
    with open('maps.tex', 'w') as file:
        file.write(lt)
    print(lt)


if __name__ == '__main__':
    main(args_parser().parse_args())
