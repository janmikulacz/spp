#!/usr/bin/env bash

python save_map_info.py --instances ../AGP/maps-val.txt --subset val
python save_map_info.py --instances ../AGP/maps-test.txt --subset test
python process1.py -i data/val  --flag val  -o data/maps.csv
python process1.py -i data/test --flag test -o data/maps.csv --append
python process2.py -i data/maps.csv
