#!/usr/bin/env bash

for context in "val" "test" "extra"
do
  python process2.py -c "${context}" -i "data/${context}.csv"
done
