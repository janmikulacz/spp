#!/usr/bin/env bash

for context in "val" "test" "extra"
do
  python process1.py -i "data/${context}" -o "data/${context}.csv"
done
