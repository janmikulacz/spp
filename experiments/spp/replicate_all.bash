#!/usr/bin/env bash

for context in "val" "test" "extra"
do
  python run_all.py -c "${context}"
done
