#!/usr/bin/env bash
set -eu

cd build
make -j 8
./path_planning
