#!/usr/bin/env bash
set -eu

simulator=$PWD/../../simulator/term3_sim_linux/term3_sim_linux/term3_sim.x86_64

cd build
make -j 8
if ! pgrep $(basename $simulator) &> /dev/null; then
   $simulator &
fi
./path_planning
