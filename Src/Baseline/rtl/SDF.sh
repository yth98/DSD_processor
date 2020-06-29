#!/bin/tcsh
echo Baseline noHazard ; ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+noHazard+SDF +access+r | grep -e violation -e result -e solution: -e via
echo Baseline hasHazard ; ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+hasHazard+SDF +access+r | grep -e violation -e result -e solution: -e via
