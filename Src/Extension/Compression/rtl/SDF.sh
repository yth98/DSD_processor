#!/bin/tcsh
echo decompression ; ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+decompression+SDF +access+r | grep -e violation -e result -e solution: -e via
echo compression ; ncverilog Final_tb.v CHIP_syn.v slow_memory.v tsmc13.v +define+compression+SDF +access+r | grep -e violation -e result -e solution: -e via
