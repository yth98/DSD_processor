#!/bin/tcsh
echo -n BA-H ; ncverilog Final_tb.v CHIP.v slow_memory.v +define+noHazard +access+r | grep -e result -e solution:
echo -n BA+H ; ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r | grep -e result -e solution:
echo -n L1-H ; ncverilog Final_tb.v L2debug/CHIP_onlyL1.v slow_memory.v +define+noHazard +access+r | grep -e result -e solution:
echo -n L1+H ; ncverilog Final_tb.v L2debug/CHIP_onlyL1.v slow_memory.v +define+hasHazard +access+r | grep -e result -e solution:
echo -n L2-H ; ncverilog Final_tb.v L2debug/CHIP_L2.v slow_memory.v +define+noHazard +access+r | grep -e result -e solution:
echo -n L2+H ; ncverilog Final_tb.v L2debug/CHIP_L2.v slow_memory.v +define+hasHazard +access+r | grep -e result -e solution:
