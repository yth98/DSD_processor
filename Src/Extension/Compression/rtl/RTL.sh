#!/bin/tcsh
echo -n DeComp ; ncverilog Final_tb.v CHIP.v slow_memory.v +define+decompression +access+r | grep -e result -e solution:
echo -n Compre ; ncverilog Final_tb.v CHIP.v slow_memory.v +define+compression +access+r | grep -e result -e solution:
