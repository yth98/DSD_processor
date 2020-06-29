# Digital System Design - Spring 2020
# Pipelined RISC-V Design

---------------------------------
Current status: 6/29
Passed: noHazzard hasHazard (47 >= nb >= 3)
        BrPred decompression compression
TODO:   Synthesize everything again and update AT_report.txt
        Report and Presentation slide (doesn't need to upload to GitHub)
---------------------------------

## Work:
### Baseline
* [v] Support 20 Instructions
    - xor
    - addi andi ori xori slli srai srli slti (I-type 00100)
    - bne
    - ALUctrlEX ALU_EXyy
* [v] Caches
* [v] Solve the hazards
    - Data forwarding unit
      EX hazard
      MEM hazard
    - Hazard detection / Pipeline stall unit
      Load-Use Data Hazard
      Branch Hazard
      ICACHE stall
      DCACHE stall
* [ ] Synthesis
### Extension
* [v] Branch Prediction
* [v] L2 Cache
* [v] Compressed Instructions