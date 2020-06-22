# Digital System Design - Spring 2020
# Pipelined RISC-V Design

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
Past status:6/12
Has not passed any test
有新進度寫在這邊
Current status:6/21
Has not passed any test
has not finished ALU logic support for all kind of instruction
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


## Work pending:
### Baseline
* [] Support 20 Instructions
    - xor
    - addi andi ori xori slli srai srli slti
      (I-type 00100)
    - bne 
      (branch when rs1 - rs2 != 0)
    - ALUctrlEX ALU_EX
    - Jump instructions
      PCpX / PCpi propagation
* [] Caches
    - DCACHE
* [] Solve the hazards
    - Data forwarding unit
      EX hazard
      MEM hazard
    - Hazard detection / Pipeline stall unit
      Load-Use Data Hazard
      Branch Hazard
      DCACHE stall
* [] Synthesis
### Extension
* [] Branch Prediction
* [] L2 Cache
* [] Compressed Instructions
