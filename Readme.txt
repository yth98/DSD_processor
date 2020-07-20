# Digital System Design - Spring 2020
# Pipelined RISC-V Design
# Contributors: Turknight / AlbertChen000 / yth98

---------------------------------
Current status: 6/30 (Final submission version)
Passed: noHazzard hasHazard (47 >= nb >= 3)
        BrPred decompression compression
---------------------------------

* Report and Presentation slide are not uploaded to GitHub.

* We modified I_mem_BrPred:
在branch prediction裡有新的I指令
原因是因為我們覺得本來的有錯誤
所以改了一些數字去跑
最後是拿這個過的

## Work:
### Baseline
* [v] Support 20 Instructions
    - xor
    - addi andi ori xori slli srai srli slti
    - bne
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
* [v] Synthesis
### Extension
* [v] Branch Prediction
* [v] L2 Cache
* [v] Compressed Instructions