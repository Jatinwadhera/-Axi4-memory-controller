# AXI4 Memory Controller with Adaptive Cache

[![Vivado](https://img.shields.io/badge/Vivado-2020.2-blue)](https://www.xilinx.com/products/design-tools/vivado.html)
[![SystemVerilog](https://img.shields.io/badge/HDL-SystemVerilog-orange)](https://en.wikipedia.org/wiki/SystemVerilog)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Target FPGA](https://img.shields.io/badge/FPGA-xc7k325tffg900--2-purple)](https://www.xilinx.com/products/silicon-devices/fpga/kintex-7.html)

A fully AXI4-compliant memory controller with adaptive 4-way set-associative write-back cache, verified with constrained-random testbenches. Targets the Xilinx Kintex-7 FPGA (`xc7k325tffg900-2`) using Vivado 2020.2.

---

## Table of Contents

- [Features](#features)
- [Architecture Overview](#architecture-overview)
- [Directory Structure](#directory-structure)
- [RTL Modules](#rtl-modules)
- [Getting Started](#getting-started)
- [Simulation](#simulation)
- [Results](#results)
- [Tools & Dependencies](#tools--dependencies)
- [Author](#author)

---

## Features

- ✅ Full **AXI4 slave interface** with VALID/READY handshake protocol
- ✅ **4-way set-associative write-back cache** with AXI4 burst refill
- ✅ **87% cache hit rate** under sequential workload
- ✅ **APB register interface** for runtime configuration and status readout
- ✅ **Prefetch engine** for speculative line fetching
- ✅ Self-checking **constrained-random XSIM testbench** with SystemVerilog Assertions (SVA)
- ✅ Parameterised design — configurable cache size, line width, associativity
- ✅ RTL-to-GDS flow ready (OpenROAD / Sky130 PDK compatible)

---

## Architecture Overview

```
         AXI4 Master (CPU / DMA)
                  │
         ┌────────▼────────┐
         │  axi4_slave_if  │  ← AXI4 VALID/READY handshake
         └────────┬────────┘
                  │
         ┌────────▼────────┐
         │   cache_ctrl    │  ← 4-way set-associative, write-back
         │  + miss_handler │  ← AXI4 burst refill on miss
         │  + prefetch_eng │  ← Speculative prefetch
         └────────┬────────┘
                  │
         ┌────────▼────────┐
         │  SRAM / DRAM    │  ← Off-chip memory
         └─────────────────┘
                  │
         ┌────────▼────────┐
         │   apb_regs      │  ← APB config/status registers
         └─────────────────┘
```

The top-level module `axi4_mem_ctrl_top` wires all sub-blocks together.

---

## Directory Structure

```
axi4-memory-controller/
├── rtl/
│   ├── axi4_mem_ctrl_top.sv    # Top-level integration
│   ├── axi4_slave_if.sv        # AXI4 slave interface (AR/AW/W/R/B channels)
│   ├── cache_ctrl.sv           # 4-way set-associative write-back cache
│   ├── miss_handler.sv         # Cache miss handler + AXI4 burst refill
│   ├── prefetch_engine.sv      # Speculative prefetch engine
│   └── apb_regs.sv             # APB peripheral register block
├── tb/
│   └── tb_axi4_mem_ctrl.sv     # Self-checking constrained-random testbench
├── constraints/
│   └── timing.xdc              # SDC/XDC timing constraints (Kintex-7)
├── scripts/
│   ├── run_sim.tcl             # Vivado XSIM simulation script
│   ├── synth.tcl               # Vivado synthesis script
│   └── openroad_flow.sh        # OpenROAD RTL-to-GDS flow (Sky130)
├── docs/
│   └── architecture.md         # Detailed micro-architecture notes
├── .github/
│   └── workflows/
│       └── lint.yml            # GitHub Actions: Verilator lint CI
├── pro_axi.xpr                 # Vivado 2020.2 project file
├── LICENSE
└── README.md
```

---

## RTL Modules

| Module | Description |
|--------|-------------|
| `axi4_mem_ctrl_top` | Top-level; integrates all sub-blocks |
| `axi4_slave_if` | Manages AXI4 AR/AW/W/R/B channels; decodes addresses |
| `cache_ctrl` | 4-way set-associative, write-back, LRU replacement |
| `miss_handler` | Handles cache misses; initiates AXI4 burst refill |
| `prefetch_engine` | Predicts next cache line and prefetches speculatively |
| `apb_regs` | APB slave; exposes hit/miss counters and config registers |

---

## Getting Started

### Prerequisites

- Xilinx Vivado 2020.2 (or later)
- QuestaSim / XSIM for simulation
- (Optional) OpenROAD + Sky130 PDK for physical design flow

### Clone the repo

```bash
git clone https://github.com/JatinWadhera/axi4-memory-controller.git
cd axi4-memory-controller
```

### Open in Vivado

```bash
vivado pro_axi.xpr
```

Or run non-interactively:

```bash
vivado -mode batch -source scripts/synth.tcl
```

---

## Simulation

### Run with Vivado XSIM (TCL)

```bash
vivado -mode batch -source scripts/run_sim.tcl
```

### What the testbench covers

- **Constrained-random addressing** across the full address space
- **Sequential burst** reads and writes (AXI4 INCR bursts, lengths 1–16)
- **Back-pressure** on RREADY / WREADY
- **Hit/miss ratio** monitoring via APB status registers
- **SVA assertions** on protocol correctness (no X-prop, handshake compliance)

Expected output:

```
[PASS] AXI4 Write burst test            — 256 transactions
[PASS] AXI4 Read burst test             — 256 transactions
[PASS] Cache hit rate >= 85%            — measured: 87%
[PASS] No protocol violations detected
[PASS] All SVA assertions passed
Simulation PASSED.
```

---

## Results

| Metric | Value |
|--------|-------|
| Cache hit rate (sequential) | **87%** |
| AXI4 protocol violations | **0** |
| Simulation test vectors | **512+** |
| Target device | Kintex-7 xc7k325tffg900-2 |
| Vivado version | 2020.2 |

---

## Tools & Dependencies

| Tool | Version | Purpose |
|------|---------|---------|
| Vivado | 2020.2 | Synthesis, P&R, XSIM simulation |
| QuestaSim | 10.7+ | Alternate simulator |
| OpenROAD | latest | RTL-to-GDS physical design |
| Yosys | 0.9+ | Open-source synthesis |
| Sky130 PDK | latest | Open PDK for GDS flow |

---

## Author

**Jatin Wadhera**  
Electronics and Computer Engineering, Thapar Institute of Engineering and Technology  
📧 [10a27jatinwadhera@gmail.com](mailto:10a27jatinwadhera@gmail.com)  
🔗 [GitHub](https://github.com/JatinWadhera) | [LinkedIn](https://linkedin.com/in/jatin-wadhera-20a37b284)

---

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.
