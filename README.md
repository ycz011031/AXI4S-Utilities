# AXI4 Utilities - Source Modules Overview

This README provides an overview of the main Verilog modules found in the `SRC/sources_1` directory of this project. These modules are designed for AXI4-Stream data switching and conversion, typically used in FPGA-based data path designs.

## Module List

### 1. `axi4_straddle_convertor`
- **File:** SRC/sources_1/axi4_straddle_convertor.v
- **Description:**
  - Converts and buffers AXI4-Stream data with support for straddle mode.
  - Handles both slave and master AXI interfaces.
  - Includes error signaling for invalid states.
  - Parameterized for AXI TUSER width and buffer size.

### 2. `axi4_switch_custom`
- **File:** SRC/sources_1/axi4_switch_custom.v
- **Description:**
  - Dual input, single output AXI4-Stream switch.
  - Assumes symmetrical input/output ports and a common clock.
  - Does **not** support straddle mode.
  - Parameters for TDATA, TUSER, and TKEEP widths.

### 3. `axi4_switch_custom_31`
- **File:** SRC/sources_1/axi4_switch_31.v
- **Description:**
  - Variant of the custom AXI4-Stream switch (supports 3 to 1 switching).
  - Similar interface and parameters as `axi4_switch_custom`.

### 4. `tkeep_convertor` and `tkeep_byte_to_dword`
- **File:** SRC/sources_1/tkeep_convertor.v
- **Description:**
  - `tkeep_convertor`: Expands a 16-bit dword TKEEP signal to a 64-bit byte TKEEP signal (useful for PCIe to FIFO conversions).
  - `tkeep_byte_to_dword`: Compresses a 64-bit byte TKEEP signal back to 16 bits.
  - Both modules use simple combinational logic for conversion.

### 5. `pcie_ats_snoop`
- **File:** SRC/sources_1/pcie_ats_snoop.v
- **Description:**
  - Monitors PCIe CQ interface for ATS messages.
  - Extracts and outputs ATS-related fields (e.g., tag, message code, routing).
  - Transparent AXI-Stream passthrough.
  - Active-low reset and Start-of-Packet (SOP) detection.

### 6. `pcie_cq_type_counter`
- **File:** SRC/sources_1/pcie_cq_type_counter.v
- **Description:**
  - Counts the number of appearances of each PCIe transaction type on the CQ interface.
  - Provides 8-bit saturating counters for 16 transaction types (0x0 to 0xF).
  - Transparent AXI-Stream passthrough.
  - Active-low reset and Start-of-Packet (SOP) detection.

---

## Usage Notes
- All modules are written in Verilog and use standard AXI4-Stream conventions.
- Parameterization allows for flexible data and user signal widths.
- See each module's source file for detailed port descriptions and implementation details.

## Directory Structure
- `SRC/sources_1/` contains the main Verilog source files for the project.
- Testbenches and additional simulation files are located in sibling directories (e.g., `SRC/sim_1/`, `SRC/sim_2/`, etc.).

---

For further details, refer to the comments and documentation within each Verilog source file.
