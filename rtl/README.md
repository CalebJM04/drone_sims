# RTL verification

`collision_predictor.sv` implements the integer closest-approach decision used by
the Python fixed-point golden model. It deliberately uses combinational division;
the first goal is behavioral equivalence, not timing closure. A later FPGA design
can pipeline or replace the divider while retaining the same vectors.

Run `make rtl` when Icarus Verilog is installed. `tools/run_rtl.py` generates
random stimuli, calculates expected results with the independent Python model,
compiles the RTL, and fails on any risk or time-to-closest-approach mismatch.

`make vectors` produces a CSV usable by another HDL simulator or a board-level
testbench.

`packet_parser.sv` consumes the frozen 36-byte protocol as a byte stream, checks
the header and CRC, and exposes signed state fields. `neighbor_table.sv` performs
bounded state storage, rollover-aware sequence rejection, boot-session reset,
retired-session replay rejection, lookup, and oldest-entry eviction. The RTL run
also executes an end-to-end parser/table conformance test using the protocol's
golden frame and a deliberately corrupted frame.
