// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// tl_main package generated by `tlgen.py` tool

package tl_main_pkg;

  localparam logic [31:0] ADDR_SPACE_INSTR       = 32'h 00000000;
  localparam logic [31:0] ADDR_SPACE_DATA        = 32'h 20000000;
  localparam logic [31:0] ADDR_SPACE_PERI_DEVICE = 32'h 30000000;

  localparam logic [31:0] ADDR_MASK_INSTR       = 32'h 0fffffff;
  localparam logic [31:0] ADDR_MASK_DATA        = 32'h 0fffffff;
  localparam logic [31:0] ADDR_MASK_PERI_DEVICE = 32'h 0fffffff;

  localparam int N_HOST   = 3;
  localparam int N_DEVICE = 3;

  typedef enum int {
    TlInstr = 0,
    TlData = 1,
    TlPeriDevice = 2
  } tl_device_e;

  typedef enum int {
    TlCore = 0,
    TlSpi = 1,
    TlJtag = 2
  } tl_host_e;

endpackage