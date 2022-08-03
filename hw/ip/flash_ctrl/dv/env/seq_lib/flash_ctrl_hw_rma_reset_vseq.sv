// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Run rma request event with random resets
class flash_ctrl_hw_rma_reset_vseq extends flash_ctrl_hw_rma_vseq;
  typedef enum {DVStRmaPageSel     = 0,
                DVStRmaErase       = 1,
                DVStRmaEraseWait   = 2,
                DVStRmaWordSel     = 3,
                DVStRmaProgram     = 4,
                DVStRmaProgramWait = 5,
                DVStRmaRdVerify    = 6} reset_state_index_e;
  `uvm_object_utils(flash_ctrl_hw_rma_reset_vseq)
  `uvm_object_new

  task body();
    logic [RmaSeedWidth-1:0] rma_seed;
    int                      state_wait_timeout_ns = 50000; // 50 us

    // INITIALIZE FLASH REGIONS
    init_data_part();
    init_info_part();
    fork begin
      // SEND RMA REQUEST (Erases the Flash and Writes Random Data To All Partitions)
      fork
        begin
          `uvm_info("Test", "RMA REQUEST", UVM_LOW)
          rma_seed = $urandom;  // Random RMA Seed
          send_rma_req(rma_seed);
        end
        begin
          reset_state_index_e reset_state_index = $urandom_range(DVStRmaPageSel, DVStRmaRdVerify);
          // Assert reset during RMA state transition
          `uvm_info("Test", $sformatf("Reset index: %0d", reset_state_index), UVM_LOW)
          `DV_SPINWAIT(wait(cfg.flash_ctrl_dv_vif.rma_state == reset_state_index);,
                       $sformatf("Timed out waiting for rma_state: %s", reset_state_index.name),
                       state_wait_timeout_ns)
          // Give more cycles for long stages
          // to trigger reset in the middle of the state.
          if (reset_state_index inside {StRmaRdVerify, StRmaErase}) cfg.clk_rst_vif.wait_clks(10);
          `uvm_info(`gfn, "RESET", UVM_LOW)
          lc_ctrl_if_rst();  // Restore lc_ctrl_if to Reset Values
          cfg.seq_cfg.disable_flash_init = 1;  // Disable Flash Random Initialisation
          apply_reset();
        end
      join_any
      disable fork;
      // Since the 2nd begin/end wait for substate of RMA,
      // the 2nd begin/end always finish first.
      // So diable fork only terminate send_rma_req.
    end join // fork begin

    `uvm_info("Test", "RMA END", UVM_LOW)
    cfg.clk_rst_vif.wait_clks($urandom_range(10, 100));

     // INITIALIZE FLASH REGIONS
     init_data_part();
     init_info_part();
     `uvm_info("Test", "RMA REQUEST", UVM_LOW)
     rma_seed = $urandom;  // Random RMA Seed
     send_rma_req(rma_seed);
     cfg.clk_rst_vif.wait_clks($urandom_range(10, 100));

     // CHECK HOST SOFTWARE HAS NO ACCESS TO THE FLASH
     // Attempt to Read from FLASH Controller
     do_flash_ctrl_access_check();
  endtask
endclass // flash_ctrl_hw_rma_reset_vseq