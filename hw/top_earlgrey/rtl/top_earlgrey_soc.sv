module top_core (
    input  logic        clk_i,
    input  logic        rst_ni,

    input  logic        fetch_enable_i,
    input  logic        en_ifetch_i,

    // SPI device interface
    input  logic        spi_sclk,
    input  logic        spi_cs,
    output logic [1:0]  spi_mode,
    input  logic        spi_sdi0,
    input  logic        spi_sdi1,
    input  logic        spi_sdi2,
    input  logic        spi_sdi3,
    output logic        spi_sdo0,
    output logic        spi_sdo1,
    output logic        spi_sdo2,
    output logic        spi_sdo3,

    // GPIO interface
    output logic [31:0] gpio_o
);

    logic rst_no;

    tlul_pkg::tl_h2d_t core_2_xbar_main;
    tlul_pkg::tl_d2h_t xbar_main_2_core;
    tlul_pkg::tl_h2d_t spi_2_xbar_main;
    tlul_pkg::tl_d2h_t xbar_main_2_spi;
    tlul_pkg::tl_h2d_t jtag_2_xbar_main;
    tlul_pkg::tl_d2h_t xbar_main_2_jtag;

    tlul_pkg::tl_h2d_t xbar_main_2_instr;
    tlul_pkg::tl_d2h_t instr_2_xbar_main;
    tlul_pkg::tl_h2d_t xbar_main_2_data;
    tlul_pkg::tl_d2h_t data_2_xbar_main;
    tlul_pkg::tl_h2d_t xbar_main_2_peri_device;
    tlul_pkg::tl_d2h_t peri_device_2_xbar_main;

    // remove assignment when connecting new module
    assign jtag_2_xbar_main = tlul_pkg::TL_H2D_DEFAULT;

    ibex_pkg::ibex_mubi_t   fetch_enable;
    prim_mubi_pkg::mubi4_t  en_ifetch;

    assign fetch_enable = (fetch_enable_i) ? ibex_pkg::IbexMuBiOn : ibex_pkg::IbexMuBiOff;
    assign en_ifetch    = (en_ifetch_i)    ? prim_mubi_pkg::MuBi4True : prim_mubi_pkg::MuBi4False;

    // reset synchronizer
    rst_gen u_rst_gen (
        .clk_i  (clk_i  ),
        .rst_ni (rst_ni ),
        .rst_no (rst_no )
    );

    // 3 master, 3 slave
    xbar_main u_xbar_main 
    (
        .clk_i              (clk_i                    ),
        .rst_ni             (rst_no                   ),

        .tl_core_i          (core_2_xbar_main         ),
        .tl_core_o          (xbar_main_2_core         ),
        .tl_spi_i           (spi_2_xbar_main          ),
        .tl_spi_o           (xbar_main_2_spi          ),
        .tl_jtag_i          (jtag_2_xbar_main         ),
        .tl_jtag_o          (xbar_main_2_jtag         ),

        .tl_instr_o         (xbar_main_2_instr        ),
        .tl_instr_i         (instr_2_xbar_main        ),
        .tl_data_o          (xbar_main_2_data         ),
        .tl_data_i          (data_2_xbar_main         ),
        .tl_peri_device_o   (xbar_main_2_peri_device  ),
        .tl_peri_device_i   (peri_device_2_xbar_main  ),
        
        .scanmode_i         (prim_mubi_pkg::MuBi4False)
    );

    // 1 master
    spi_device_tlul u_spi_device_tlul 
    (
        .clk_i      (clk_i           ),
        .rst_ni     (rst_no          ),
        .test_mode  (1'b1            ),
        .spi_sclk   (spi_sclk        ),
        .spi_cs     (spi_cs          ),
        .spi_mode   (spi_mode        ),
        .spi_sdi0   (spi_sdi0        ),
        .spi_sdi1   (spi_sdi1        ),
        .spi_sdi2   (spi_sdi2        ),
        .spi_sdi3   (spi_sdi3        ),
        .spi_sdo0   (spi_sdo0        ),
        .spi_sdo1   (spi_sdo1        ),
        .spi_sdo2   (spi_sdo2        ),
        .spi_sdo3   (spi_sdo3        ),
        .tl_i       (xbar_main_2_spi ),
        .tl_o       (spi_2_xbar_main )
    );

    // 1 master, 2 slave
    // cpu_cluster u_cpu_cluster (
    //     .clk_i              (clk_i             ),
    //     .rst_ni             (rst_no            ),

    //     .fetch_enable_i     (fetch_enable      ),
    //     .en_ifetch_i        (en_ifetch         ),

    //     .tl_core_i          (xbar_main_2_core  ),
    //     .tl_core_o          (core_2_xbar_main  ),

    //     .tl_instr_i         (xbar_main_2_instr ),
    //     .tl_instr_o         (instr_2_xbar_main ),
    //     .tl_data_i          (xbar_main_2_data  ),
    //     .tl_data_o          (data_2_xbar_main  )
    // );
    rv_core_ibex #(
        .AlertAsyncOn(alert_handler_reg_pkg::AsyncOn[64:61]),
        .RndCnstLfsrSeed(RndCnstRvCoreIbexLfsrSeed),
        .RndCnstLfsrPerm(RndCnstRvCoreIbexLfsrPerm),
        .RndCnstIbexKeyDefault(RndCnstRvCoreIbexIbexKeyDefault),
        .RndCnstIbexNonceDefault(RndCnstRvCoreIbexIbexNonceDefault),
        .PMPEnable(RvCoreIbexPMPEnable),
        .PMPGranularity(RvCoreIbexPMPGranularity),
        .PMPNumRegions(RvCoreIbexPMPNumRegions),
        .MHPMCounterNum(RvCoreIbexMHPMCounterNum),
        .MHPMCounterWidth(RvCoreIbexMHPMCounterWidth),
        .RV32E(RvCoreIbexRV32E),
        .RV32M(RvCoreIbexRV32M),
        .RV32B(RvCoreIbexRV32B),
        .RegFile(RvCoreIbexRegFile),
        .BranchTargetALU(RvCoreIbexBranchTargetALU),
        .WritebackStage(RvCoreIbexWritebackStage),
        .ICache(RvCoreIbexICache),
        .ICacheECC(RvCoreIbexICacheECC),
        .ICacheScramble(RvCoreIbexICacheScramble),
        .BranchPredictor(RvCoreIbexBranchPredictor),
        .DbgTriggerEn(RvCoreIbexDbgTriggerEn),
        .DbgHwBreakNum(RvCoreIbexDbgHwBreakNum),
        .SecureIbex(RvCoreIbexSecureIbex),
        .DmHaltAddr(RvCoreIbexDmHaltAddr),
        .DmExceptionAddr(RvCoreIbexDmExceptionAddr),
        .PipeLine(RvCoreIbexPipeLine)
    ) u_rv_core_ibex (
        // [61]: fatal_sw_err
        // [62]: recov_sw_err
        // [63]: fatal_hw_err
        // [64]: recov_hw_err
        .alert_tx_o  ( alert_tx[64:61] ),
        .alert_rx_i  ( alert_rx[64:61] ),

        // Inter-module signals
        .rst_cpu_n_o(),
        .ram_cfg_i(ast_ram_1p_cfg),
        .hart_id_i(rv_core_ibex_hart_id),
        .boot_addr_i(rv_core_ibex_boot_addr),
        .irq_software_i(rv_plic_msip),
        .irq_timer_i(rv_core_ibex_irq_timer),
        .irq_external_i(rv_plic_irq),
        .esc_tx_i(alert_handler_esc_tx[0]),
        .esc_rx_o(alert_handler_esc_rx[0]),
        .debug_req_i(rv_dm_debug_req),
        .crash_dump_o(rv_core_ibex_crash_dump),
        .lc_cpu_en_i(lc_ctrl_lc_cpu_en),
        .pwrmgr_cpu_en_i(pwrmgr_aon_fetch_en),
        .pwrmgr_o(rv_core_ibex_pwrmgr),
        .nmi_wdog_i(aon_timer_aon_nmi_wdog_timer_bark),
        .edn_o(edn0_edn_req[7]),
        .edn_i(edn0_edn_rsp[7]),
        .icache_otp_key_o(otp_ctrl_sram_otp_key_req[2]),
        .icache_otp_key_i(otp_ctrl_sram_otp_key_rsp[2]),
        .fpga_info_i(fpga_info_i),
        .corei_tl_h_o(main_tl_rv_core_ibex__corei_req),
        .corei_tl_h_i(main_tl_rv_core_ibex__corei_rsp),
        .cored_tl_h_o(main_tl_rv_core_ibex__cored_req),
        .cored_tl_h_i(main_tl_rv_core_ibex__cored_rsp),
        .cfg_tl_d_i(rv_core_ibex_cfg_tl_d_req),
        .cfg_tl_d_o(rv_core_ibex_cfg_tl_d_rsp),
        .scanmode_i,
        .scan_rst_ni,

        // Clock and reset connections
        .clk_i (clkmgr_aon_clocks.clk_main_infra),
        .clk_edn_i (clkmgr_aon_clocks.clk_main_infra),
        .clk_esc_i (clkmgr_aon_clocks.clk_io_div4_secure),
        .clk_otp_i (clkmgr_aon_clocks.clk_io_div4_secure),
        .rst_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
        .rst_edn_ni (rstmgr_aon_resets.rst_lc_n[rstmgr_pkg::Domain0Sel]),
        .rst_esc_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel]),
        .rst_otp_ni (rstmgr_aon_resets.rst_lc_io_div4_n[rstmgr_pkg::Domain0Sel])
    );
    // 1 slave
    peri_device u_peri_device (
        .clk_i              (clk_i                    ),
        .rst_ni             (rst_no                   ),

        .tl_peri_device_i   (xbar_main_2_peri_device  ),
        .tl_peri_device_o   (peri_device_2_xbar_main  ),

        .gpio_o             (gpio_o                   )
    );

endmodule