`include "timescale.v"
`include "camera_control_defs.vh"
`include "ov7670_regs.vh"

`default_nettype wire

module CameraControl_TOP (
    input sys_clk,          // 27MHz clock input
    input sys_rst_n,        // reset input
    inout master_scl,       // I2C SCL
    inout master_sda,       // I2C SDA
    output reg led_out,
    output cam_reset,
    output cam_clk,
    output cam_pwdn,
    output reg config_done  // New flag for top.v to know when camera is ready
);

    typedef enum {
        WAIT_RDY, 
        SEND_INIT, 
        SEND_INIT2, 
        SEND_INIT_DONE, 
        WAIT_CAMERA_INIT_DONE, 
        CAMERA_INIT_DONE,
        WAIT_TRANSMIT_COMPLETE, 
        TRANSMIT_COMPLETE, 
        CHECK_ROM_DATA, 
        START_DELAY
    } CONTROL_STATES;

    localparam [6:0] OV7670_ADDR = 7'h21;

    wire ctrl_done_wire;
    assign cam_reset = sys_rst_n;
    assign cam_clk = sys_clk;
    assign cam_pwdn = 1'b0;

    reg [7:0] data_buffer_out;
    reg store_data;
    reg send_data;
    reg delay_reset;
    reg [7:0] rom_addr;
    CONTROL_STATES controller_state;

    wire tx_en;
    wire [7:0] wr_data;
    wire [2:0] wr_addr;
    wire rx_en;
    wire [7:0] rd_data;
    wire [2:0] rd_addr;

    wire scl_i;
    wire scl_o;
    wire scl_o_oen;

    wire sda_i;
    wire sda_o;
    wire sda_o_oen;

    wire cyc;
    wire [2:0] reg_addr;
    wire cmd_ack;
    wire device_ready;
    wire transmit_error;
    wire delay_done;

    wire [7:0] rom_reg_addr;
    wire [7:0] rom_reg_val;

    assign master_scl = scl_o_oen ? 1'bZ : scl_o;
    assign master_sda = sda_o_oen ? 1'bZ : sda_o;
    assign cyc = tx_en | rx_en;
    assign reg_addr = (tx_en) ? wr_addr : rd_addr;

    // I2C Master core
    i2C_MASTER_Control i2c_master(
        .wb_clk_i(sys_clk),
        .wb_rst_i(1'b0),
        .arst_i(sys_rst_n),
        .wb_dat_i(wr_data),
        .wb_adr_i(reg_addr),
        .wb_we_i(tx_en),
        .wb_stb_i(1'b1),
        .scl_padoen_o(scl_o_oen),
        .scl_pad_i(master_scl),
        .scl_pad_o(scl_o),
        .sda_padoen_o(sda_o_oen),
        .sda_pad_i(master_sda),
        .sda_pad_o(sda_o),
        .wb_cyc_i(cyc),
        .wb_dat_o(rd_data),
        .wb_ack_o(cmd_ack),
        .wb_inta_o()
    );

    // I2C Control FSM
    i2c_control_fsm i2c_controller(
        .clk(sys_clk), 
        .rst_n(sys_rst_n), 
        .device_addr(OV7670_ADDR), 
        .init_done(ctrl_done_wire), 
        .data_in(data_buffer_out),
        .store_data(store_data), 
        .send_data(send_data),
        .tx_en(tx_en), 
        .rx_en(rx_en), 
        .wr_data(wr_data),
        .wr_addr(wr_addr), 
        .rd_data(rd_data), 
        .rd_addr(rd_addr),
        .cmd_ack_i(cmd_ack), 
        .device_rdy(device_ready), 
        .error_o(transmit_error),
        .load_data(1'b0), 
        .recv_data(1'b0)
    );

    // OV7670 configuration ROM
    ov7670_default settings_rom(
        .addr_i(rom_addr), 
        .dout({rom_reg_addr, rom_reg_val})
    );

    // Delay module for reset sequence
    device_delay i2c_init_delay(
        .clk_i(sys_clk), 
        .rst_n(sys_rst_n), 
        .syn_rst(delay_reset), 
        .delay_done(delay_done)
    );

    initial begin
        controller_state <= WAIT_RDY;
        send_data <= 1'b0;
        store_data <= 1'b0;
        led_out <= 1'b1;
        delay_reset <= 1'b0;
        rom_addr <= 8'h00;
        config_done <= 1'b0;
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            controller_state <= WAIT_RDY;
            send_data <= 1'b0;
            led_out <= 1'b1;
            delay_reset <= 1'b0;
            rom_addr <= 8'h00;
            config_done <= 1'b0;
        end else begin
            case (controller_state)
                WAIT_RDY: begin
                    if (ctrl_done_wire && delay_done) begin 
                        controller_state <= CHECK_ROM_DATA;
                    end
                end
                CHECK_ROM_DATA: begin
                    if (rom_reg_addr == 8'hff && rom_reg_val == 8'hff)
                        controller_state <= TRANSMIT_COMPLETE;
                    else if (rom_reg_addr == 8'hff && rom_reg_val == 8'hf0) begin
                        delay_reset <= 1'b1;
                        controller_state <= START_DELAY;
                    end else
                        controller_state <= SEND_INIT;
                end
                START_DELAY: begin
                    delay_reset <= 1'b0;
                    if (!delay_done) begin
                        controller_state <= WAIT_RDY;
                        rom_addr <= rom_addr + 1'b1;
                    end
                end
                SEND_INIT: begin
                    controller_state <= SEND_INIT2;
                    store_data <= 1'b1;
                    data_buffer_out <= rom_reg_addr;
                end
                SEND_INIT2: begin
                    controller_state <= SEND_INIT_DONE;
                    store_data <= 1'b1;
                    data_buffer_out <= rom_reg_val;
                end
                SEND_INIT_DONE: begin
                    store_data <= 1'b0;
                    controller_state <= WAIT_CAMERA_INIT_DONE;
                end
                WAIT_CAMERA_INIT_DONE: begin
                    send_data <= 1'b1;
                    controller_state <= CAMERA_INIT_DONE;
                end
                CAMERA_INIT_DONE: begin
                    send_data <= 1'b0;
                    controller_state <= WAIT_TRANSMIT_COMPLETE;
                end
                WAIT_TRANSMIT_COMPLETE: begin
                    if (transmit_error)
                        controller_state <= TRANSMIT_COMPLETE;
                    else if (device_ready) begin
                        controller_state <= CHECK_ROM_DATA;
                        rom_addr <= rom_addr + 1'b1;
                    end
                end
                TRANSMIT_COMPLETE: begin
                    led_out <= 1'b0;
                    config_done <= 1'b1; // Signal to HDMI pipeline that camera is active
                end
            endcase
        end
    end
endmodule
