`timescale 1ns / 1ps

module camera_interface (
    input  wire        clk,         // 27MHz base input
    input  wire        clk_100,     // 108MHz (Fast FSM & I2C clock)
    input  wire        rst_n,
    input  wire [3:0]  key,         // Unused, but kept for top_module compatibility
    
    // Output to HDMI Interface
    output wire        rd_en,       // Acts as write_enable for the next stage
    output wire [9:0]  data_count_r,
    output wire [15:0] dout,
    
    // Physical Camera Pins
    input  wire        cmos_pclk,
    input  wire        cmos_href,
    input  wire        cmos_vsync,
    input  wire [7:0]  cmos_db,
    inout  wire        cmos_sda,
    inout  wire        cmos_scl,
    output wire        cmos_xclk,
    
    // Debug LEDs
    output wire [3:0]  led
);

    // ========================================================
    // 1. CLOCK & SYNC
    // ========================================================
    // Bypass PLL: Feed physical 27MHz directly to camera!
    assign cmos_xclk = clk;
    assign data_count_r = 10'd0; // Unused 

    // Synchronize camera timing signals to our fast 108MHz clock
    reg vsync_1, vsync_2;
    reg pclk_1, pclk_2;
    reg href_1, href_2;

    always @(posedge clk_100) begin
        vsync_1 <= cmos_vsync; vsync_2 <= vsync_1;
        pclk_1  <= cmos_pclk;  pclk_2  <= pclk_1;
        href_1  <= cmos_href;  href_2  <= href_1;
    end

    // ========================================================
    // 2. I2C MASTER
    // ========================================================
    reg        start, stop;
    reg  [7:0] wr_data;
    wire [1:0] ack;

    i2c_top i2c_inst (
        .clk(clk_100),
        .rst_n(rst_n),
        .start(start),
        .stop(stop),
        .wr_data(wr_data),
        .ack(ack),
        .sda(cmos_sda),
        .scl(cmos_scl),
        .idle()
    );

    // ========================================================
    // 3. FSM REGISTERS
    // ========================================================
    localparam idle = 0, start_sccb = 1, write_address = 2, write_data = 3, 
               digest_loop = 4, delay = 5, vsync_fedge = 6, byte1 = 7, 
               byte2 = 8, fifo_write = 9;

    reg [3:0]  state_q = idle, state_d;
    reg [26:0] delay_q = 0, delay_d;
    reg        start_delay_q = 0, start_delay_d;
    reg [8:0]  message_index_q = 0, message_index_d;
    reg        delay_finish;
    
    reg [3:0]  led_q = 4'b1111, led_d;
    reg [15:0] pixel_q = 0, pixel_d;
    reg        wr_en_q = 0, wr_en_d;

    assign led = led_q;
    assign dout = pixel_q;
    assign rd_en = wr_en_q;

    // ========================================================
    // 4. OV2640 INITIALIZATION ROM (VGA RGB565)
    // ========================================================
    localparam MSG_INDEX = 40;
    reg [15:0] message [0:MSG_INDEX];

    initial begin
        message[0] = 16'hFF_01; message[1] = 16'h12_80; message[2] = 16'hFF_00;
        message[3] = 16'h2C_FF; message[4] = 16'h2E_DF; message[5] = 16'hFF_01;
        message[6] = 16'h3C_32; message[7] = 16'h11_00; message[8] = 16'h09_02;
        message[9] = 16'h04_28; message[10]= 16'h13_E5; message[11]= 16'h14_48;
        message[12]= 16'h2C_0C; message[13]= 16'h33_78; message[14]= 16'h3A_33;
        message[15]= 16'h3B_FB; message[16]= 16'h3E_00; message[17]= 16'h43_11;
        message[18]= 16'h16_02; message[19]= 16'h39_02; message[20]= 16'h35_88;
        message[21]= 16'h22_0A; message[22]= 16'h37_40; message[23]= 16'h23_00;
        message[24]= 16'h34_A0; message[25]= 16'h36_1A; message[26]= 16'h06_02;
        message[27]= 16'h0C_00; message[28]= 16'h0D_B7; message[29]= 16'h0E_01;
        message[30]= 16'h15_00; message[31]= 16'hFF_00; message[32]= 16'hE5_7F;
        message[33]= 16'hF9_C0; message[34]= 16'h41_24; message[35]= 16'hE0_14;
        message[36]= 16'h76_B5; message[37]= 16'h33_E5; message[38]= 16'h34_48;
        message[39]= 16'h40_C0; message[40]= 16'hDA_00; 
    end

    // ========================================================
    // 5. SEQUENTIAL LOGIC
    // ========================================================
    always @(posedge clk_100 or negedge rst_n) begin
        if (!rst_n) begin
            state_q         <= idle;
            delay_q         <= 0;
            start_delay_q   <= 0;
            message_index_q <= 0;
            led_q           <= 4'b1111;
            pixel_q         <= 0;
            wr_en_q         <= 0;
        end else begin
            state_q         <= state_d;
            delay_q         <= delay_d;
            start_delay_q   <= start_delay_d;
            message_index_q <= message_index_d;
            led_q           <= led_d;
            pixel_q         <= pixel_d;
            wr_en_q         <= wr_en_d;
        end
    end

    // ========================================================
    // 6. FSM NEXT-STATE LOGIC (Auto-Retry + Pixel Capture)
    // ========================================================
    always @* begin
        state_d         = state_q;
        led_d           = led_q;
        start           = 0;
        stop            = 0;
        wr_data         = 0;
        start_delay_d   = start_delay_q;
        delay_d         = delay_q;
        delay_finish    = 0;
        message_index_d = message_index_q;
        pixel_d         = pixel_q;
        wr_en_d         = 0;
        
        // Dynamic Startup Delay
        if(start_delay_q) delay_d = delay_q + 1'b1;
        
        if((message_index_q == 0 ? delay_q[24] : delay_q[16]) && message_index_q != (MSG_INDEX+1) && (state_q != start_sccb)) begin
            delay_finish = 1;
            start_delay_d = 0;
            delay_d = 0;
        end else if((delay_q[26] && message_index_q == (MSG_INDEX+1)) || (delay_q[26] && state_q == start_sccb)) begin
            delay_finish = 1;
            start_delay_d = 0;
            delay_d = 0;
        end
        
        case(state_q) 
            idle: begin
                if(delay_finish) begin
                    state_d = start_sccb;
                    start_delay_d = 0;
                end else start_delay_d = 1;
            end
                  
            start_sccb: begin
                start = 1;
                wr_data = 8'h42;
                state_d = write_address;                        
            end
                  
            write_address: begin
                if(ack==2'b11) begin 
                    wr_data = message[message_index_q][15:8];
                    state_d = write_data;
                end else if(ack==2'b10) begin 
                    stop = 1; start_delay_d = 1; state_d = delay; // NACK retry
                end
            end
                  
            write_data: begin
                if(ack==2'b11) begin 
                    wr_data = message[message_index_q][7:0];
                    state_d = digest_loop;
                end else if(ack==2'b10) begin 
                    stop = 1; start_delay_d = 1; state_d = delay;
                end
            end
                  
            digest_loop: begin
                if(ack==2'b11) begin
                    stop = 1; start_delay_d = 1;
                    message_index_d = message_index_q + 1'b1;
                    state_d = delay;
                end else if(ack==2'b10) begin 
                    stop = 1; start_delay_d = 1; state_d = delay;
                end
            end
                  
            delay: begin
                if(message_index_q == (MSG_INDEX+1) && delay_finish) begin 
                    state_d = vsync_fedge;
                    led_d = 4'b0110; // I2C Success: LEDs 2 and 3 ON!
                end else if(state_q==0 && delay_finish) begin 
                    state_d = start_sccb;
                end
            end
                  
            vsync_fedge: begin
                if(vsync_1==0 && vsync_2==1) state_d = byte1;
            end
            
            byte1: begin
                if(pclk_1==1 && pclk_2==0 && href_1==1 && href_2==1) begin
                    pixel_d[15:8] = cmos_db;
                    state_d = byte2;
                end else if(vsync_1==1 && vsync_2==1) begin
                    state_d = vsync_fedge;
                end
            end
                    
            byte2: begin
                if(pclk_1==1 && pclk_2==0 && href_1==1 && href_2==1) begin
                    pixel_d[7:0] = cmos_db;
                    state_d = fifo_write;
                end else if(vsync_1==1 && vsync_2==1) begin
                    state_d = vsync_fedge;
                end
            end
                    
            fifo_write: begin
                wr_en_d = 1;
                state_d = byte1;
            end
                    
            default: state_d = idle;
        endcase
    end
endmodule
