`timescale 1ns/1ps
module tb_packet_pipeline;
    logic clk = 0, rst = 1, byte_valid = 0;
    logic [7:0] byte_in;
    logic frame_valid, frame_error;
    logic [7:0] flags, ttl, hops;
    logic [15:0] source, seq_num, boot_id;
    logic [31:0] timestamp_ms;
    logic signed [31:0] position_x_cm, position_y_cm, position_z_cm;
    logic signed [15:0] velocity_x_cms, velocity_y_cms, velocity_z_cms;
    reg [287:0] golden = 288'hd702010105001234abcdbeef000030390000007bfffffe3800000315ff9c00c8fed4bce9;
    integer i;

    packet_parser parser(.*);
    always #5 clk = ~clk;

    task send_packet(input logic corrupt);
        begin
            for (i = 0; i < 36; i = i + 1) begin
                @(negedge clk);
                byte_valid = 1;
                byte_in = golden[287-i*8 -: 8];
                if (corrupt && i == 20) byte_in = byte_in ^ 8'h01;
            end
            @(negedge clk);
            byte_valid = 0;
            #1;
        end
    endtask

    logic update_valid = 0;
    logic [15:0] update_source, update_sequence, update_boot_id;
    logic [31:0] update_timestamp_ms;
    logic signed [31:0] update_position_x_cm, update_position_y_cm, update_position_z_cm;
    logic signed [15:0] update_velocity_x_cms, update_velocity_y_cms, update_velocity_z_cms;
    logic update_accepted, update_rejected_old;
    logic [15:0] query_source;
    logic query_valid;
    logic [15:0] query_sequence, query_boot_id;
    logic [31:0] query_timestamp_ms;
    logic signed [31:0] query_position_x_cm, query_position_y_cm, query_position_z_cm;
    logic signed [15:0] query_velocity_x_cms, query_velocity_y_cms, query_velocity_z_cms;

    neighbor_table #(.ENTRIES(2)) dut_table(.*);

    task update(input [15:0] seq, input [15:0] boot, input signed [31:0] x);
        begin
            @(negedge clk);
            update_source = 16'h1234; update_sequence = seq; update_boot_id = boot;
            update_timestamp_ms = 32'd12345;
            update_position_x_cm = x; update_position_y_cm = -456; update_position_z_cm = 789;
            update_velocity_x_cms = -100; update_velocity_y_cms = 200; update_velocity_z_cms = -300;
            update_valid = 1;
            @(negedge clk); #1; update_valid = 0;
        end
    endtask

    initial begin
        repeat (2) @(negedge clk);
        rst = 0;
        send_packet(0);
        if (!frame_valid || frame_error || source != 16'h1234 || seq_num != 16'habcd
            || boot_id != 16'hbeef || position_x_cm != 123 || position_y_cm != -456
            || velocity_z_cms != -300) $fatal(1, "valid packet parse failed");
        send_packet(1);
        if (frame_valid || !frame_error) $fatal(1, "corrupt packet accepted");

        update(16'habcd, 16'hbeef, 123);
        if (!update_accepted) $fatal(1, "initial table update rejected");
        update(16'habcc, 16'hbeef, 999);
        if (!update_rejected_old) $fatal(1, "old sequence accepted");
        update(16'h0000, 16'hcafe, 456);
        if (!update_accepted) $fatal(1, "new boot session rejected");
        update(16'habce, 16'hbeef, 999);
        if (!update_rejected_old) $fatal(1, "retired boot session accepted");
        query_source = 16'h1234; #1;
        if (!query_valid || query_boot_id != 16'hcafe || query_sequence != 0
            || query_position_x_cm != 456) $fatal(1, "table query failed");
        $display("PASS: packet parser, CRC, state table, reboot/replay handling");
        $finish;
    end
endmodule
