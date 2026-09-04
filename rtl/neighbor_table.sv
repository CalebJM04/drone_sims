module neighbor_table #(
    parameter ENTRIES = 8
) (
    input logic clk,
    input logic rst,
    input logic update_valid,
    input logic [15:0] update_source,
    input logic [15:0] update_sequence,
    input logic [15:0] update_boot_id,
    input logic [31:0] update_timestamp_ms,
    input logic signed [31:0] update_position_x_cm,
    input logic signed [31:0] update_position_y_cm,
    input logic signed [31:0] update_position_z_cm,
    input logic signed [15:0] update_velocity_x_cms,
    input logic signed [15:0] update_velocity_y_cms,
    input logic signed [15:0] update_velocity_z_cms,
    output logic update_accepted,
    output logic update_rejected_old,
    input logic [15:0] query_source,
    output logic query_valid,
    output logic [15:0] query_sequence,
    output logic [15:0] query_boot_id,
    output logic [31:0] query_timestamp_ms,
    output logic signed [31:0] query_position_x_cm,
    output logic signed [31:0] query_position_y_cm,
    output logic signed [31:0] query_position_z_cm,
    output logic signed [15:0] query_velocity_x_cms,
    output logic signed [15:0] query_velocity_y_cms,
    output logic signed [15:0] query_velocity_z_cms
);
    logic valid [0:ENTRIES-1];
    logic [15:0] source [0:ENTRIES-1], seq_num [0:ENTRIES-1], boot_id [0:ENTRIES-1];
    logic [15:0] retired_boot [0:ENTRIES-1];
    logic retired_valid [0:ENTRIES-1];
    logic [31:0] timestamp_ms [0:ENTRIES-1], age [0:ENTRIES-1];
    logic signed [31:0] px [0:ENTRIES-1], py [0:ENTRIES-1], pz [0:ENTRIES-1];
    logic signed [15:0] vx [0:ENTRIES-1], vy [0:ENTRIES-1], vz [0:ENTRIES-1];
    logic [31:0] clock;
    integer i, selected;
    logic [15:0] difference;

    always_comb begin
        query_valid = 0;
        query_sequence = 0; query_boot_id = 0; query_timestamp_ms = 0;
        query_position_x_cm = 0; query_position_y_cm = 0; query_position_z_cm = 0;
        query_velocity_x_cms = 0; query_velocity_y_cms = 0; query_velocity_z_cms = 0;
        for (i = 0; i < ENTRIES; i = i + 1) begin
            if (valid[i] && source[i] == query_source) begin
                query_valid = 1;
                query_sequence = seq_num[i]; query_boot_id = boot_id[i];
                query_timestamp_ms = timestamp_ms[i];
                query_position_x_cm = px[i]; query_position_y_cm = py[i]; query_position_z_cm = pz[i];
                query_velocity_x_cms = vx[i]; query_velocity_y_cms = vy[i]; query_velocity_z_cms = vz[i];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            clock <= 0;
            update_accepted <= 0;
            update_rejected_old <= 0;
            for (i = 0; i < ENTRIES; i = i + 1) begin
                valid[i] <= 0;
                retired_valid[i] <= 0;
                age[i] <= 0;
            end
        end else begin
            clock <= clock + 1'b1;
            update_accepted <= 0;
            update_rejected_old <= 0;
            if (update_valid) begin
                selected = -1;
                for (i = 0; i < ENTRIES; i = i + 1)
                    if (valid[i] && source[i] == update_source) selected = i;
                if (selected >= 0) begin
                    difference = update_sequence - seq_num[selected];
                    if ((update_boot_id == boot_id[selected]
                         && (difference == 0 || difference[15]))
                        || (retired_valid[selected] && update_boot_id == retired_boot[selected])) begin
                        update_rejected_old <= 1;
                    end else begin
                        if (update_boot_id != boot_id[selected]) begin
                            retired_boot[selected] <= boot_id[selected];
                            retired_valid[selected] <= 1;
                        end
                        seq_num[selected] <= update_sequence;
                        boot_id[selected] <= update_boot_id;
                        timestamp_ms[selected] <= update_timestamp_ms;
                        px[selected] <= update_position_x_cm; py[selected] <= update_position_y_cm; pz[selected] <= update_position_z_cm;
                        vx[selected] <= update_velocity_x_cms; vy[selected] <= update_velocity_y_cms; vz[selected] <= update_velocity_z_cms;
                        age[selected] <= clock;
                        update_accepted <= 1;
                    end
                end else begin
                    selected = -1;
                    for (i = 0; i < ENTRIES; i = i + 1)
                        if (!valid[i] && selected < 0) selected = i;
                    if (selected < 0) begin
                        selected = 0;
                        for (i = 1; i < ENTRIES; i = i + 1)
                            if (age[i] < age[selected]) selected = i;
                    end
                    valid[selected] <= 1;
                    source[selected] <= update_source;
                    seq_num[selected] <= update_sequence;
                    boot_id[selected] <= update_boot_id;
                    retired_valid[selected] <= 0;
                    timestamp_ms[selected] <= update_timestamp_ms;
                    px[selected] <= update_position_x_cm; py[selected] <= update_position_y_cm; pz[selected] <= update_position_z_cm;
                    vx[selected] <= update_velocity_x_cms; vy[selected] <= update_velocity_y_cms; vz[selected] <= update_velocity_z_cms;
                    age[selected] <= clock;
                    update_accepted <= 1;
                end
            end
        end
    end
endmodule
