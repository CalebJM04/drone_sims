module packet_parser (
    input  logic clk,
    input  logic rst,
    input  logic byte_valid,
    input  logic [7:0] byte_in,
    output logic frame_valid,
    output logic frame_error,
    output logic [7:0] flags,
    output logic [7:0] ttl,
    output logic [7:0] hops,
    output logic [15:0] source,
    output logic [15:0] seq_num,
    output logic [15:0] boot_id,
    output logic [31:0] timestamp_ms,
    output logic signed [31:0] position_x_cm,
    output logic signed [31:0] position_y_cm,
    output logic signed [31:0] position_z_cm,
    output logic signed [15:0] velocity_x_cms,
    output logic signed [15:0] velocity_y_cms,
    output logic signed [15:0] velocity_z_cms
);
    logic [7:0] bytes [0:33];
    logic [7:0] crc_high;
    logic [15:0] crc;
    logic [5:0] count;
    integer i;

    function automatic [15:0] crc_byte(input [15:0] prior, input [7:0] data);
        integer bit_index;
        reg [15:0] value;
        begin
            value = prior ^ {data, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                value = value[15] ? (value << 1) ^ 16'h1021 : value << 1;
            crc_byte = value;
        end
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            count <= 0;
            crc <= 16'hffff;
            frame_valid <= 0;
            frame_error <= 0;
            for (i = 0; i < 34; i = i + 1)
                bytes[i] <= 0;
        end else begin
            frame_valid <= 0;
            frame_error <= 0;
            if (byte_valid) begin
                if (count < 34) begin
                    bytes[count] <= byte_in;
                    crc <= crc_byte(crc, byte_in);
                    count <= count + 1'b1;
                end else if (count == 34) begin
                    crc_high <= byte_in;
                    count <= 35;
                end else begin
                    count <= 0;
                    if (crc == {crc_high, byte_in}
                        && bytes[0] == 8'hd7 && bytes[1] == 8'h02
                        && bytes[2] == 8'h01) begin
                        flags <= bytes[3];
                        ttl <= bytes[4];
                        hops <= bytes[5];
                        source <= {bytes[6], bytes[7]};
                        seq_num <= {bytes[8], bytes[9]};
                        boot_id <= {bytes[10], bytes[11]};
                        timestamp_ms <= {bytes[12], bytes[13], bytes[14], bytes[15]};
                        position_x_cm <= $signed({bytes[16], bytes[17], bytes[18], bytes[19]});
                        position_y_cm <= $signed({bytes[20], bytes[21], bytes[22], bytes[23]});
                        position_z_cm <= $signed({bytes[24], bytes[25], bytes[26], bytes[27]});
                        velocity_x_cms <= $signed({bytes[28], bytes[29]});
                        velocity_y_cms <= $signed({bytes[30], bytes[31]});
                        velocity_z_cms <= $signed({bytes[32], bytes[33]});
                        frame_valid <= 1;
                    end else begin
                        frame_error <= 1;
                    end
                    crc <= 16'hffff;
                end
            end
        end
    end
endmodule
