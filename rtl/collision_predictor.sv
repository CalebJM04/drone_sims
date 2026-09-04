module collision_predictor (
    input  logic signed [31:0] pa_x_cm, pa_y_cm, pa_z_cm,
    input  logic signed [31:0] pb_x_cm, pb_y_cm, pb_z_cm,
    input  logic signed [15:0] va_x_cms, va_y_cms, va_z_cms,
    input  logic signed [15:0] vb_x_cms, vb_y_cms, vb_z_cms,
    input  logic        [15:0] safety_cm,
    input  logic        [31:0] horizon_ms,
    output logic               risk,
    output logic        [31:0] tcpa_ms,
    output logic               tcpa_valid
);
    logic signed [63:0] prx, pry, prz, vrx, vry, vrz;
    logic signed [63:0] dot_product, speed_squared, t_ms;
    logic signed [63:0] closest_x, closest_y, closest_z;
    logic [127:0] closest_squared, safety_squared_scaled;

    always_comb begin
        prx = $signed(pa_x_cm) - $signed(pb_x_cm);
        pry = $signed(pa_y_cm) - $signed(pb_y_cm);
        prz = $signed(pa_z_cm) - $signed(pb_z_cm);
        vrx = $signed(va_x_cms) - $signed(vb_x_cms);
        vry = $signed(va_y_cms) - $signed(vb_y_cms);
        vrz = $signed(va_z_cms) - $signed(vb_z_cms);
        dot_product = prx * vrx + pry * vry + prz * vrz;
        speed_squared = vrx * vrx + vry * vry + vrz * vrz;
        risk = 1'b0;
        tcpa_ms = 32'd0;
        tcpa_valid = 1'b0;
        t_ms = 64'sd0;
        closest_x = prx * 64'sd1000;
        closest_y = pry * 64'sd1000;
        closest_z = prz * 64'sd1000;
        closest_squared = closest_x * closest_x + closest_y * closest_y + closest_z * closest_z;
        safety_squared_scaled = safety_cm * safety_cm * 128'd1000000;

        if (speed_squared == 0) begin
            risk = closest_squared < safety_squared_scaled;
        end else if (dot_product < 0) begin
            t_ms = ((-dot_product) * 64'sd1000 + speed_squared / 2) / speed_squared;
            tcpa_ms = t_ms[31:0];
            tcpa_valid = 1'b1;
            closest_x = prx * 64'sd1000 + vrx * t_ms;
            closest_y = pry * 64'sd1000 + vry * t_ms;
            closest_z = prz * 64'sd1000 + vrz * t_ms;
            closest_squared = closest_x * closest_x + closest_y * closest_y + closest_z * closest_z;
            risk = (t_ms <= $signed({1'b0, horizon_ms})) && (closest_squared < safety_squared_scaled);
        end
    end
endmodule

