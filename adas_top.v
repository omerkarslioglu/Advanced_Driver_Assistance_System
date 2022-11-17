module adas_top
(
    // default inputs
    input clk;
    input rst_n;

    // from car
    input timer_trick_i;
    input mode_i;                   // (1) auto mode - (0) assist mode
    input [1:0] redlight_i;         // (1) lidar - (0) camera
    input [1:0] crosswall_i;
    input [7:0] distance_lidar_i;
    input [7:0] distance_cam_i;
    input [7:0] speed_measured_i;

    // from user
    input [7:0] following_distance_i;
    input [7:0] speed_i;

    // autonom
    output gas_o;
    output brake_o;

    // assistance
    output redlight_o;
    output crosswall_o;
    output following_distance_o;
);

localparam assistance = 1'b0;
localparam autonomous = 1'b1;

reg gas_r;
reg brake_r;
reg [7:0] speed_buff;
reg [7:0] following_distance_buff;
reg [7:0] sensor_data_avg_r;
reg [7:0] sensor_data_flow [2:0];

//wire [1:0] sensor_data;

//assign sensor_data = {redlight_i, };
assign gas_o = gas_r;
assign brake_o = brake_r;

// gas&brake controller
always(posedge clk) begin
    if(~rst_n) begin
        /* autonomous */
        gas_r <= 1'b0;
        brake_r <= 1'b0;
    end
    else if(timer_trick_i) begin
        case(mode_i)
            autonomous: begin
                if(speed_measured_i > speed_buff) begin
                    gas_r <= 1'b0;
                    brake_r <= 1'b1; 
                end
                else if(speed_measured_i < speed_buff) begin
                    gas_r <= 1'b1;
                    brake_r <= 1'b0; 
                end
                else if(speed_measured_i == speed_buff) begin
                    gas_r <= 1'b1;
                    brake_r <= 1'b1;     
                end
            end
            assistance: begin 
                gas_r <= 1'b0;
                brake_r <= 1'b0;
            end
        endcase

    end
end

/* speed & following_distance that will be using in auto mode */
always(posedge clk) begin
    if(~rst_n) begin
        speed_buff <= 'd100; 
        following_distance_buff <= 'd50;
    end
    else if(timer_trick_i && mode_i) begin
        /* speed */
        case({redlight_i, crosswall_i})
            4'b1100: begin
                speed_buff <= 8'b0;
            end
            4'b1111: begin
                speed_buff <= 8'b0;
            end
            4'b0011: begin
                speed_buff <= 'd20;
            end
            default: begin 
                /* state equals 4'b0000 and other states */
                if(speed_i == 8'b0) begin
                    speed_buff <= 'd100; 
                end
                else begin
                    speed_buff <= speed_i; 
                end
            end
        endcase

        /* following distance */
        if(following_distance_i == 8'b0) begin
            following_distance_buff <= 'd50; 
        end
        else begin
            following_distance_buff <= following_distance_i; 
        end
    end
    else begin
        speed_buff <= speed_buff; 
        following_distance_buff <= following_distance_buff;
    end
end

/* following_measured_distance */
always(posedge clk) begin
    if(~rst_n) begin

    end
    else if(timer_trick_i) begin
        if()
        sensor_data_avg_r <= (distance_cam_i + distance_lidar_i) / 2; 
        sensor_data_flow[0] <= sensor_data_avg_r
    end
    else begin

    end
end


endmodule