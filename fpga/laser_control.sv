`timescale 1ns/1ps
// Functional fixed-point implementation. Timing/resource closure requires Moku Compile.
module laser_control (
  input wire Clk, Reset,
  input wire signed [15:0] InputA, InputB, InputC, InputD,
  input wire [31:0] Control [0:15],
  output reg [31:0] Status [0:15],
  output reg signed [15:0] OutputA, OutputB
);
  localparam [31:0] MAGIC = 32'h4c415302;
  localparam DISABLED=0, PID=1, PREVIEW=2, RAMP=3, OPEN_LOOP=4;
  localparam F_CONFIG=1, F_HEARTBEAT=2, F_SIGNAL=4,
             F_TRACKING=8, F_DURATION=16, F_ARITHMETIC=32;
  reg signed [31:0] staging[0:46], cfg[0:46];
  reg [46:0] present;
  reg [31:0] write_seen, commit_seen, commit_ack, command_seen;
  reg [31:0] heartbeat_seen, heartbeat_age, elapsed, ticks, divider;
  reg [31:0] phase;
  reg [31:0] faults;
  reg [2:0] state;
  reg configured, zeroed, first_sample;
  reg peak_ref, peak_meas;
  reg [31:0] signal_age;
  reg signed [15:0] prev_phase_a, prev_phase_b;
  reg signed [63:0] displacement;
  reg signed [31:0] measurement, reference, drive;
  reg signed [31:0] integral, derivative, prev_y, model_y, res1, res2;
  reg signed [31:0] sensor_history[0:255];
  reg [7:0] sensor_pointer;
  integer i;
  reg valid_staging;
  reg signed [31:0] da, db, period_count;
  reg signed [63:0] displacement_next;
  reg [31:0] future_phase, future_phase1;
  reg signed [63:0] r_now, rd, rd1, err, new_integral, new_derivative;
  reg signed [63:0] new_model, aligned_model, trim, new_res1, new_res2;
  reg signed [63:0] requested, bounded, next_drive, dac;
  reg arithmetic_bad;
  reg [31:0] tick_fault;
  reg [15:0] telemetry[0:15];
  reg [3:0] telemetry_index;
  reg [31:0] telemetry_divider;
  reg [15:0] telemetry_check;

  `include "sine_lut.svh"

  function automatic signed [63:0] mul24(input signed [31:0] x, y);
    reg signed [63:0] product;
    begin product = x * y; mul24 = product >>> 24; end
  endfunction
  function automatic signed [63:0] clamp64(input signed [63:0] x, lo, hi);
    begin clamp64 = x < lo ? lo : (x > hi ? hi : x); end
  endfunction
  function automatic overflow32(input signed [63:0] x);
    begin overflow32 = x > 64'sd2147483647 || x < -64'sd2147483648; end
  endfunction
  function automatic dac_range_valid(input signed [31:0] lo,hi,scale);
    reg signed [63:0] low_product,high_product;
    begin
      low_product=lo*scale; high_product=hi*scale;
      dac_range_valid=low_product>=-64'sd140733193388032 && high_product<=64'sd140733193388032;
    end
  endfunction
  function automatic signed [31:0] noise_q24(input [31:0] index,seed);
    reg [31:0] h;
    begin
      h=index+seed; h=h^(h>>16); h=h*32'h7feb352d;
      h=h^(h>>15); h=h*32'h846ca68b; h=h^(h>>16);
      noise_q24=$signed(h[31:16])*32'sd512;
    end
  endfunction
  function automatic signed [31:0] gate_q24(input [31:0] p);
    reg signed [63:0] position,duty,edge_width,one;
    begin
      position={40'd0,p[31:8]}; duty=$signed(cfg[36]); edge_width=$signed(cfg[38]); one=16777216;
      if(duty==0) gate_q24=0;
      else if(duty==one) gate_q24=one;
      else if(edge_width==0) gate_q24=position<duty ? one : 0;
      else if(position<edge_width) gate_q24=one/2+(position*one)/(2*edge_width);
      else if(position>=one-edge_width) gate_q24=((position-(one-edge_width))*one)/(2*edge_width);
      else if(position<duty-edge_width) gate_q24=one;
      else if(position<duty+edge_width) gate_q24=one-((position-(duty-edge_width))*one)/(2*edge_width);
      else gate_q24=0;
    end
  endfunction
  function automatic signed [31:0] trajectory(input [31:0] p,index);
    reg signed [63:0] result,wave,position,symmetry,one;
    begin
      position={40'd0,p[31:8]}; symmetry=$signed(cfg[37]); one=16777216;
      case(cfg[14])
        0: wave=one;
        1: wave=sine_q24(p[31:24]);
        2: wave=$signed(gate_q24(p))*64'sd2-one;
        3: begin
          if(position<symmetry) wave=-one+(2*position*one)/symmetry;
          else wave=one-(2*(position-symmetry)*one)/(one-symmetry);
        end
        4: wave=gate_q24(p);
        default: wave=noise_q24(index,cfg[46]);
      endcase
      result=$signed(cfg[16])+mul24(cfg[15],wave[31:0]);
      trajectory = clamp64(result, -64'sd2147483648, 64'sd2147483647);
    end
  endfunction
  function automatic signed [31:0] disturbance(input [31:0] index);
    reg [31:0] active_tick,p;
    reg signed [31:0] wave;
    begin
      active_tick=index-cfg[43]; p=cfg[42]+cfg[41]*active_tick;
      if(cfg[39]==0 || index<$unsigned(cfg[43]) || active_tick>=$unsigned(cfg[44])) disturbance=0;
      else begin
        case(cfg[39])
          1: wave=16777216;
          2: wave=sine_q24(p[31:24]);
          3: wave=p[31] ? -32'sd16777216 : 32'sd16777216;
          default: wave=noise_q24(active_tick,cfg[45]);
        endcase
        disturbance=mul24(cfg[40],wave);
      end
    end
  endfunction

  always @* begin
    valid_staging = (&present) && staging[0] >= 100 && staging[0] <= 31250000 &&
      staging[1] > 0 && staging[2] >= 0 && staging[3] >= 2 && staging[3] <= 65536 &&
      staging[4] > 0 && (staging[5] == 1 || staging[5] == -1) &&
      staging[6] >= -327680 && staging[6] < staging[7] && staging[7] <= 327680 &&
      staging[8] >= staging[6] && staging[8] <= staging[7] &&
      staging[9] > 0 && staging[10] > 0 && staging[11] > 0 && staging[11] <= 32767 &&
      staging[12] > 0 && staging[13] > 0 && staging[13] <= 2147418112 &&
      staging[14] >= 0 && staging[14] <= 5 && staging[15] >= 0 &&
      staging[19] >= 0 && staging[19] <= 255 && staging[20] >= 0 && staging[20] <= 255 &&
      staging[21] > 0 && staging[21] < 16777216 && staging[22] > 0 &&
      staging[23] > 0 && staging[24] > 0 && staging[25] >= 0 && staging[26] >= 0 &&
      staging[27] > 0 && staging[28] >= 0 && staging[28] <= 16777216 && staging[29] >= 0 &&
      dac_range_valid(staging[6],staging[7],staging[13]) &&
      staging[36] >= 0 && staging[36] <= 16777216 && staging[37] > 0 && staging[37] < 16777216 &&
      staging[38] >= 0 && staging[38] <= staging[36]/2 && staging[38] <= (16777216-staging[36])/2 &&
      staging[39] >= 0 && staging[39] <= 4 && staging[40] >= -327680 && staging[40] <= 327680 &&
      staging[43] >= 0 && staging[44] >= 0 && (staging[39] == 0 || staging[44] > 0) &&
      (($signed(staging[15]) + $signed({staging[16][31], staging[16]})) <= 2147483647) &&
      (($signed(staging[16]) - $signed({staging[15][31], staging[15]})) >= -2147483648);
    for (integer s=0; s<16; s=s+1) Status[s] = 0;
    Status[0]=MAGIC; Status[1]=commit_ack; Status[2]={29'd0,state}; Status[3]=faults;
    Status[4]=ticks;
    Status[5]=Control[8] < 47 ? cfg[Control[8]] : 0;
    Status[6]=measurement; Status[7]=reference; Status[8]=drive; Status[9]=res1;
    Status[10]=write_seen; Status[11]=command_seen;
    Status[12]={29'd0,zeroed,configured,(peak_ref && peak_meas)};
    Status[13]=elapsed;
    Status[14]=commit_seen;
    dac = ($signed(drive) * $signed(cfg[13])) >>> 32;
    OutputA = configured ? clamp64(dac,-32768,32767) : 0;
    // Monitor selectable without changing the configuration transaction.
    case (Control[9])
      1: OutputB = reference >>> 16;
      2: OutputB = (reference - measurement) >>> 16;
      3: OutputB = res1 >>> 16;
      4: OutputB = telemetry[telemetry_index];
      default: OutputB = measurement >>> 16;
    endcase
  end

  // One coherent sixteen-word frame per board tick. At 1 kHz this is ~16 kword/s.
  // The logger must oversample and recover held word boundaries before decoding.
  always @(posedge Clk) begin
    if (Reset) begin
      telemetry_index<=0; telemetry_divider<=0;
      for (integer t=0; t<16; t=t+1) telemetry[t]<=0;
    end else if (configured && divider==0) begin
      telemetry_index<=0; telemetry_divider<=0;
      telemetry[0]<=16'ha55a; telemetry[1]<=16'h0001;
      telemetry[2]<=ticks[31:16]; telemetry[3]<=ticks[15:0];
      telemetry[4]<=measurement[31:16]; telemetry[5]<=measurement[15:0];
      telemetry[6]<=reference[31:16]; telemetry[7]<=reference[15:0];
      telemetry[8]<=drive[31:16]; telemetry[9]<=drive[15:0];
      telemetry[10]<=res1[31:16]; telemetry[11]<=res1[15:0];
      telemetry[12]<=commit_ack[31:16]; telemetry[13]<=commit_ack[15:0];
      telemetry[14]<={7'd0,faults[5:0],state};
      telemetry_check=16'ha55a ^ 16'h0001 ^ ticks[31:16] ^ ticks[15:0] ^
        measurement[31:16] ^ measurement[15:0] ^ reference[31:16] ^ reference[15:0] ^
        drive[31:16] ^ drive[15:0] ^ res1[31:16] ^ res1[15:0] ^
        commit_ack[31:16] ^ commit_ack[15:0] ^ {7'd0,faults[5:0],state};
      telemetry[15]<=telemetry_check;
    end else if (configured) begin
      if (telemetry_divider >= cfg[0]/16-1) begin
        telemetry_divider<=0;
        if (telemetry_index<15) telemetry_index<=telemetry_index+1;
      end else telemetry_divider<=telemetry_divider+1;
    end
  end

  always @(posedge Clk) begin
    if (Reset) begin
      present<=0; write_seen<=0; commit_seen<=0; commit_ack<=0; command_seen<=0;
      heartbeat_seen<=0; heartbeat_age<=0; elapsed<=0; ticks<=0; divider<=0;
      phase<=0; faults<=0; state<=DISABLED; configured<=0; zeroed<=0;
      peak_ref<=0; peak_meas<=0; signal_age<=0; first_sample<=1;
      prev_phase_a<=0; prev_phase_b<=0; displacement<=0;
      measurement<=0; reference<=0; drive<=0; integral<=0; derivative<=0;
      prev_y<=0; model_y<=0; res1<=0; res2<=0; sensor_pointer<=0;
      for (i=0; i<47; i=i+1) begin staging[i]<=0; cfg[i]<=0; end
      for (i=0; i<256; i=i+1) sensor_history[i]<=0;
    end else begin
      prev_phase_a<=InputA; prev_phase_b<=InputB;
      if (configured) begin
        // Separate phase unwrap avoids a false wrap when A/B straddle their boundaries.
        period_count=cfg[3];
        da=$signed(InputA)-$signed({prev_phase_a[15],prev_phase_a});
        db=$signed(InputB)-$signed({prev_phase_b[15],prev_phase_b});
        if (da > period_count/2) da=da-period_count;
        else if (da < -period_count/2) da=da+period_count;
        if (db > period_count/2) db=db-period_count;
        else if (db < -period_count/2) db=db+period_count;
        displacement_next=displacement + ($signed(db-da)*$signed(cfg[4])*$signed(cfg[5]));
        displacement<=displacement_next;
        if (overflow32(displacement_next)) begin
          faults<=faults|F_ARITHMETIC; zeroed<=0;
          if (state==PID || state==PREVIEW || state==OPEN_LOOP) state<=RAMP;
        end
        if ($signed(InputC) >= cfg[11] || $signed(InputC) <= -cfg[11]) peak_ref<=1;
        if ($signed(InputD) >= cfg[11] || $signed(InputD) <= -cfg[11]) peak_meas<=1;
      end
      if (Control[0]==MAGIC && Control[3]!=write_seen) begin
        write_seen<=Control[3];
        if (Control[1]<47) begin staging[Control[1]]<=Control[2]; present[Control[1]]<=1; end
        else faults<=faults|F_CONFIG;
      end
      if (Control[7]!=heartbeat_seen) begin heartbeat_seen<=Control[7]; heartbeat_age<=0; end
      if (divider >= (configured ? cfg[0]-1 : 31249)) begin
        divider<=0; ticks<=ticks+1;
        if (heartbeat_age != 32'hffffffff && Control[7]==heartbeat_seen) heartbeat_age<=heartbeat_age+1;
        if (configured) begin
          measurement<=displacement[31:0];
          if (peak_ref && peak_meas) signal_age<=0;
          else if (signal_age != 32'hffffffff) signal_age<=signal_age+1;
        end
        peak_ref<=0; peak_meas<=0;

        if (Control[0]==MAGIC && Control[4]!=commit_seen) begin
          commit_seen<=Control[4];
          if (state==DISABLED && valid_staging && Control[4]!=0) begin
            for (i=0; i<47; i=i+1) cfg[i]<=staging[i];
            commit_ack<=Control[4]; configured<=1; present<=0; faults<=0;
            zeroed<=0; displacement<=0; signal_age<=32'hffffffff;
            drive<=staging[8];
          end else faults<=faults|F_CONFIG;
        end

        if (Control[0]==MAGIC && Control[5]!=command_seen) begin
          command_seen<=Control[5];
          if (Control[6]==0) begin
            if (state!=DISABLED) state<=RAMP;
          end else if (Control[6]==3 && state==DISABLED && configured && signal_age==0) begin
            displacement<=0; measurement<=0; zeroed<=1; faults<=0;
          end else if ((Control[6]==1 || Control[6]==2 || Control[6]==4) && state==DISABLED &&
                       configured && zeroed && faults==0 && signal_age==0) begin
            state<=Control[6][2:0]; phase<=cfg[18]; elapsed<=0; heartbeat_age<=0;
            integral<=0; derivative<=0; prev_y<=0; model_y<=0; res1<=0; res2<=0;
            first_sample<=1; sensor_pointer<=0;
            for (i=0; i<256; i=i+1) sensor_history[i]<=0;
          end else faults<=faults|F_CONFIG;
        end

        if (state==RAMP) begin
          next_drive=$signed(drive)+clamp64($signed(cfg[8])-$signed({drive[31],drive}),-$signed({cfg[9][31],cfg[9]}),$signed(cfg[9]));
          drive<=next_drive[31:0];
          if (next_drive==cfg[8]) state<=DISABLED;
        end else if (state==PID || state==PREVIEW || state==OPEN_LOOP) begin
          tick_fault=0; arithmetic_bad=0;
          elapsed<=elapsed+1;
          if (elapsed>=cfg[1]) tick_fault=tick_fault|F_DURATION;
          if (cfg[2]>0 && heartbeat_age>=cfg[2]) tick_fault=tick_fault|F_HEARTBEAT;
          if (signal_age>=cfg[12]) tick_fault=tick_fault|F_SIGNAL;
          r_now=trajectory(phase,elapsed); reference<=r_now[31:0];
          err=r_now-$signed(displacement);
          if (err>cfg[10] || err<-$signed({cfg[10][31],cfg[10]})) tick_fault=tick_fault|F_TRACKING;
          if (overflow32(err) || overflow32(displacement)) arithmetic_bad=1;
          phase<=phase+cfg[17];

          if (state==OPEN_LOOP) begin
            requested=mul24(r_now[31:0],cfg[24]);
          end else if (state==PID) begin
            new_integral=clamp64($signed(integral)+mul24(err[31:0],cfg[26]),-$signed({cfg[27][31],cfg[27]}),$signed(cfg[27]));
            trim=$signed(prev_y)-$signed(displacement);
            if (overflow32(trim)) arithmetic_bad=1;
            new_derivative=first_sample ? 0 : mul24(derivative,cfg[28])+mul24(trim[31:0],cfg[29]);
            requested=mul24(err[31:0],cfg[25])+new_integral+new_derivative;
            integral<=new_integral[31:0]; derivative<=new_derivative[31:0];
            if (overflow32(new_derivative)) arithmetic_bad=1;
          end else begin
            future_phase=phase+(cfg[17]*cfg[19]); future_phase1=future_phase+cfg[17];
            rd=trajectory(future_phase,elapsed+cfg[19]); rd1=trajectory(future_phase1,elapsed+cfg[19]+1);
            trim=rd1-mul24(rd[31:0],cfg[21]);
            if (overflow32(trim)) arithmetic_bad=1;
            requested=mul24(trim[31:0],cfg[23]);
            new_model=mul24(model_y,cfg[21])+mul24(drive,cfg[22]);
            aligned_model=cfg[20]==0 ? new_model : $signed(sensor_history[sensor_pointer]);
            sensor_history[sensor_pointer]<=new_model[31:0];
            if (sensor_pointer+1 >= cfg[20]) sensor_pointer<=0;
            else sensor_pointer<=sensor_pointer+1;
            trim=aligned_model-$signed(displacement);
            new_res1=mul24(res1,cfg[30])+mul24(res2,cfg[31])+mul24(trim[31:0],cfg[34]);
            new_res2=mul24(res1,cfg[32])+mul24(res2,cfg[33])+mul24(trim[31:0],cfg[35]);
            requested=requested+mul24(new_res1[31:0],cfg[24]);
            model_y<=new_model[31:0]; res1<=new_res1[31:0]; res2<=new_res2[31:0];
            if (overflow32(new_model) || overflow32(trim) || overflow32(new_res1) || overflow32(new_res2)) arithmetic_bad=1;
          end
          first_sample<=0; prev_y<=displacement[31:0];
          requested=requested+$signed(disturbance(elapsed));
          if (overflow32(requested)) arithmetic_bad=1;
          if (arithmetic_bad) tick_fault=tick_fault|F_ARITHMETIC;
          bounded=clamp64(requested,$signed(cfg[6]),$signed(cfg[7]));
          next_drive=$signed(drive)+clamp64(bounded-$signed(drive),-$signed({cfg[9][31],cfg[9]}),$signed(cfg[9]));
          if (tick_fault!=0) begin faults<=faults|tick_fault; state<=RAMP; end
          else drive<=next_drive[31:0];
        end
      end else divider<=divider+1;
    end
  end
endmodule
