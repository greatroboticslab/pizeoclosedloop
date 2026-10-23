`timescale 1ns/1ps
module testbench;
  reg Clk=0, Reset=1;
  always #5 Clk=~Clk;
  reg signed [15:0] a=0,b=0,c=1000,d=1000;
  reg [31:0] controls[0:15];
  wire [31:0] status[0:15];
  wire signed [15:0] out_a,out_b;
  reg [31:0] words[0:46];
  reg [127:0] vectors[0:2047];
  reg [15:0] captured[0:15];
  string config_file,vector_file;
  integer i,j,n,algorithm,seq=0;
  reg [31:0] old_tick;
  laser_control dut(.Clk(Clk),.Reset(Reset),.InputA(a),.InputB(b),.InputC(c),.InputD(d),
    .Control(controls),.Status(status),.OutputA(out_a),.OutputB(out_b));
  task tick;
    begin
      old_tick=status[4];
      while(status[4]===old_tick) @(negedge Clk);
    end
  endtask
  task stage(input integer address,input [31:0] value);
    begin
      @(negedge Clk); controls[1]=address; controls[2]=value;
      @(negedge Clk); seq=seq+1; controls[3]=seq;
      @(negedge Clk);
      if(status[10]!=seq) $fatal(1,"write was not acknowledged");
    end
  endtask
  task command(input integer value);
    begin
      @(negedge Clk); controls[6]=value; controls[5]=controls[5]+1;
      tick();
      if(status[11]!=controls[5]) $fatal(1,"command not acknowledged");
    end
  endtask
  initial begin
    if (!$value$plusargs("config=%s",config_file)) $fatal(1,"config required");
    if (!$value$plusargs("vectors=%s",vector_file)) $fatal(1,"vectors required");
    if (!$value$plusargs("algorithm=%d",algorithm)) algorithm=1;
    if (!$value$plusargs("count=%d",n)) n=10;
    $readmemh(config_file,words);
    $readmemh(vector_file,vectors,0,n-1);
    for(i=0;i<16;i=i+1) controls[i]=0;
    repeat(3) @(negedge Clk); Reset=0;
    if(out_a!==0 || status[2]!==0) $fatal(1,"unsafe boot");
    controls[0]=32'h4c415302;
    stage(0,words[0]); controls[4]=1; tick();
    if(status[1]!=0 || !(status[3]&1)) $fatal(1,"partial commit accepted");
    for(i=0;i<47;i=i+1) stage(i,words[i]);
    controls[4]=2; tick();
    if(status[1]!=2) $fatal(1,"complete commit rejected");
    repeat(3) tick();
    command(3);
    if(!(status[12]&4)) $fatal(1,"zero failed");
    controls[9]=4;
    command(algorithm);
    if(status[2]!=algorithm) $fatal(1,"start rejected: %h",status[3]);
    for(j=0;j<n;j=j+1) begin
      b=vectors[j][111:96];
      tick();
      if(status[8]!==vectors[j][95:64])
        $fatal(1,"numeric mismatch step %0d got %0d expected %0d faults %0d",j,$signed(status[8]),$signed(vectors[j][95:64]),status[3]);
      if(status[7]!==vectors[j][63:32]) $fatal(1,"trajectory mismatch at step %d",j);
      if(dut.disturbance(j)!==vectors[j][31:0]) $fatal(1,"disturbance mismatch at step %d",j);
      repeat(2) @(negedge Clk);
      for(i=0;i<16;i=i+1) begin
        while(dut.telemetry_index!=i) @(negedge Clk);
        captured[i]=out_b;
      end
      $write("FRAME");
      for(i=0;i<16;i=i+1) $write(" %h",captured[i]);
      $display("");
    end
    // Autonomous completion, without any heartbeat or subsequent host commands.
    for(j=0;j<words[1]+20;j=j+1) tick();
    if(status[2]!=0 || status[8]!=words[8] || !(status[3]&16))
      $fatal(1,"duration stop did not ramp safe: state %d fault %d",status[2],status[3]);
    command(3); command(algorithm);
    c=0; d=0;
    repeat(20) tick();
    if(status[2]!=0 || !(status[3]&4)) $fatal(1,"signal loss failed to stop");
    c=1000; d=1000; a=0; b=0; repeat(3) tick();
    command(3);
    // Common-mode phase wraps must cancel, including staggered A/B transitions.
    a=32760; b=32760; tick();
    if($signed(status[6])!=0) $fatal(1,"common mode phase leaked into displacement");
    a=-32760; tick();
    if($signed(status[6])!=-16*65536) $fatal(1,"A phase unwrap incorrect: %d",$signed(status[6]));
    b=-32760; tick();
    if($signed(status[6])!=0) $fatal(1,"B phase unwrap incorrect");
    command(3); command(algorithm); tick(); command(0);
    repeat(20) tick();
    if(status[2]!=0 || status[8]!=words[8] || status[3]!=0)
      $fatal(1,"explicit stop did not ramp safe");
    // Enable the optional heartbeat policy; otherwise experiments are autonomous.
    words[2]=3;
    for(i=0;i<47;i=i+1) stage(i,words[i]);
    controls[4]=3; tick(); repeat(3) tick(); command(3); command(algorithm);
    repeat(20) tick();
    if(status[2]!=0 || !(status[3]&2)) $fatal(1,"optional heartbeat watchdog failed");
    // Staged invalid limits never replace an acknowledged live configuration.
    words[6]=6*65536;
    for(i=0;i<47;i=i+1) stage(i,words[i]);
    controls[4]=4; tick();
    if(status[1]!=3 || !(status[3]&1)) $fatal(1,"invalid limits were accepted");
    // Overflowing the displacement representation invalidates zero and refuses start.
    a=0; b=0; tick(); command(3);
    b=32760; tick(); b=-32760; tick();
    if(!(status[3]&32) || (status[12]&4)) $fatal(1,"displacement overflow was not latched");
    command(algorithm);
    if(status[2]!=0) $fatal(1,"start accepted after arithmetic overflow");
    a=0; b=0; repeat(3) tick(); command(3); command(algorithm); tick();
    if(status[2]!=algorithm) $fatal(1,"could not recover zero before reset test");
    Reset=1; repeat(3) @(negedge Clk); Reset=0;
    if(out_a!==0 || status[2]!==0 || status[1]!==0 || status[12]!==0)
      $fatal(1,"reset during control did not disable and clear configuration");
    $display("PASS: numeric vectors, phase unwrap, transactional config and autonomous protections");
    $finish;
  end
  initial begin #100000000; $fatal(1,"simulation timeout"); end
endmodule
