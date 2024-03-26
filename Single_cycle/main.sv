module main(input logic clk,reset);
	logic [31:0] pcnext,add,instr,pca,wbpc;
	logic signed [31:0] a,b,aluresult,newb,newa,immgen,rdata,regwb;
	logic [2:0] br_type,ImmControl;
	logic [3:0] alusignal;
	logic [1:0] wbsel;
	logic regwrite,memwr,memr,selb,cs,brtaken,sel_a,brreq,jmpreq;

	controller Controller(clk,reset,instr[6:0],instr[30],instr[14:12],regwrite,memwr,memr,selb,ImmControl,wbsel,cs,sel_a,brreq,jmpreq,br_type,alusignal);
	
	m2 PCSelect(pcnext,aluresult,brtaken,pca);
	pc PC(clk,reset,pca,add);
	pcnext PCPlus4(add,pcnext);
	instrmem InstructionMemory(add,instr);
	regfile RegisterFile(clk,regwrite,instr[19:15],instr[24:20],instr[11:7],regwb,a,b);
	immediate ImmGen(instr,ImmControl,immgen);
	m2 regaselect(a,add,sel_a,newa);
	m2 ImmSelect(immgen,b,selb,newb);
	alu ALU(newa,newb,alusignal,aluresult);
	branchcond BranCon(a,b,br_type,brreq,jmpreq,brtaken);
	datamem DataMemory(aluresult,b,instr[14:12],clk,memwr,memr,cs,rdata);
	pcnext PCPlus4_2(add,wbpc);
	m3 WriteBackSelect(aluresult,rdata,wbpc,wbsel,regwb);	
	
endmodule

module m3(input logic signed [31:0] a,b,c, input logic [1:0] control, output logic signed [31:0] result);
	always_comb begin
		if (control==2'b01) result = b;
		else if (control == 2'b00) result = a;
		else if (control == 2'b11) result = c;
	end
endmodule

module branchcond(input logic [31:0] a,b, input logic [2:0] br_type, input logic brreq,jmpreq, output logic brtaken);
	logic [32:0] suba;
	logic cmpnotzero, neg, overflow;
	logic brel;
	
	assign suba = {1'b0,a} - {1'b0,b};
	assign cmpnotzero =| suba[31:0];
	assign neg = suba[31];
	assign overflow = (neg & ~a[31] & b[31]) | (~neg & a[31] & ~b[31]);
	
	always_comb begin
	case(br_type)
		3'b000: brel = ~cmpnotzero; //equals
		3'b001: brel = cmpnotzero; //notequals
		3'b010: brel = (neg ^ overflow); //less than
		3'b100: brel = suba[31]; //less than unsigned
		3'b011: brel = ~(neg ^ overflow); //greater than or equal unsigned
		3'b111: brel = ~suba[31]; //greater of equal to signed
		default: brel = 0;
	endcase end
	
	assign brtaken = (jmpreq) | (brel & brreq);
endmodule

module immediate(input logic [31:0] instr, input logic [2:0] control, output logic signed [31:0] immgen);
	always_comb begin
            case (control)
                3'b000 : immgen = {{20{instr[31]}}, instr[31:20]}; //I type 
                3'b001 : immgen = {{20{instr[31]}}, instr[31:25], instr[11:7]}; //S type 
				3'b011 : immgen = {{12{instr[31]}},instr[19:12],instr[20],instr[30:21],1'b0}; //jal
				3'b100 : immgen = {instr[31:12], 12'b0}; //lui-auipc
				3'b101 : immgen = {{20{instr[31]}},instr[7],instr[30:25],instr[11:8],1'b0}; //btype
				default: immgen = 31'd0;
			endcase
	end
endmodule

module m2(input logic signed [31:0] a,b, input logic control, output logic signed [31:0] result);
	always_comb begin
		if (control) result = b;
		else result = a;
	end
endmodule

module pc (input clk, reset, input logic [31:0] pcnext, output logic [31:0] add);
	always@(posedge clk) begin
		if (reset) add <= 'b0;
		else add <= pcnext;
	end
endmodule

module pcnext(input logic[31:0] oldadd, output logic[31:0] pcnext);
	always_comb begin
		pcnext <= oldadd + 4;
	end
endmodule

module instrmem(input logic [31:0] add, output logic [31:0] instr);
	logic [31:0] rega [0:31];
	initial begin
		$readmemh("C:/exp7codes.txt",rega);
	end
	assign instr = rega[add>>2];
endmodule

module regfile(input logic clk, regwrite, input logic [4:0] rs1,rs2,rd, input logic signed [31:0] data, output logic signed [31:0] a,b);
	logic [31:0] regb [0:31];
	logic rs1valid,rs2valid,wrvalid;
	
	assign rs1valid = |rs1;
	assign rs2valid = |rs2;
	assign wrvalid = |rd;
	
	assign a = (rs1valid) ? regb[rs1] : '0;
	assign b = (rs2valid) ? regb[rs2] : '0;
	
	initial begin
		regb[0][31:0] = 'b0;
	end
	
	always @(negedge clk) begin
		if (regwrite) begin
			if (wrvalid) regb[rd] <= data;
		end
	end
endmodule

module alu(input logic signed [31:0] r1,r2, input logic [3:0] alucontrol, output logic signed [31:0] ALUresult);
	always_comb begin
		case(alucontrol) 
			0: ALUresult <= r1 + r2; 
			1: ALUresult <= r1 - r2;
			2: ALUresult <= r1 << r2[4:0];
			3: ALUresult <= $signed(r1) < $signed(r2);
			4: ALUresult <= r1<r2;
			5: ALUresult <= r1^r2;
			6: ALUresult <= r1>>r2[4:0];
			7: ALUresult <= r1>>>r2[4:0];
			8: ALUresult <= r1|r2;
			9: ALUresult <= r1&r2;
			10: ALUresult <= r2; //lui
			default: ALUresult <= 'b0;
		endcase
	end
endmodule

module datamem(input logic [31:0] addr, wdata, input logic [2:0] f3, input logic clk,wren,rden,cs,output logic [31:0] rdata);
	logic [31:0] mem [0:31];
	logic [31:0] readdata,writedata;
	logic [3:0] mask;
	
	//load operation
	always_comb begin
		if (~cs & rden) readdata <= mem[addr[31:2]]; else readdata<='0;  //asynchronous read
	end
	
	always_comb begin  //reading according to if b/hw/w -- manual
		if (rden) begin
			case(f3) 
			3'b000: begin case(addr[1:0]) 2'b00 : rdata = {24'b0, readdata[7:0]};
									2'b01 : rdata = {24'b0, readdata[15:8]};
									2'b10 : rdata = {24'b0, readdata[23:16]};
									2'b11 : rdata = {24'b0, readdata[31:24]}; 
									default : rdata = '0; endcase end //byte signed
									
			3'b001: begin case(addr[1]) 	1'b0 : rdata = {{16{readdata[15]}}, readdata[15:0]};
									1'b1 : rdata = {{16{readdata[31]}}, readdata[31:16]};
									default : rdata = '0;	endcase	end //halfword signed
			
			3'b100: begin case(addr[1:0])	2'b00 : rdata = {{24{readdata[7]}},readdata[7:0]};
									2'b01 : rdata = {{24{readdata[15]}},readdata[15:8]};
									2'b10 : rdata = {{24{readdata[23]}},readdata[23:16]};
									2'b11 : rdata = {{24{readdata[31]}},readdata[31:24]};
									default : rdata = '0; endcase end//byte unsigned
									
			3'b010: rdata = readdata;	//word
			
			3'b101: begin case(addr[1]) 
									1'b0 : rdata = {16'b0, readdata[15:0]};
									1'b1 : rdata = {16'b0, readdata[31:16]};
									default : rdata = '0; endcase  end	//halfword unsigned
			
			default rdata = '0;
			
			endcase
		end
	end
	
	always_comb begin //generating masks and getting writedata from rs2
		if (wren) begin
			case(f3)
			3'b000: begin case(addr[1:0]) 2'b00 : begin mask = 4'b0001; writedata[7:0]   = wdata[7:0];   end
									2'b01 : begin mask = 4'b0010; writedata[15:8]  = wdata[15:8];  end
									2'b10 : begin mask = 4'b0100; writedata[23:16] = wdata[23:16]; end
									2'b11 : begin mask = 4'b1000; writedata[31:24] = wdata[31:24]; end
									default: begin mask=4'b0000; writedata=0; end endcase end //byte
									
			3'b001: begin case(addr[1]) 1'b0 : begin mask = 4'b0011; writedata[15:0]=wdata[15:0]; end
										1'b1 : begin mask = 4'b1100; writedata[31:16]=wdata[31:16]; end
										default: begin 	mask = 4'b0000; writedata = wdata; end endcase end //halfword
										
			3'b010: begin mask = 4'b1111; writedata = wdata; end //word
			
			endcase
		end
	end
	
	always_ff@(negedge clk) begin 
		if (wren & ~cs)	begin 
			if (mask[0]) mem[addr[31:2]][7:0]   <= writedata[7:0];
			if (mask[1]) mem[addr[31:2]][15:8]  <= writedata[15:8];
			if (mask[2]) mem[addr[31:2]][23:16] <= writedata[23:16];
			if (mask[3]) mem[addr[31:2]][31:24] <= writedata[31:24];
		end
	end
endmodule

module controller(input logic clk,reset,input logic [6:0]instr,input logic funct7,input logic [2:0] f3, output logic regwrite,memwr,memr,selb, output logic [2:0] ImmControl, output logic [1:0] wbsel, output logic csa, sel_a, brreq,jmpreq, output logic [2:0] brtype, output logic [3:0] ALUcontrol);
	always_comb begin
		csa = 1;
		case(instr) 
			7'b0110011: begin regwrite = 1; selb = 1; ImmControl = 3'b000; memwr = 1'b0; memr = 1'b0; wbsel = 2'b00; csa=1; sel_a=1'b0;brtype = 1'b0; jmpreq = 1'b0; brreq = 1'b0; //rtype
			if (f3==3'b000) begin
				if (funct7)   //looking at 5th bit only
					ALUcontrol <= 4'b0001;  //sub
				else ALUcontrol <= 4'b0000; end //add 
			else if (f3==3'b001) ALUcontrol <= 4'b0010; //sll
			else if (f3==3'b010) ALUcontrol <= 4'b0011; //slt
			else if (f3==3'b011) ALUcontrol <= 4'b0100; //sltu
			else if (f3==3'b100) ALUcontrol <= 4'b0101;  //xor
			else if (f3==3'b101) begin if (funct7) ALUcontrol <= 4'b0111; else ALUcontrol <= 4'b0110; end
			else if (f3==3'b110) ALUcontrol <= 4'b1000;
			else if (f3==3'b111) ALUcontrol <= 4'b1001;
			else ALUcontrol <=3'b000;  //base case
			end

			7'b0100011: begin ALUcontrol <= 4'b0000; regwrite = 0; memwr = 1'b1; memr = 1'b0; selb = 0;ImmControl = 3'b001;wbsel =2'b00;csa=1'b0; sel_a=1'b0;brtype = 1'b0; jmpreq = 1'b0; brreq = 1'b0; end //store
			
			7'b0000011: begin ALUcontrol <= 4'b0000; regwrite = 1; memwr = 1'b0; memr = 1'b1; selb = 0;ImmControl = 3'b000; wbsel = 2'b01;csa=1'b0; sel_a=1'b0;brtype = 1'b0; jmpreq = 1'b0; brreq = 1'b0; end //lw
			
			7'b0010011: begin regwrite = 1; memwr = 1'b0; memr = 1'b0; selb = 1'b0; ImmControl = 3'b000; wbsel = 2'b00;csa=1'b1; sel_a=1'b0;brtype = 1'b0; jmpreq = 1'b0; brreq = 1'b0; //itype
			if (f3==3'b000)      ALUcontrol <= 4'b0000;  //add 
			else if (f3==3'b010) ALUcontrol <= 4'b0011; //slt
			else if (f3==3'b011) ALUcontrol <= 4'b0100; //sltu
			else if (f3==3'b100) ALUcontrol <= 4'b0101; //xori
			else if (f3==3'b110) ALUcontrol <= 4'b1000;  //ori
			else if (f3==3'b111) ALUcontrol <= 4'b1001;  //andi
			else if (f3==3'b001) ALUcontrol <= 4'b0010;  //slli
			else ALUcontrol <= 3'b000;  //base case
			end
			
			7'b1100011: begin ALUcontrol = 4'b0000; selb = 0; regwrite = 0; ImmControl = 3'b101; wbsel = 2'b00; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b1; jmpreq =1'b0; brreq = 1'b1;
			case(f3)
				3'b000: brtype = 3'b000; //equals
				3'b001: brtype = 3'b001; //not equal
				3'b100: brtype = 3'b010; //less than
				3'b101: brtype = 3'b111; //greater than signed
				3'b110: brtype = 3'b100; //less than unsigned
				3'b111: brtype = 3'b011; //greater than unsigned
				default: brtype= 3'b000;
			endcase
			end //btype
			
			7'b1101111: begin ALUcontrol = 4'b0000; selb = 0; regwrite = 1; ImmControl = 3'b011; wbsel = 2'b11; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b1; brtype=1'b0; jmpreq = 1'b1; brreq=1'b0; end //jal 
			
			7'b1100111: begin ALUcontrol = 4'b0000; selb = 0; regwrite = 1; ImmControl = 3'b000; wbsel = 2'b11; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b0; brtype=1'b0; jmpreq = 1'b1; brreq=1'b0; end //jalr
			
			7'b0110111: begin ALUcontrol = 4'b1010; selb = 0; regwrite = 1; ImmControl = 3'b100; wbsel = 2'b00; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b0; brtype=1'b0; jmpreq = 1'b0; brreq=1'b0; end //lui
			
			7'b0010111: begin ALUcontrol = 4'b1010; selb = 0; regwrite = 1; ImmControl = 3'b100; wbsel = 2'b00; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b1; brtype=1'b0; jmpreq = 1'b0; brreq=1'b0; end //auipc
		endcase
	end
endmodule
