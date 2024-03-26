module main(input logic clk,reset,intr);
	logic [31:0] pcnexta,add,instr,pca,wbpc,newadd,wbadd,newinstr,wb,csr_data,csr_addr,epc,csr_rdata,newpca;
	logic signed [31:0] a,b,aluresult,newb,newa,immgen,rdata,regwb,pipeb,newalu,fnewa,fnewb;
	logic [2:0] br_type,ImmControl;
	logic [3:0] alusignal;
	logic [1:0] wbsel,pipewbsel;
	logic regwrite,memwr,memr,selb,cs,brtaken,sel_a,brreq,jmpreq,piperegwrite,pipememwr,pipememr,fora,forb,stall,stall_W,pipecs,flush,pipecsr_rd,pipecsr_wr,csr_rd,csr_wr,is_mret,pipeis_mret,epc_taken;
	
	controller Controller(clk,reset,newinstr[6:0],newinstr[30],newinstr[14:12],regwrite,memwr,memr,selb,ImmControl,wbsel,cs,sel_a,brreq,jmpreq,csr_rd,csr_wr,is_mret,br_type,alusignal);
	newcont PipelineController(clk,reset,stall_W,regwrite,memwr,memr,wbsel,cs,csr_rd,csr_wr,is_mret,piperegwrite,pipememwr,pipememr,pipecs,pipewbsel,pipecsr_rd,pipecsr_wr,pipeis_mret);
	pipefor Forwarding(clk,brtaken,newinstr[19:15],newinstr[24:20],wb[11:7],wb[6:0],piperegwrite,fora,forb,stall,stall_W,flush);
	
	m2 PCSelect(pcnexta,aluresult,brtaken,pca);
	m2 EPCSelect(pca,epc,epc_taken,newpca);
	pc PC(clk,reset,newpca,add);
	pcnext PCPlus4(add,pcnexta);
	instrmem InstructionMemory(add,instr);
	regisone InstructionRegister_decode(clk,reset,stall,flush,instr,newinstr);
	regis pcfetch_decode(clk,reset,stall,add,newadd);
	regfile RegisterFile(clk,piperegwrite,newinstr[19:15],newinstr[24:20],wb[11:7],regwb,a,b);
	immediate ImmGen(newinstr,ImmControl,immgen);
	m2 ForwardA(a,newalu,fora,fnewa);
	m2 ForwardB(b,newalu,forb,fnewb);
	m2 regaselect(fnewa,newadd,sel_a,newa);
	m2 ImmSelect(immgen,fnewb,selb,newb);
	alu ALU(newa,newb,alusignal,aluresult);
	regis aluRegister_mem(clk,reset,stall_W,aluresult,newalu);
	regis IRregis_mem(clk,reset,stall_W,newinstr,wb);
	branchcond BranCon(fnewa,fnewb,br_type,brreq,jmpreq,brtaken);
	regis datawdRegister_mem(clk,reset,stall_W,b,pipeb);
	datamem DataMemory(newalu,pipeb,wb[14:12],clk,pipememwr,pipememr,pipecs,rdata);
	pcnext PCPlus4_2(wbadd,wbpc);
	regis pc_mem(clk,reset,stall_W,newadd,wbadd);
	m3 WriteBackSelect(newalu,rdata,wbpc,csr_rdata,pipewbsel,regwb);	
	
	
	regis Data_CSR(clk,reset,stall_W,fnewa,csr_data);
	regis Addr_CSR(clk,reset,stall_W,immgen,csr_addr);
	csr_reg csr(csr_addr[11:0],wbadd,csr_data,clk,reset,pipecsr_wr,pipecsr_rd,intr,pipeis_mret,csr_rdata,epc,epc_taken);
	
endmodule


module pipefor(input clk,brtaken,input logic [4:0] a,b,rd,input logic [6:0] wb,input logic piperegwrite,output logic fora,forb,stall, stall_W,flush);
	logic rs1haz,rs2haz,rs1valid,rs2valid;
	
	assign rs1valid = |a;
	assign rs2valid = |b;
	
	assign rs1haz = ((a==rd) & piperegwrite) & rs1valid;
	assign rs2haz = ((b==rd) & piperegwrite) & rs1valid;
	
	assign stall = ((a==rd) | (b==rd)) & (wb==7'b0000011);
	assign stall_W = 1'b0;
	assign flush = (brtaken) ? 1'b1: 1'b0;
	
	
	assign fora = rs1haz;
	assign forb = rs2haz;
endmodule

module newcont(input logic clk,reset,stall,regwrite,memwr,memr,input logic [1:0] wbsel, input logic cs,csr_rd,csr_wr,is_mret,output logic piperegwrite,pipememwr,pipememr,pipecs,output logic [1:0] pipewbsel,output logic pipecsr_rd,pipecsr_wr,pipeis_mret);
	always_ff@(posedge clk) begin
		if (reset | (stall==2'b11)) begin
			piperegwrite <=1'b0;
			pipememwr <= 1'b0;
			pipememr <= 1'b0;
			pipewbsel <= 2'b00;
			pipecs <= 1'b1;
			pipecsr_rd <= 1'b0;
			pipecsr_wr <= 1'b0;
			pipeis_mret <= 1'b0;
		end
		else begin
			piperegwrite <=regwrite;
			pipememwr <= memwr;
			pipememr <= memr;
			pipewbsel <= wbsel;
			pipecs <= cs;
			pipecsr_wr <= csr_wr;
			pipecsr_rd <= csr_rd;
			pipeis_mret <= is_mret;
		end
	end
endmodule 

module regisone(input logic clk,reset,stall,flush, input logic [31:0] a, output logic [31:0] b);
	always_ff@(posedge clk) begin
		if (reset|flush) b <= 'd0; else if (~stall) b<=a;
	end
endmodule

module regisa(input logic clk, reset, input logic [31:0] a, output logic [31:0] b);
	always_ff@(posedge clk) begin
		if (reset) b <= 'b0; else b<=a;
	end
endmodule 

module regis(input logic clk,reset,stall, input logic [31:0] a, output logic [31:0] b);
	always_ff@(posedge clk) begin
		if (reset) b <= 'b0; else if (~stall) b<=a;
	end
endmodule


module m3(input logic signed [31:0] a,b,c,d, input logic [1:0] control, output logic signed [31:0] result);
	always_comb begin
		if (control==2'b01) result = b;
		else if (control == 2'b00) result = a;
		else if (control == 2'b11) result = c;
		else if (control == 2'b10) result = d;
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
		$readmemh("C:/intercodes.txt",rega);
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
		regb[1][31:0] = 'd15;
		regb[2][31:0] = 'd8;
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
			11: ALUresult <= 'b0;
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

module controller(input logic clk,reset,input logic [6:0]instr,input logic funct7,input logic [2:0] f3, output logic regwrite,memwr,memr,selb, output logic [2:0] ImmControl, output logic [1:0] wbsel, output logic csa, sel_a, brreq,jmpreq,csr_rd,csr_wr,is_mret, output logic [2:0] brtype, output logic [3:0] ALUcontrol);
	always_comb begin
		csa = 1; csr_rd = 1'b0; csr_wr = 1'b0; is_mret = 1'b0;
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
			
			7'b1110011: begin case(f3) //csrrw
				3'b001: begin ALUcontrol = 'd11; selb = 0; regwrite = 1; ImmControl = 3'b000; wbsel = 2'b10; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b1; brtype=1'b0; jmpreq = 1'b0; brreq=1'b0;csr_rd = 1'b1; csr_wr = 1'b1;is_mret = 1'b0;end //rw
				3'b000: begin ALUcontrol = 'd11; selb = 0; regwrite = 0; ImmControl = 3'b000; wbsel = 2'b10; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b1; brtype=1'b0; jmpreq = 1'b0; brreq=1'b0;csr_rd = 1'b0; csr_wr = 1'b0;is_mret = 1'b1;end //mret
				endcase end
				
			default: begin ALUcontrol = 'd11; selb = 0; regwrite = 0; ImmControl = 3'b100; wbsel = 2'b00; csa = 1; memwr = 1'b0; memr = 1'b0; sel_a=1'b1; brtype=1'b0; jmpreq = 1'b0; brreq=1'b0; end //noop
		endcase
	end
endmodule
