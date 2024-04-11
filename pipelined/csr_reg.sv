module csr_reg(input logic [11:0] addr, input logic [31:0] pc, wdata, input logic clk,reset,wr_en,rd_en,interupt,is_mret, output logic [31:0] rdata, epc,output logic epctaken);
	logic [31:0] csr_mip,csr_mie,csr_mstatus, csr_mcause, csr_mtvec, csr_mepc;
	logic  csr_mip_f,csr_mie_f,csr_mstatus_f, csr_mcause_f, csr_mtvec_f, csr_mepc_f,interupt_csr_en;
	

	always_comb begin //mret 
		if (is_mret) begin epctaken = 1'b1; epc = csr_mepc; end
		else if (interupt_csr_en) begin 
				epctaken = 1'b1;
				if (csr_mtvec [1:0] == 2'b00) epc = { {csr_mtvec[31:2]}, 2'b00};
                else if (csr_mtvec [1:0] == 2'b01) epc = { {csr_mtvec[31:2], 2'b00}}  + (csr_mcause << 2); end
		else begin epctaken=1'b0; epc = pc; end
	end
	
	always_comb begin //read
		if (rd_en) begin
			case(addr) 
				12'h300 : rdata = csr_mstatus; // mstatus csr
				12'h304 : rdata = csr_mie; // csr mie
				12'h305 : rdata = csr_mtvec; // csr mtvec
				12'h341 : rdata = csr_mepc;    // csr mepc
				12'h342 : rdata = csr_mcause; //csr_mcause
				12'h344 : rdata = csr_mip; // csr mip
				default : rdata = '0;
			endcase
		end
	end
	
	always_comb begin //write flags
		csr_mip_f = 1'b0;
		csr_mie_f = 1'b0;
		csr_mstatus_f = 1'b0;
		csr_mcause_f = 1'b0;
		csr_mtvec_f = 1'b0;
		csr_mepc_f = 1'b0;
		if (wr_en) begin
			case(addr)
				12'h300 : csr_mstatus_f = 1'b1; // mstatus csr
				12'h304 : csr_mie_f = 1'b1; // csr mie
				12'h305 : csr_mtvec_f = 1'b1; // csr mtvec
				12'h341 : csr_mepc_f = 1'b1;    // csr mepc
				12'h342 : csr_mcause_f = 1'b1; //csr_mcause
				12'h344 : csr_mip_f = 1'b1; // csr mip
			endcase
		end	
	end
	
	always_ff@(negedge reset,posedge clk) begin //actually writing to registers!
		if (reset) begin csr_mip <= '0; csr_mcause <= '0; csr_mepc <= '0; csr_mtvec <= '0; csr_mie <= '0; csr_mstatus<= '0; end 
		else begin
		if (interupt) csr_mip[11] = 1'b1; else if (csr_mip_f) csr_mip <= wdata;
		if (interupt_csr_en) csr_mcause = 32'b1; else if (csr_mcause_f) csr_mcause <= wdata;
		if (interupt_csr_en) csr_mepc <= pc; else if (csr_mepc_f) csr_mepc <= wdata;
		if (csr_mtvec_f) csr_mtvec <= wdata;
		if (csr_mie_f) csr_mie <= wdata;
		if (csr_mstatus_f) csr_mstatus <= wdata;
		end	
	end
	
	always_comb begin
		interupt_csr_en = csr_mstatus[3] & (csr_mip[11] & csr_mie[11]);
	end
endmodule
