//===========================================================
//  Module encoder : pulse = nombre de tick + offset
//							pulse > offset -> marche avant
//							pulse < offset -> marche arriere
//
//							tickAP compte les flancs montant de A
//							tickAN compte les flancs descendant de A
//===========================================================	

module encoder(input  logic        A,B,clk,
					output logic [31:0] pulse);

	logic [31:0] cnt, valeurProv;
	logic [31:0] tickAP, tickAN, tickBP, tickBN;
	logic tickReset;

	// compte le temps et donne la valeur provisoir au dt fixe
	
	always_ff @(posedge clk) begin
		if (cnt==32'd1_000_000) begin
			valeurProv <= tickAP + tickAN + tickBP + tickBN;
			cnt <= 0;
			tickReset <= 1'b1;
		end
		else begin
			tickReset <= 1'b0;
			cnt <= cnt + 32'b1;
		end
		
	end
	
	//actualisent les compteurs aux 4 flancs possibles
	
	always_ff @(posedge A, posedge tickReset) begin
		if(tickReset) tickAP <= 16'b0000_0011_1111_1111;
		else tickAP <= B ? tickAP-32'b1 : tickAP+32'b1;		
	end
	
	always_ff @(negedge A, posedge tickReset) begin
		if(tickReset) tickAN <= 16'b0000_0011_1111_1111;
		else tickAN <= B ? tickAN+32'b1 : tickAN-32'b1;		
	end
	
	always_ff @(posedge B, posedge tickReset) begin
		if(tickReset) tickBP <= 16'b0000_0011_1111_1111;
		else tickBP <= A ? tickBP+32'b1 : tickBP-32'b1;		
	end
	
	always_ff @(negedge B, posedge tickReset) begin
		if(tickReset) tickBN <= 16'b0000_0011_1111_1111;
		else tickBN <= A ? tickBN-32'b1 : tickBN+32'b1;		
	end
	
	assign pulse = valeurProv;
	
endmodule
