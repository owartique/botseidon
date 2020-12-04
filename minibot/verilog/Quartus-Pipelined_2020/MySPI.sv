//-------------------------------------------------------------
//
// SPI Module
// 

module spi_slave(
	input  logic 			SPI_CLK, 		//clock issued by Rasp
	input  logic			SPI_CS,  		//chip select issued by rasp
	input  logic 			SPI_MOSI,		//Data coming bit per bit from rasp
	output logic 			SPI_MISO,		//Data coming bit per bit from DE0	
	output logic[7:0] 	Data_Addr,		//Address decodée depuis le message reçu du rasp
	input  logic[31:0] 	Data_Read,		//Data a envoyer au PI
	input  logic			Clk				//clock de 50MHz du DE0 	
);

	logic [39:0] SPI_reg; //les trois premiers MSB sont l'addresse et les 32 LSB sont les données
 
	
	
//---SPI Sysnchronization -------------------------------------

	logic SPI_CLK_sync;
	logic SPI_CS_sync;

	always_ff @(posedge Clk) begin
		SPI_CLK_sync <= SPI_CLK;
		SPI_CS_sync  <= SPI_CS;
	end
	
//--- SPI FSM -------------------------------------------------

	typedef enum logic [1:0] {S0,S1,S2,S3} statetype;
	statetype state, nextstate;
	
	logic [5:0] SPI_cnt;
	logic 		SPI_cnt_reset, SPI_cnt_inc;
	logic			SPI_reg_reset, SPI_reg_shift, SPI_reg_load;	
	logic 		MISO_we, MISO_reset;

// State Register & Bit counter & SPI Register & MISO
	
	always_ff @(posedge Clk) begin
	
		if (SPI_CS_sync)			state <= S0;
		else 							state <= nextstate;
		
		if (SPI_cnt_reset) 	 	SPI_cnt <= 6'b0;
		else if (SPI_cnt_inc) 	SPI_cnt <= SPI_cnt + 6'b1;

		if (SPI_reg_reset) 		SPI_reg <= 40'b0;
		else if (SPI_reg_shift)	SPI_reg <= {SPI_reg[38:0], SPI_MOSI};
		else if (SPI_reg_load)	begin
		Data_Addr <= SPI_reg[7:0];
		SPI_reg <= {Data_Read, SPI_reg[7:0]};
		end
		
		if (MISO_reset) 			SPI_MISO <= 0;
		else if (SPI_reg_load)	SPI_MISO <= Data_Read[31];
		else if (MISO_we)			SPI_MISO <= SPI_reg[39];
		
	end
	
// Next State Logic

	always_comb begin
	
		// Default value
		nextstate     = state;
		SPI_cnt_reset = 0; 
		SPI_cnt_inc   = 0;
		SPI_reg_reset = 0; 
		SPI_reg_shift = 0; 
		SPI_reg_load  = 0;
		MISO_reset    = 0;
		MISO_we       = 0;
		
		
		case (state)
			S0 : if (~SPI_CS_sync) begin 			// negedge of SPI_CS
						SPI_cnt_reset = 1;
						SPI_reg_reset = 1;
						MISO_reset    = 1;
						nextstate = S1;
					end
					
			S1	: if (SPI_CLK_sync) begin			// posedge of SPI_CLK
						SPI_reg_shift = 1;
						SPI_cnt_inc   = 1;
						nextstate = S2;
					end
					
			S2 : if (~SPI_CLK_sync) begin			// negedge of SPI_CLK
						MISO_we = 1;
						if (SPI_cnt == 8) SPI_reg_load = 1; // quand on a fait 8 shift, on a déjà l'addresse de la donnée qu'on veut
						if (SPI_cnt == 40) begin            // 36 shifts, on a terminé
							nextstate = S3;
						end
						else nextstate = S1;
					end
					
			S3 : if (SPI_CS_sync) begin 			// posedge of SPI_CS
						nextstate = S0;
					end
			default : nextstate = S0;
		endcase
	end
	
endmodule

//-------------------------------------------------------------
/*
	Dès qu'on a lu 4 bits, on sait déjà a quelle addresse il faut aller chercher la donnée. Donc on load dans le SPI_reg le 32bits situés à l'addresse 
	reçu et on les mets en MSB.
	Maintenant il faut encore shifter de 32 fois pour envoyer chaque les 32bits vers le Raspberry
*/