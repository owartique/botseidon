
//=======================================================
//  MyARM
//=======================================================

module MyDE0_Nano(

//////////// CLOCK //////////
input logic 		          		CLOCK_50,

//////////// LED //////////
output logic		     [7:0]		LED,

//////////// KEY //////////
input logic 		     [1:0]		KEY,

//////////// SW //////////
input logic 		     [3:0]		SW,

//////////// 2x13 GPIO Header //////////
inout logic 		    [12:0]		GPIO_2,
input logic 		     [2:0]		GPIO_2_IN,

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_0_PI,
input logic 		     [1:0]		GPIO_0_PI_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_1,
input logic 		     [1:0]		GPIO_1_IN
);			 

//=======================================================
//  Instantiation generale
//=======================================================

	logic 		 	clk, reset;
	
	assign clk   = CLOCK_50;
	assign reset = GPIO_0_PI[1]; 
  
//=======================================================
//  SPI
//=======================================================

	//instantiation des 4 elements de la communication SPI
	
	logic 			spi_clk, spi_cs, spi_mosi, spi_miso;			

	assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
	assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
	assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15
	assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13
	
	//elements ajoutes au SPI
	 
	logic [31:0] DataToSend;
	logic [7:0]	 DataAdr;
	
	spi_slave spi_slave_instance(
		.SPI_CLK    (spi_clk),			 //input
		.SPI_CS     (spi_cs),			 //input
		.SPI_MOSI   (spi_mosi),			 //input
		.SPI_MISO   (spi_miso),			 //input
		.Data_Addr  (DataAdr),			 //output 	(renvoie l'adresse ou il faut lire l'info)	
		.Data_Read  (DataToSend),		 //input		(avec DataAdr on va dans notre "memoire" et on renvoit DataTosend pour le mettre sur le PI)
		.Clk        (clk)				    //input
	);

	
//=======================================================
//  Memory
//=======================================================	

	logic [31:0] valeurProv;
	
	always_comb begin
		case(DataAdr)
			8'b0 : valeurProv = codLeft;
			8'b1 : valeurProv = codRight;
			8'b10: valeurProv = nbrTicksLeft;
			8'b11: valeurProv = nbrTicksRight;
			default: valeurProv = 32'b1111_1111_1111_1111_1111_1111_1111_1111;
		endcase
	end
	assign DataToSend = valeurProv;

//=======================================================
//  Encoder
//=======================================================	

	// vitesses

	logic [31:0] codLeft;
	logic [31:0] codRight;

	logic leftA,leftB;
	assign leftA = GPIO_1[0];
	assign leftB = GPIO_1_IN[0];

	logic rightA, rightB;
	assign rightA = GPIO_1[2];
	assign rightB = GPIO_1[1];


	encoder EncodeurLeft(leftA,leftB,CLOCK_50,codLeft);
	encoder EncodeurRight(rightA,rightB,CLOCK_50,codRight);

	// nombres de ticks

	logic [31:0] nbrTicksLeft;
	logic [31:0] nbrTicksRight;

	nombreDeTick tick1(leftA, leftB, CLOCK_50, nbrTicksLeft);
	nombreDeTick tick2(rightA, rightB, CLOCK_50, nbrTicksRight);


endmodule


//=======================================================
//  Module nbrTick
//=======================================================

module nombreDeTick(input logic A, B, clk,
						  output logic [31:0] Ticks);
					
	logic [31:0] bulshit1,bulshit2,bulshit3,bulshit4;
	//A leads B for counter clock wise rotation
	always_ff @(posedge A) begin
		bulshit1 = bulshit1 + 1'b1;
	end
	
	always_ff @(negedge A) begin
		bulshit2 = bulshit2 + 1'b1;
	end
	
	always_ff @(posedge B) begin
		bulshit3 = bulshit3 + 1'b1;
	end
	
	always_ff @(negedge B) begin
		bulshit4 = bulshit4 + 1'b1;
	end
	
	
	assign Ticks = bulshit1 + bulshit2 + bulshit3 + bulshit4;
	
endmodule


////===========================================================
////  Module encoder : pulse = nombre de tick + offset
////							pulse > offset -> marche avant
////							pulse < offset -> marche arriere
////
////							tickAP compte les flancs montant de A
////							tickAN compte les flancs descendant de A
////===========================================================	
//
//module encoder(input  logic        A,B,clk,
//					output logic [31:0] pulse);
//
//	logic [31:0] cnt, valeurProv;
//	logic [31:0] tickAP, tickAN, tickBP, tickBN;
//	logic tickReset;
//
//	// compte le temps et donne la valeur provisoir au dt fixe
//	
//	always_ff @(posedge clk) begin
//		if (cnt==32'd1_000_000) begin
//			valeurProv <= tickAP + tickAN + tickBP + tickBN;
//			cnt <= 0;
//			tickReset <= 1'b1;
//		end
//		else begin
//			tickReset <= 1'b0;
//			cnt <= cnt + 32'b1;
//		end
//		
//	end
//	
//	//actualisent les compteurs aux 4 flancs possibles
//	
//	always_ff @(posedge A, posedge tickReset) begin
//		if(tickReset) tickAP <= 16'b0000_0011_1111_1111;
//		else tickAP <= B ? tickAP-32'b1 : tickAP+32'b1;		
//	end
//	
//	always_ff @(negedge A, posedge tickReset) begin
//		if(tickReset) tickAN <= 16'b0000_0011_1111_1111;
//		else tickAN <= B ? tickAN+32'b1 : tickAN-32'b1;		
//	end
//	
//	always_ff @(posedge B, posedge tickReset) begin
//		if(tickReset) tickBP <= 16'b0000_0011_1111_1111;
//		else tickBP <= A ? tickBP+32'b1 : tickBP-32'b1;		
//	end
//	
//	always_ff @(negedge B, posedge tickReset) begin
//		if(tickReset) tickBN <= 16'b0000_0011_1111_1111;
//		else tickBN <= A ? tickBN-32'b1 : tickBN+32'b1;		
//	end
//	
//	assign pulse = valeurProv;
//	
//endmodule
