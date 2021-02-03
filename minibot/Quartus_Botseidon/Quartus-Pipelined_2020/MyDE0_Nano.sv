
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
//  I2C  
//=======================================================
	
	//assign GPIO_1[4] = GPIO_0_PI[2]; // SDA
	//assign GPIO_1[5] = GPIO_0_PI[3]; // SCL
  
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
			8'b00000 : valeurProv = codLeft;
			8'b00001 : valeurProv = codRight;
			
			8'b00010: valeurProv = nbrTicksLeft1;
			8'b00011: valeurProv = nbrTicksRight1;
			
			8'b00100: valeurProv = nbrTicksLeft2;
			8'b00101: valeurProv = nbrTicksRight2;
			
			8'b00110: valeurProv = nbrTicksLeft3;
			8'b00111: valeurProv = nbrTicksRight3;
			
			8'b01000: valeurProv = nbrTicksLeft4;
			8'b01001: valeurProv = nbrTicksRight4;
			
			8'b01010: valeurProv = nbrTicksLeft5;
			8'b01011: valeurProv = nbrTicksRight5;
			
			8'b01100: valeurProv = nbrTicksLeft6;
			8'b01101: valeurProv = nbrTicksRight6;
			
			8'b01110: valeurProv = nbrTicksLeft7;
			8'b01111: valeurProv = nbrTicksRight7;
			
			8'b10000: valeurProv = nbrTicksLeft8;
			8'b10001: valeurProv = nbrTicksRight8;
			
			8'b10010: valeurProv = nbrTicksLeft9;
			8'b10011: valeurProv = nbrTicksRight9;
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


	encoder EncodeurLeft(leftA, leftB, CLOCK_50,codLeft);
	encoder EncodeurRight(rightA, rightB, CLOCK_50 ,codRight);

	// nombres de ticks

	logic [31:0] nbrTicksLeft1;
	logic [31:0] nbrTicksRight1;
	
	logic [31:0] nbrTicksLeft2;
	logic [31:0] nbrTicksRight2;
	
	logic [31:0] nbrTicksLeft3;
	logic [31:0] nbrTicksRight3;
	
	logic [31:0] nbrTicksLeft4;
	logic [31:0] nbrTicksRight4;
	
	logic [31:0] nbrTicksLeft5;
	logic [31:0] nbrTicksRight5;
	
	logic [31:0] nbrTicksLeft6;
	logic [31:0] nbrTicksRight6;
	
	logic [31:0] nbrTicksLeft7;
	logic [31:0] nbrTicksRight7;
	
	logic [31:0] nbrTicksLeft8;
	logic [31:0] nbrTicksRight8;
	
	logic [31:0] nbrTicksLeft9;
	logic [31:0] nbrTicksRight9;
	

	nombreDeTick1 tick1l(leftA, CLOCK_50, nbrTicksLeft1);
	nombreDeTick1 tick1r(rightA, CLOCK_50, nbrTicksRight1);
	
	nombreDeTick1 tick2l(leftB, CLOCK_50, nbrTicksLeft2);
	nombreDeTick1 tick2r(rightB, CLOCK_50, nbrTicksRight2);
	
	nombreDeTick3 tick3l(leftA, leftB, CLOCK_50, nbrTicksLeft3);
	nombreDeTick3 tick3r(rightA, rightB, CLOCK_50, nbrTicksRight3);
	
	nombreDeTick4 tick4l(leftA, leftB, CLOCK_50, nbrTicksLeft4);
	nombreDeTick4 tick4r(rightA, rightB, CLOCK_50, nbrTicksRight4);
	
	nombreDeTick5 tick5l(leftA, CLOCK_50, nbrTicksLeft5);
	nombreDeTick5 tick5r(rightA, CLOCK_50, nbrTicksRight5);
	
	nombreDeTick5 tick6l(leftB, CLOCK_50, nbrTicksLeft6);
	nombreDeTick5 tick6r(rightB, CLOCK_50, nbrTicksRight6);
	
	nombreDeTick6 tick7l(leftA, leftB, CLOCK_50, nbrTicksLeft7);
	nombreDeTick6 tick7r(rightA, rightB, CLOCK_50, nbrTicksRight7);
	
	nombreDeTick2 tick8l(leftA, CLOCK_50, nbrTicksLeft8);
	nombreDeTick2 tick8r(rightA, CLOCK_50, nbrTicksRight8);
	
	nombreDeTick2 tick9l(leftB, CLOCK_50, nbrTicksLeft9);
	nombreDeTick2 tick9r(rightB, CLOCK_50, nbrTicksRight9);
	
	
endmodule


//=======================================================
//  Module nbrTick1 : posedge A
//=======================================================

module nombreDeTick1(input logic A, clk,
						  output logic [31:0] Ticks);
					
	logic [31:0] bulshit1;
	//A leads B for counter clock wise rotation
	always_ff @(posedge A) begin
		bulshit1 = bulshit1 + 1'b1;
	end	
	
	assign Ticks = bulshit1;
	
endmodule

//=======================================================
//  Module nbrTick2 : negedge A
//=======================================================

module nombreDeTick2(input logic A, clk,
						  output logic [31:0] Ticks);
					
	logic [31:0] bulshit1;
	//A leads B for counter clock wise rotation
	always_ff @(negedge A) begin
		bulshit1 = bulshit1 + 1'b1;
	end	
	
	assign Ticks = bulshit1;
	
endmodule

//=======================================================
//  Module nbrTick3 : posedge A et B
//=======================================================

module nombreDeTick3(input logic A, B, clk,
						  output logic [31:0] Ticks);
					
	logic [31:0] bulshit1,bulshit3;
	//A leads B for counter clock wise rotation
	always_ff @(posedge A) begin
		bulshit1 = bulshit1 + 1'b1;
	end
	
	always_ff @(posedge B) begin
		bulshit3 = bulshit3 + 1'b1;
	end

	
	assign Ticks = bulshit1 + bulshit3;
	
endmodule

//=======================================================
//  Module nbrTick4 : negedge A et B
//=======================================================

module nombreDeTick4(input logic A, B, clk,
						  output logic [31:0] Ticks);
					
	logic [31:0] bulshit1,bulshit3;
	//A leads B for counter clock wise rotation
	always_ff @(negedge A) begin
		bulshit1 = bulshit1 + 1'b1;
	end
	
	always_ff @(negedge B) begin
		bulshit3 = bulshit3 + 1'b1;
	end

	
	assign Ticks = bulshit1 + bulshit3;
	
endmodule


//=======================================================
//  Module nbrTick5 : posedge et negedge A
//=======================================================

module nombreDeTick5(input logic A, clk,
						  output logic [31:0] Ticks);
					
	logic [31:0] bulshit1,bulshit2;
	//A leads B for counter clock wise rotation
	always_ff @(posedge A) begin
		bulshit1 = bulshit1 + 1'b1;
	end
	
	always_ff @(negedge A) begin
		bulshit2 = bulshit2 + 1'b1;
	end
	
	assign Ticks = bulshit1 + bulshit2;
	
endmodule

//=======================================================
//  Module nbrTick6 : posedge et negedge de A et B
//=======================================================

module nombreDeTick6(input logic A, B, clk,
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
