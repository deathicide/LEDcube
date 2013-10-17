#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

// ATMEL ATMEGA1284P pinout
//
//                       +---\/---+
//           (D 0) PB0  1|        |40  PA0 (AI 0 / D24)
//           (D 1) PB1  2|        |39  PA1 (AI 1 / D25)
//      INT2 (D 2) PB2  3|        |38  PA2 (AI 2 / D26)
//       PWM (D 3) PB3  4|        |37  PA3 (AI 3 / D27)
//    PWM/SS (D 4) PB4  5|        |36  PA4 (AI 4 / D28)
//      MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D29)
//  PWM/MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D30)
//   PWM/SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D31)
//                 RST  9|        |32  AREF
//                 VCC 10|        |31  GND 
//                 GND 11|        |30  AVCC
//               XTAL2 12|        |29  PC7 (D 23)
//               XTAL1 13|        |28  PC6 (D 22)
//      RX0 (D 8)  PD0 14|        |27  PC5 (D 21) TDI
//      TX0 (D 9)  PD1 15|        |26  PC4 (D 20) TDO
// RX1/INT0 (D 10) PD2 16|        |25  PC3 (D 19) TMS
// TX1/INT1 (D 11) PD3 17|        |24  PC2 (D 18) TCK
//      PWM (D 12) PD4 18|        |23  PC1 (D 17) SDA
//      PWM (D 13) PD5 19|        |22  PC0 (D 16) SCL
//      PWM (D 14) PD6 20|        |21  PD7 (D 15) PWM
//                       +--------+

//programming with Arduino as ISP
//atmega       ->  arduino as isp
//mosi  pin 6  ->  pin D 11
//miso  pin 7  ->  pin D 12
//sck   pin 8  ->  pin D 13
//reset pin 9  ->  pin D 10
//arduino heartbeat
//pin D 9 is anode of led to R to gnd

//hardware setup
//port a
//PA0-PA2 is selection on demux to the ff clocks (high)
//PA3 is OEbar on the octal flip flops (low)
//PA4-PA7 are nc
//port b
//PB0-PB7 are data outputs to the octal flip flops (high)
//port c
//PC0 is change display button (high) this is pcint16
//PC1 is indicator led (high) this is pcint17
//PC2-PC7 are nc
//port d
//PD0-PD7 is MOSFET layer gnd selection (high)

//definitions
#define DATA_BUS PORTB
#define LAYER_SELECT PORTD
#define ADD_BUS PORTA
#define ADD_MASK     B00000111
#define ADD_MASK_BAR B11111000
#define OE PORTA
//generally or with oemask
#define OE_MASK     B00001000
//generally and with oemaskbar
#define OE_MASK_BAR B11110111
//green led
#define GREEN_LED 17
#define GREEN_LED_MASK B00000010

//function declarations
//init stuff
void boot();
void delaymSec(unsigned int i);
void delaySec(byte i);
void initPorts();
void initTimer2();

//simple drawing instructions
void checkerboardLayer(byte k);
void checkerboardxWall(byte k);
void checkerboardyWall(byte i);
void clrCol(byte i,byte j);
void clrLayer(byte i);
void clrVoxel(byte i,byte j,byte k);
void clrxRow(byte i,byte j);
void clrxWall(byte i);
void clryRow(byte i,byte k);
void clryWall(byte i);
void copyLayercube(byte layertmp,byte layercube);
void copyLayertmp(byte layercube,byte layertmp);
void copyxWallcube(byte xwall,byte xwallcube);
void copyxWalltmp(byte xwall,byte xwalltmp);
void copyyWallcube(byte ywall,byte ywallcube);
void copyyWalltmp(byte ywall,byte ywalltmp);
void drawCharLayer(byte chr,byte k);
void drawCharxWall(byte chr,byte j);
void drawCharyWall(byte chr,byte i);
byte getCharByte(byte chr,byte i);
void flipx();
void flipy();
void flipz();
boolean getVoxelStatecube(byte i,byte j,byte k);
void invertCol(byte i,byte j);
void invertCube();
void invertLayer(byte k);
void invertVoxel(byte i,byte j,byte k);
void invertxRow(byte j,byte k);
void invertxWall(byte j);
void invertyRow(byte i,byte k);
void invertyWall(byte i);
void mirrorCol(byte i,byte j);
void mirrorLayerx(byte k);
void mirrorLayery(byte k);
void mirrorxRow(byte j,byte k);
void mirrorxWall(byte j);
void mirroryRow(byte i,byte k);
void mirroryWall(byte j);
void rotateCCWonX();
void rotateCWonX();
void rotateCCWonY();
void rotateCWonY();
void rotateCCWonZ();
void rotateCWonZ();
void moveBuffercube();
void sendVoxelHoriX(byte i,byte j,byte k,char dir,unsigned int time);
void sendVoxelHoriY(byte i,byte j,byte k,char dir,unsigned int time);
void sendVoxelVert(byte i,byte j,byte k,char dir,unsigned int time);
byte sndVoxHoriYLogic(byte i,byte k,byte tmp,char a);
byte sndVoxVertLogic(byte i,byte j,byte tmp,char a);
void setCol(byte i,byte j);
void setcubeBuffer(byte value);
void setLayer(byte i);
void settmpBuffer(byte value);
void setVoxel(byte i,byte j, byte k);
void setxRow(byte i, byte j);
void setxWall(byte i);
void setyRow(byte i, byte k);
void setyWall(byte i);
void shiftLayer(byte layer,int n);
void shiftxWall(byte xwall,int n);
void shiftyWall(byte ywall,int n);

//constants
//size of the cube
static const byte CUBESIZE = 8;

//global variables
//voxel positions in the cube
byte x = 0;
byte y = 0;
byte z = 0;

//characters
//store in program mem
//read from program mem by using pgm_read_byte_near("character" + i)
//or can use *_far if *_near doesnt span enough mem space
//A
prog_uchar A[8] PROGMEM = {B00111100,
                           B01111110,
                           B01100110,
                           B01111110,
                           B01111110,
                           B01100110,
                           B01100110,
                           B01100110};
prog_uchar B[8] PROGMEM = {B00111110,
                           B01111110,
                           B01100110,
                           B00111110,
                           B00111110,
                           B01100110,
                           B01111110,
                           B00111110};
prog_uchar C[8] PROGMEM = {B01111100,
                           B01111110,
                           B00000110,
                           B00000110,
                           B00000110,
                           B00000110,
                           B01111110,
                           B01111100};
prog_uchar D[8] PROGMEM = {B00111110,
                           B01111110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01111110,
                           B00111110};
prog_uchar E[8] PROGMEM = {B01111110,
                           B01111110,
                           B00000110,
                           B01111110,
                           B01111110,
                           B00000110,
                           B01111110,
                           B01111110};
prog_uchar F[8] PROGMEM = {B01111110,
                           B01111110,
                           B00000110,
                           B00011110,
                           B00011110,
                           B00000110,
                           B00000110,
                           B00000110};
prog_uchar G[8] PROGMEM = {B01111100,
                           B01111110,
                           B00000110,
                           B00000110,
                           B00110110,
                           B01100110,
                           B01111110,
                           B00111100};
prog_uchar H[8] PROGMEM = {B01100110,
                           B01100110,
                           B01100110,
                           B01111110,
                           B01111110,
                           B01100110,
                           B01100110,
                           B01100110};
prog_uchar I[8] PROGMEM = {B01111110,
                           B01111110,
                           B00011000,
                           B00011000,
                           B00011000,
                           B00011000,
                           B01111110,
                           B01111110};
prog_uchar J[8] PROGMEM = {B01111110,
                           B01111110,
                           B00011000,
                           B00011000,
                           B00011000,
                           B00011010,
                           B00011010,
                           B00001100};
prog_uchar K[8] PROGMEM = {B01100110,
                           B00110110,
                           B00011110,
                           B00001110,
                           B00001110,
                           B00011110,
                           B00110110,
                           B01100110};
prog_uchar L[8] PROGMEM = {B00000110,
                           B00000110,
                           B00000110,
                           B00000110,
                           B00000110,
                           B00000110,
                           B01111110,
                           B01111110};
prog_uchar M[8] PROGMEM = {B00100100,
                           B01011010,
                           B01011010,
                           B01011010,
                           B01011010,
                           B01011010,
                           B01011010,
                           B01011010};
prog_uchar N[8] PROGMEM = {B00111110,
                           B01111110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110};
prog_uchar O[8] PROGMEM = {B00111100,
                           B01111110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01111110,
                           B00111100};
prog_uchar P[8] PROGMEM = {B00111100,
                           B01111110,
                           B01100110,
                           B01111110,
                           B00111110,
                           B00000110,
                           B00000110,
                           B00000110};
prog_uchar Q[8] PROGMEM = {B00111100,
                           B01111110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B00111110,
                           B01011100};
prog_uchar R[8] PROGMEM = {B00111100,
                           B01111110,
                           B01100110,
                           B01111110,
                           B00111110,
                           B00110110,
                           B01100110,
                           B01000110};
prog_uchar S[8] PROGMEM = {B01111100,
                           B01111110,
                           B00001100,
                           B00011000,
                           B00110000,
                           B01100000,
                           B01111110,
                           B00111110};
prog_uchar T[8] PROGMEM = {B01111110,
                           B01111110,
                           B00011000,
                           B00011000,
                           B00011000,
                           B00011000,
                           B00011000,
                           B00011000};
prog_uchar U[8] PROGMEM = {B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01111110,
                           B00111100};
prog_uchar V[8] PROGMEM = {B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B01100110,
                           B00111100,
                           B00011000};
prog_uchar W[8] PROGMEM = {B01000010,
                           B01000010,
                           B01011010,
                           B01011010,
                           B01011010,
                           B01011010,
                           B01011010,
                           B00100100};
prog_uchar X[8] PROGMEM = {B01100110,
                           B01100110,
                           B00100100,
                           B00011000,
                           B00011000,
                           B00100100,
                           B01100110,
                           B01100110};
prog_uchar Y[8] PROGMEM = {B01100110,
                           B01100110,
                           B00100100,
                           B00011000,
                           B00011000,
                           B00100100,
                           B01100110,
                           B01100110};
prog_uchar Z[8] PROGMEM = {B01111110,
                           B01111110,
                           B00100000,
                           B00010000,
                           B00001000,
                           B00000100,
                           B01111110,
                           B01111110};
prog_uchar n0[8] PROGMEM = {B01111110,
                            B01111110,
                            B01100110,
                            B01100110,
                            B01100110,
                            B01100110,
                            B01111110,
                            B01111110};  
prog_uchar n1[8] PROGMEM = {B01111110,
                            B00010000,
                            B00011100,
                            B00011000,
                            B00011000,
                            B00011000,
                            B01111110,
                            B01111110};
prog_uchar n2[8] PROGMEM = {B01111110,
                            B01111110,
                            B01100000,
                            B01111110,
                            B01111110,
                            B00000110,
                            B01111110,
                            B01111110};
prog_uchar n3[8] PROGMEM = {B01111110,
                            B01111110,
                            B01100000,
                            B01111110,
                            B01111110,
                            B01100000,
                            B01111110,
                            B01111110};   
prog_uchar n4[8] PROGMEM = {B01110000,
                            B01101000,
                            B01100100,
                            B01111110,
                            B01100000,
                            B01100000,
                            B01100000,
                            B01100000};
prog_uchar n5[8] PROGMEM = {B01111110,
                            B01111110,
                            B00000110,
                            B01111110,
                            B01111110,
                            B01100000,
                            B01111110,
                            B01111110};
prog_uchar n6[8] PROGMEM = {B00000110,
                            B00000110,
                            B00000110,
                            B00111110,
                            B01111110,
                            B01100110,
                            B01111110,
                            B00111100};
prog_uchar n7[8] PROGMEM = {B01111110,
                            B01111110,
                            B01100000,
                            B00100000,
                            B00110000,
                            B00010000,
                            B00011000,
                            B00001000};
prog_uchar n8[8] PROGMEM = {B01111110,
                            B01111110,
                            B01100110,
                            B01111110,
                            B01111110,
                            B01100110,
                            B01111110,
                            B01111110};
prog_uchar n9[8] PROGMEM = {B01111110,
                            B01111110,
                            B01100110,
                            B01111110,
                            B01111110,
                            B01100000,
                            B01111110,
                            B01111110};                             
PROGMEM const prog_uchar *chrNames[] = {A,B,C,D,E,F,G,H,I,J,K,L,M,
                                        N,O,P,Q,R,S,T,U,V,W,X,Y,Z,
                                        n0,n1,n2,n3,n4,n5,n6,n7,n8,
                                        n9};

//buffers
//buffer for actual led states
//this buffer is in the form y,z with the byte representing the x rows
//ex cubebuffer[1][2] = B00001111 would light up the first 4 lights
//on the second row to the right and the third layer
volatile byte cubeBuffer[8][8];
//temporary buffer so the cube can be double buffered for complex drawings
volatile byte tmpBuffer[8][8];
//the current layer global
volatile byte currentLayer = 0;

int main(){
  //initialize the ports
  initPorts();
  
  //delay a bit so leds dont possibly get burnt out
  //when programming
  //press reset before uploading
  //take this out of the actual program
  //**************************************************************************************************
  //**************************************************************************************************
  //**************************************************************************************************
  //**************************************************************************************************
  //**************************************************************************************************
  //delaySec(5);
  
  //do some boot time operations
  boot();
  
  //setup timer 2
  initTimer2();
  
  //main loop
  while(1){

    /*
    //start by testing each led individually with a 1 second delay between
    for(z = 0;z<CUBESIZE;z++){
      for(y = 0;y<CUBESIZE;y++){
        for(x = 0;x<CUBESIZE;x++){
          setVoxel(x,y,z);
          delaySec(1);
          clrVoxel(x,y,z);
        }
      }
    }
    
    //now test each xrow
    for(z=0;z<CUBESIZE;z++){
      for(y=0;y<CUBESIZE;y++){
        setxRow(y,z);
        delaySec(1);
        clrxRow(y,z);
      }
    }
    
    //now test each yrow
    for(z=0;z<CUBESIZE;z++){
      for(x=0;x<CUBESIZE;x++){
        setyRow(x,z);
        delaySec(1);
        clryRow(x,z);
      }
    }
    
    //now test each column
    for(y=0;y<CUBESIZE;y++){
      for(x=0;x<CUBESIZE;x++){
        setCol(x,y);
        delaySec(1);
        clrCol(x,y);
      }
    }
    
    //now test each layer
    for(z=0;z<CUBESIZE;z++){
      setLayer(z);
      delaySec(1);
      clrLayer(z);
    }
    
    //now test each wall (vertical layers)
    for(x=0;x<CUBESIZE;x++){
      setxWall(x);
      delaySec(1);
      clrxWall(x);
    }
    for(y=0;y<CUBESIZE;y++){
      setyWall(y);
      delaySec(1);
      clryWall(y);
    }
    
    //now turn on the entire cube
    setcubeBuffer(B11111111);
    delaySec(3);
    setcubeBuffer(0);
    */

    //font tests
    //template
    /*
    cubeBuffer[7][7] = B0;
    cubeBuffer[7][6] = B0;
    cubeBuffer[7][5] = B0;
    cubeBuffer[7][4] = B0;
    cubeBuffer[7][3] = B0;
    cubeBuffer[7][2] = B0;
    cubeBuffer[7][1] = B0;
    cubeBuffer[7][0] = B0;
    delaySec(3);
    */
    
    unsigned int t;
    
    for(t=0;t<1001;t++){
      delaymSec(8);
      setVoxel(0,0,7);
      setLayer(0);
      setxRow(4,4);
      delaymSec(5);
      setcubeBuffer(0);
    }
    
    for(t=0;t<1001;t++){
      delaymSec(6);
      setVoxel(0,7,7);
      setLayer(0);
      setxRow(4,4);
      delaymSec(5);
      setcubeBuffer(0);
    }
    
    for(t=0;t<1001;t++){
      delaymSec(8);
      setVoxel(7,7,7);
      setLayer(0);
      setxRow(4,4);
      delaymSec(6);
      setcubeBuffer(0);
    }
    
    while(1);
    
    //A
    drawCharxWall('A',7);
    delaySec(1);
    for(x=0;x<CUBESIZE-1;x++){
      clrxWall((CUBESIZE-1)-x-1);
      shiftxWall((CUBESIZE-1)-x,-1);
      clrxWall((CUBESIZE-1)-x);
      delaySec(1);
    }
    clrxWall(0);
    //B
    drawCharxWall('B',7);
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    for(x=0;x<CUBESIZE-1;x++){
      clryWall((CUBESIZE-1)-x-1);
      shiftyWall((CUBESIZE-1)-x,-1);
      clryWall((CUBESIZE-1)-x);
      delaySec(1);
    }
    clryWall(0);
    //C
    drawCharxWall('C',7);
    delaySec(1);
    clryWall(0);
    rotateCWonZ();
    delaySec(1);
    for(x=0;x<CUBESIZE-1;x++){
      clryWall(x+1);
      shiftyWall(x,1);
      clryWall(x);
      delaySec(1);
    }
    clryWall(7);
    //D
    drawCharxWall('D',7);
    delaySec(1);
    flipy();
    delaySec(1);
    for(x=0;x<CUBESIZE-1;x++){
      clrxWall(x+1);
      shiftxWall(x,1);
      clrxWall(x);
      delaySec(1);
    }
    clrxWall(7);
    //E
    drawCharxWall('E',7);
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    clrxWall(7);
    //F
    drawCharxWall('F',7);
    delaySec(1);
    rotateCWonZ();
    delaySec(1);
    rotateCWonZ();
    delaySec(1);
    rotateCWonZ();
    delaySec(1);
    rotateCWonZ();
    delaySec(1);
    clrxWall(7);
    //G
    drawCharxWall('G',7);
    delaySec(1);
    shiftxWall(7,-1);
    clrxWall(7);
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    setcubeBuffer(0);
    //H
    drawCharxWall('H',7);
    delaySec(1);
    shiftxWall(7,-3);
    clrxWall(7);
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    setcubeBuffer(0);
    //I
    drawCharxWall('I',7);
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    for(z=0;z<CUBESIZE;z++){
      for(y=0;y<CUBESIZE;y++){
        sendVoxelHoriX(7,y,z,-1,10);
      }
    }
    clryWall(0);
    //J
    drawCharxWall('J',7);
    delaySec(1);
    for(z=0;z<CUBESIZE;z++){
      for(x=0;x<CUBESIZE;x++){
        sendVoxelHoriY(x,7,z,-1,10);
      }
    }
    for(z=0;z<CUBESIZE;z++){
      for(x=0;x<CUBESIZE;x++){
        sendVoxelHoriY(x,0,z,1,10);
      }
    }
    delaySec(1);
    clrxWall(0);
    //K
    drawCharxWall('K',7);
    delaySec(1);
    rotateCCWonX();
    delaySec(1);
    rotateCCWonX();
    delaySec(1);
    rotateCCWonX();
    delaySec(1);
    rotateCCWonX();
    delaySec(1);
    clryWall(0);
    //L
    drawCharxWall('L',7);
    delaySec(1);
    rotateCWonX();
    delaySec(1);
    rotateCWonX();
    delaySec(1);
    rotateCWonX();
    delaySec(1);
    rotateCWonX();
    delaySec(1);
    clryWall(0);
    //M
    drawCharxWall('M',7);
    delaySec(1);
    rotateCWonZ();
    delaySec(1);
    rotateCCWonY();
    delaySec(1);
    rotateCCWonY();
    delaySec(1);
    rotateCCWonY();
    delaySec(1);
    rotateCCWonY();
    delaySec(1);
    clryWall(0);
    //N
    drawCharxWall('N',7);
    delaySec(1);
    rotateCCWonZ();
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    clryWall(7);
    //O
    drawCharxWall('O',7);
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    rotateCWonY();
    delaySec(1);
    //P
    drawCharxWall('P',7);
    delaySec(1);
    rotateCWonX();
    delaySec(1);
    for(y=0;y<CUBESIZE;y++){
      for(x=0;x<CUBESIZE;x++){
        sendVoxelVert(x,y,0,1,10);
      }
    }
    for(y=0;y<CUBESIZE;y++){
      for(x=0;x<CUBESIZE;x++){
        sendVoxelVert(x,y,7,-1,10);
      }
    }
    delaySec(1);
    clrLayer(0);
    //Q
    drawCharxWall('Q',7);
    delaySec(1);
    rotateCWonZ();
    delaySec(1);
    flipx();
    delaySec(1);
    setcubeBuffer(0);
    //R
    drawCharxWall('R',7);
    delaySec(1);
    rotateCCWonX();
    delaySec(1);
    flipz();
    delaySec(1);
    setcubeBuffer(0);
    //S
    drawCharxWall('S',7);
    delaySec(3);
    //T
    drawCharxWall('T',7);
    delaySec(3);
    //U
    drawCharxWall('U',7);
    delaySec(3);
    //V
    drawCharxWall('V',7);
    delaySec(3);
    //W
    drawCharxWall('W',7);
    delaySec(3);
    //X
    drawCharxWall('X',7);
    delaySec(3);
    //Y
    drawCharxWall('Y',7);
    delaySec(3);
    //Z
    drawCharxWall('Z',7);
    delaySec(3);
    //0
    drawCharxWall('0',7);
    delaySec(3);
    //1
    drawCharxWall('1',7);
    delaySec(3);
    //2
    drawCharxWall('2',7);
    delaySec(3);
    //3
    drawCharxWall('3',7);
    delaySec(3);
    //4
    drawCharxWall('4',7);
    delaySec(3);
    //5
    drawCharxWall('5',7);
    delaySec(3);
    //6
    drawCharxWall('6',7);
    delaySec(3);
    //7
    drawCharxWall('7',7);
    delaySec(3);
    //8
    drawCharxWall('8',7);
    delaySec(3);
    //9
    drawCharxWall('9',7);
    delaySec(3);
    
    setcubeBuffer(0);
    
    for(z=0;z<CUBESIZE;z++){
      setLayer(z);
      delaySec(3);
      clrLayer(z);
    }
    
    setcubeBuffer(0);
    delaySec(3);
    
    setcubeBuffer(0xff);
    delaySec(3);
  }
}

//interrupt on timer 2 to draw cube
ISR(TIMER2_COMPA_vect){
  int j;
  //turn off all layers
  LAYER_SELECT = 0;
  //set OEbar on ffs
  OE |= OE_MASK;
  //put the first layer data onto the ffs
  for(j=0;j<CUBESIZE;j++){
    //put the byte on to the data bus
    DATA_BUS = cubeBuffer[j][currentLayer];
    //latch the j'th ff
    //which needs to be j+1 so the 74138 has a rising edge
    ADD_BUS = (ADD_BUS & ADD_MASK_BAR) | (ADD_MASK & (j+1));
  }
  //turn the ff output on
  OE &= OE_MASK_BAR;
  //select the current layer
  LAYER_SELECT = 1 << currentLayer;
  //now go to the next layer
  currentLayer++;
  //check to see if the current layer is outside the cube size
  if(currentLayer>CUBESIZE) currentLayer = 0;
}

//do some boot time functions
//namely reset the ffs to 0 output
void boot(){
  int i;
  
  //put a low on all the ffs output
  DATA_BUS = 0;
  //cycle through all the ffs clocks
  //need cubesize+1 so the 7th row is reset
  for(i=0;i<CUBESIZE+1;i++){
    ADD_BUS = (ADD_BUS & ADD_MASK_BAR) | (i & ADD_MASK);
  }
  //now reset port a with OEbar high
  ADD_BUS = OE_MASK;
}

//delays a given number of seconds using timer 1
//i is the number of seconds to delay
void delaySec(byte i){
  byte j = 0;
  //turn on timer 1
  //16000000/1024/15625 = 1
  //15625 = B00111101 00001001 = 0x3d09
  OCR1AH = 0x3d;
  OCR1AL = 0x09;
  //set ctc mode
  //WGM13:0 = 0100
  //WGM13 = 0 WGM12 = 1 WGM01 = 0 WGM00 = 0
  //set 1024 prescaler
  //CS12:0 = 3 = B101
  //CS12 = 1 CS11 = 0 CS10 = 1
  //bits in tccr1b
  //ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
  //set ctc mode
  //bits in tccr1a
  //COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
  TCCR1A |= 0;
  
  //wait until the timer value matches ocr1a
  while(j < i){
    while((TIFR1 & B00000010) != B00000010);
    j++;
    //clear ocf1a with ISR (auto)
    TIFR1 |= B00000010;
  }
  //clear ocf1a with ISR (auto)
  TIFR1 |= B00000010;
  
  //turn off timer 1
  TCCR1B &= ~((1 << WGM12) | (1 << CS12) | (1 << CS10));
}

//delays a given number of milliseconds using timer 0
//i is the number of milliseconds to delay
void delaymSec(unsigned int i){
  unsigned int j = 0;
  //turn on timer 0
  //16000000/1024/16 = 977 Hz
  OCR0A = 15;
  //set ctc mode
  //WGM02:0 = 010
  //WGM12 = 0 WGM01 = 1 WGM00 = 0
  //set 1024 prescaler
  //CS02:0 = 101
  //CS02 = 1 CS01 = 0 CS00 = 1
  //bits in tccr0b
  //FOC0A FOC0B - - WGM02 CS02 CS01 CS00
  TCCR0B = (1 << CS02) | (1 << CS00);
  //set ctc mode
  //bits in tccr0a
  //COM0A1 COM0A0 COM0B1 COM0B0 - - WGM01 WGM00
  TCCR0A = (1 << WGM01);
  
  //wait until the timer value matches ocr1a
  //when OCF0A = 1
  //clear tcnt0 the timer counter
  TCNT0 = 0;
  while(j < i){
    while((TIFR0 & B00000010) != B00000010);
    j++;
    //clear OFC0A
    TIFR0 |= B00000010;
  }
  //clear OFC0A
  TIFR0 |= B00000010;
  
  //turn off timer 0
  TCCR0B &= ~((1 << CS02) | (1 << CS00));
}

//setup the ports as referenced at the top of the file
//also clears cubeBuffer
void initPorts(){
  //initialize port A to have outputs on the lower nibble
  DDRA = B00001111;
  //initialize port B to be outputs
  DDRB = B11111111;
  //initizlize port C to have output on PC1 and input on PC0
  DDRC |= B00000010;
  DDRC &= B11111110;
  //initialize port D to be outputs
  DDRD = B11111111;
  //clear port a and set OEbar on the ffs
  ADD_BUS = OE_MASK;
  //clear ports b and d
  DATA_BUS = 0;
  LAYER_SELECT = 0;
  //turn the green led on
  PORTC &= ~GREEN_LED_MASK;
  //clear the cubebuffer
  setcubeBuffer(0);
}

//setup timer 2 in ctc mode and enable interrupts
void initTimer2(){
  //setup timer 2 interrupt
  //16000000/128/12 = 10416.7
  //10417/8 = 1302 fps full draw rate
  //interrupt compare number 11 to give a count of 12
  OCR2A = 11;
  //set the prescaler at 128
  TCCR2B |= B00000101;
  //set ctc mode
  TCCR2A |= B00000010;
  //enable timer2 interrupt
  TIMSK2 |= B00000010;
  //global enable interrupts
  SREG |= B10000000;
}

/*
**************************************************************************
CHECKERBOARD COMMANDS
**************************************************************************
*/

//put a checkerboard layer on cubeBuffer
void checkerboardLayer(){
  int j,k;
  
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      if(j % 2 == 0) cubeBuffer[j][k] = 0xaa;
      else cubeBuffer[j][k] = 0x55;
    }
  }
}

//put a vertical checkerboard on a xwall
//j is which xwall to put the checkerboard on
//modifies cubeBuffer
void checkerboardxWall(byte j){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    if(k % 2 == 0) cubeBuffer[j][k] = 0xaa;
    else cubeBuffer[j][k] = 0x55;
  }
}

//put a checkerboard on a ywall
//i is the ywall to put the checkerboard on
//modifies cubeBuffer
void checkerboardyWall(byte i){
  int j,k;
  
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      if(k % 2 == 0) cubeBuffer[j][k] = (1 << i) & 0xaa;
      else cubeBuffer[j][k] = (1 << i) & 0x55;
    }
  }
}

/*
**************************************************************************
CLEAR COMMANDS
**************************************************************************
*/

//clear a column of leds
//i is the row x position
//j is the row y position
//modifies (j,0:7) of cubeBuffer to be 0 at the ith bit in x
void clrCol(byte i,byte j){
  int k;
  for(k=0;k<CUBESIZE;k++){
    cubeBuffer[j][k] &= ~(1 << i);
  }
}

//clears a layer of leds
//i is the z layer to clear
//modifies cubeBuffer to have 0 in [0:7][i]
void clrLayer(byte i){
  int j;
  for(j=0;j<CUBESIZE;j++){
    cubeBuffer[j][i] = 0;
  }
}

//turn off a single led in the cube
//i is the x direction (max of 7)
//j is the y direction (max of 7)
//k is the z direction (max of 7)
//modifies cubeBuffer to have a 0 bit in the ith position of [j][k]
void clrVoxel(byte i,byte j,byte k){
  cubeBuffer[j][k] &= ~(1 << i);
}

//clears a row of leds
//i is the y row
//j is the z layer
//modifies cubeBuffer to have 0 in [i][j]
void clrxRow(byte i,byte j){
  cubeBuffer[i][j] = 0;
}

//clears a verticle layer of leds
//i is the x row to clear
//modifies cubeBuffer so 0s are in [i][0:7]
void clrxWall(byte i){
  int j;
  for(j=0;j<CUBESIZE;j++){
    cubeBuffer[i][j] = 0;
  }
}

//clears a row of leds
//i is the x row
//k is the z layer
//modifies cubeBuffer
void clryRow(byte i,byte k){
  int j;
  
  for(j=0;j<CUBESIZE;j++){
    clrVoxel(i,j,k);
  }
}

//clears a verticle layer of leds
//i is the y row to clear
//modifies cubeBuffer so 0s are in [j][0:7]
void clryWall(byte i){
  int j,k;
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      cubeBuffer[j][k] &= ~(1 << i);
    }
  }
}

/*
**************************************************************************
COPY T0/FROM COMMANDS
**************************************************************************
*/

//copies a layer from tmpBuffer to cubeBuffer
//layertmp is which layer to copy into cubeBuffer from tmpBuffer
//layercube is which layer to put the copied layer in cubeBuffer
void copyLayercube(byte layertmp,byte layercube){
  int i;
  
  for(i=0;i<CUBESIZE;i++){
    cubeBuffer[i][layercube] = tmpBuffer[i][layertmp];
  }
}

//copies a layer from cubeBuffer to tmpBuffer
//layercube is which layer to copy into tmpBuffer from cubeBuffer
//layertmp is which layer to copy into tmpBuffer
void copyLayertmp(byte layercube,byte layertmp){
  int i;
  
  for(i=0;i<CUBESIZE;i++){
    tmpBuffer[i][layertmp] = cubeBuffer[i][layercube];
  }
}

//copys a xwall from tmpBuffer to cubeBuffer
//xwall is the xwall to copy from tmpBuffer
//xwallcube is where to copy the xwall to in cubeBuffer
void copyxWallcube(byte xwall,byte xwallcube){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    cubeBuffer[xwallcube][k] = tmpBuffer[xwall][k];
  } 
}

//copys a xwall from cubeBuffer to tmpBuffer
//xwall is the xwall to copy from cubeBuffer
//xwalltmp is where to copy the xwall to in tmpBuffer
void copyxWalltmp(byte xwall,byte xwalltmp){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    tmpBuffer[xwalltmp][k] = cubeBuffer[xwall][k];
  } 
}

//copys a ywall from tmpBuffer to cubeBuffer
//ywall is the ywall to copy from tmpBuffer
//ywallcube is where to copy the ywall to in cubeBuffer
void copyyWallcube(byte ywall,byte ywallcube){
  int j,k;
  
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      cubeBuffer[j][k] |= ((tmpBuffer[j][k] & (1 << ywall)) >> ywall) << ywallcube;
    }
  }
}

//copys a ywall from cubeBuffer to tmpBuffer
//ywall is the xwall to copy from cubeBuffer
//ywalltmp is where to copy the ywall to in tmpBuffer
void copyyWalltmp(byte ywall,byte ywalltmp){
  int j,k;
  
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      tmpBuffer[j][k] = ((cubeBuffer[j][k] & (1 << ywall)) >> ywall) << ywalltmp;
    }
  }
}

/*
**************************************************************************
DRAW CHARACTER COMMANDS
**************************************************************************
*/

//draw a character on a layer
//chr is the ascii representation of a character from "A" to "Z"
//other letters entered will be set to "A"
//k is the layer number
void drawCharLayer(byte chr,byte k){
  int a;
  
  for(a=0;a<CUBESIZE;a++){
    cubeBuffer[a][k] = getCharByte(chr,a);
  }
}

//draw a character on a x wall
//chr is the ascii representation of a character from "A" to "Z"
//other letters entered will be set to "A"
//j is the x wall number
void drawCharxWall(byte chr,byte j){
  int a;
  
  for(a=7;a>=0;a--){
    cubeBuffer[j][a] = getCharByte(chr,(CUBESIZE-1)-a);
  }
}

//draw a character on a y wall of cubeBuffer
//chr is the ascii representation of a character from "A" to "Z"
//other letters entered will be set to "A"
//i is the y wall number
void drawCharyWall(byte chr,byte i){
  int a,b;
  
  for(a=0;a<CUBESIZE;a++){
    for(b=0;b<CUBESIZE;b++){
      cubeBuffer[b][a] = (cubeBuffer[b][a] & (~(1 << i))) | (((getCharByte(chr,b) & (1 << b)) >> b ) << i);
    }
  }
}

//read a single byte (from progmem) of a letter specified and return that byte
//chr is the ascii representation of a character from "A" to "Z" and "0" to "9"
//other letters entered will be set to "A"
//i is which byte of data to read from the specified letter
//returns the character selected's ith row
byte getCharByte(byte chr,byte i){
  //take the character entered and subtract off
  //0x41 so if an "A" is entered the result is 0
  //which picks out the letter "A" from chrNames
  //"A" = 0X41 ASCII
  byte j = 0;
  
  //if a non cap letter or other was entered just default to "A"
  if(j > 0x40 && j < 0x5b) j = chr - 0x41;
  else if(j > 0x2f && j < 0x3A) j = chr - 0x30 + 26;
  
  //if i runs outside the cube return a 0
  if(i > CUBESIZE) return(0);
  else return(pgm_read_byte_far(chrNames[j] + i));
}

/*
**************************************************************************
DRAW LINE COMMANDS
**************************************************************************
*/

//draw a line between 2 voxels
//i is the x coord of the first point
//j is the y coord of the first point
//k is the z coord of the first point
//a is the x coord of the second point
//b is the y coord of the second point
//c is the z coord of the second point
void drawLine(byte i,byte j,byte k,byte a,byte b,byte c){
  byte tmp = 0;
  char xdist,ydist,zdist;
  byte r,t,p;
  char m_y,m_z;
  
  xdist=a-i;
  ydist=b-j;
  zdist=c-k;
    
  m_y = ydist/xdist;
  m_z = zdist/xdist;
  
  //if both voxels are on make a line between them of on
  if(getVoxelStatecube(i,j,k) && getVoxelStatecube(a,b,c)){
    for(r=0;r<CUBESIZE;r++){
      for(t=0;t<CUBESIZE;t++){
        for(p=0;p<CUBESIZE;p++){
          
        }
      }
    }
  }
  //if both voxels are off make a line between them of off
  else if(!getVoxelStatecube(i,j,k) && !getVoxelStatecube(a,b,c)){
    
  }
}
  
/*
**************************************************************************
FLIP COMMANDS
**************************************************************************
*/

//swap rows in the xdir for the entire cube
//modifies cubeBuffer
void flipx(){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    mirrorLayerx(k);
  }
}

//swap rows in the ydir for the entire cube
//modifies cubeBuffer
void flipy(){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    mirrorLayery(k);
  }
}

//swap rows in the zdir for the entire cube
//modifies cubeBuffer
void flipz(){
  int j;
  
  for(j=0;j<CUBESIZE;j++){
    mirrorxWall(j);
  }
}

/*
**************************************************************************
VOXEL STATE COMMANDS
**************************************************************************
*/

//figure out if a voxel is on in the cubeBuffer
//i is the x coord
//j is the y coord
//k is the z coord 
//    of the led
//returns true or false - true being the led is on
boolean getVoxelStatecube(byte i,byte j,byte k){
  if(cubeBuffer[j][k] & (1 << i)) return(true);
  else return(false);
}

/*
**************************************************************************
INVERT COMMANDS
**************************************************************************
*/

//invert a column of leds in cubeBuffer
//i is the x coord
//j is the y coord
//     of the led
void invertCol(byte i,byte j){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    invertVoxel(i,j,k);
  }
}

//invert the entire cubeBuffer
void invertCube(){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    invertLayer(k);
  }
}

//invert a layer of leds in cubeBuffer
//k is the layer to invert
void invertLayer(byte k){
  int i;
  
  for(i=0;i<CUBESIZE;i++){
    invertxRow(i,k);
  }
}

//flip a voxel state in cubeBuffer
//i is the x coord
//j is the y coord
//k is the z coord 
//    of the led
void invertVoxel(byte i,byte j,byte k){
  boolean state = getVoxelStatecube(i,j,k);
  
  if(state) clrVoxel(i,j,k);
  else setVoxel(i,j,k);
}

//invert a row of leds in cubeBuffer
//j is the y coord
//k is the layer
//     of the led
void invertxRow(byte j,byte k){
  int i;
  
  for(i=0;i<CUBESIZE;i++){
    invertVoxel(i,j,k);
  }
}

//invert a xwall in cubeBuffer
//j is the xwall to invert
void invertxWall(byte j){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    invertxRow(j,k);
  }
}

//invert a yrow of leds in cubeBuffer
//i is the x coord
//k is the layer
//     of the led
void invertyRow(byte i,byte k){
  int j;
  
  for(j=0;j<CUBESIZE;j++){
    invertVoxel(i,j,k);
  }
}

//invert a ywall in cubeBuffer
//i is the ywall to invert
void invertyWall(byte i){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    invertyRow(i,k);
  }
}

/*
**************************************************************************
MIRROR COMMANDS
**************************************************************************
*/

//mirrors a column
//0 -> 7 1 -> 6 etc
//i is the x coord
//j is the y coord
//modifies cubeBuffer
void mirrorCol(byte i,byte j){
  int k;
  byte tmp=0;
  
  //build the column byte
  for(k=0;k<CUBESIZE;k++){
    tmp |= ((cubeBuffer[j][k] & (1 << i)) >> i) << k;
  }
  
  clrCol(i,j);
  
  //put the tmp byte back onto the column
  //only backwards
  for(k=0;k<CUBESIZE;k++){
    cubeBuffer[j][k] = (cubeBuffer[j][k] & (~(1 << i))) | (((tmp & (1 << ((CUBESIZE-1)-k))) >> ((CUBESIZE-1)-k)) << i);
  }
}

//mirror a layer in the xdir
//k is the layer to mirror
//modifies cubeBuffer
void mirrorLayerx(byte k){
  int j;
  
  for(j=0;j<CUBESIZE;j++){
    mirrorxRow(j,k);
  }
}

//mirror a layer in the ydir
//k is the layer to mirror
//modifies cubeBuffer
void mirrorLayery(byte k){
  int i;
  
  for(i=0;i<CUBESIZE;i++){
    mirroryRow(i,k);
  }
}

//mirrors an xrow
//j is the y coord
//k is the layer
//modifies cubeBuffer
void mirrorxRow(byte j,byte k){
  int i;
  byte tmp=0;
  
  //build the xrow byte
  tmp = cubeBuffer[j][k];
  
  clrxRow(j,k);  
  
  for(i=0;i<CUBESIZE;i++){
    cubeBuffer[j][k] |= ((tmp & (1 << i)) >> i) << ((CUBESIZE-1)-i);
  }
}

//mirror the xwall
//j is the xwall to mirror
//modifies cubeBuffer
void mirrorxWall(byte j){
  int i;
  
  for(i=0;i<CUBESIZE;i++){
    mirrorCol(i,j);
  }
}

//mirrors a yrow
//i is the x coord
//k is the layer
//modifies cubeBuffer
void mirroryRow(byte i,byte k){
  int j;
  byte tmp=0;
  
  //build the tmp byte
  for(j=0;j<CUBESIZE;j++){
    //unhappy as well
    //select out the yrow
    //need to shift the bits left to form a byte
    //otherwise they all sit in the same spot
    //in particular the spot is the give i number
    //so the xrow number needs to be subtracted from the shift
    tmp |= (cubeBuffer[j][k] & (1 << i)) << (j-i);
  }
  
  clryRow(i,k);
  
  for(j=0;j<CUBESIZE;j++){
    //another horrible mess
    //pick out a bit of tmp
    //then move the bit to the left edge
    //then move the bit to the correct yrow
    cubeBuffer[j][k] |= ((tmp & (1 << ((CUBESIZE-1)-j))) >> ((CUBESIZE-1)-j)) << i;
  }
}

//mirrors a ywall
//j is the ywall to mirror
//modifies cubeBuffer
void mirroryWall(byte j){
  int k;
  
  for(k=0;k<CUBESIZE;k++){
    mirrorxRow(j,k);
  }
}

/*
**************************************************************************
ROTATION COMMANDS
**************************************************************************
*/

//rotate cubeBuffer from a layer to a xwall
//looking forward at 0,0 rotates left
//uses tmpBuffer to store then copies to cubeBuffer
void rotateCCWonX(){
  int j,k;
  
  settmpBuffer(0);
  
  for(j=0;j<CUBESIZE;j++){
    for(k=0;k<CUBESIZE;k++){
      tmpBuffer[(CUBESIZE-1)-k][j] = cubeBuffer[j][k];
    }
  }
  
  moveBuffercube();
}

//rotate cubeBuffer from a xwall to a layer
//looking forward at 0,0 rotates right
//uses tmpBuffer to store then copies to cubeBuffer
void rotateCWonX(){
  int j,k;
  
  settmpBuffer(0);
  
  for(j=0;j<CUBESIZE;j++){
    for(k=0;k<CUBESIZE;k++){
      tmpBuffer[k][(CUBESIZE-1)-j] = cubeBuffer[j][k];
    }
  }
  
  moveBuffercube();
}

//rotate cubeBuffer from a layer to a ywall
//looking right at 0,0 rotates right
//uses tmpBuffer to store then copies to cubeBuffer
void rotateCCWonY(){
  int i,j,k;
  
  settmpBuffer(0);
  
  for(j=0;j<CUBESIZE;j++){
    for(i=0;i<CUBESIZE;i++){
      for(k=0;k<CUBESIZE;k++){
        tmpBuffer[j][(CUBESIZE-1)-i] |= ((cubeBuffer[j][k] & (1 << i)) >> i) << k;
      }
    }
  }
  
  moveBuffercube();
}

//rotate cubeBuffer from a ywall to a layer
//looking right at 0,0 rotates left
//uses tmpBuffer to store then copies to cubeBuffer
void rotateCWonY(){
  int i,j,k;
  
  settmpBuffer(0);
  
  for(j=0;j<CUBESIZE;j++){
    for(i=0;i<CUBESIZE;i++){
      for(k=0;k<CUBESIZE;k++){
        tmpBuffer[j][i] |= ((cubeBuffer[j][k] & (1 << i)) >> i) << ((CUBESIZE-1)-k);
      }
    }
  }
  
  moveBuffercube();
}

//rotates the cubeBuffer from a x wall to a y wall
//looking down on 0,0 rotate left
//uses tmpBuffer to store then copies to cubeBuffer
void rotateCCWonZ(){
  int i,j,k;
  
  settmpBuffer(0);
  
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      for(i=0;i<CUBESIZE;i++){
        //awful
        //pick out each bit in the row
        //move the bit to the 0 position
        //then shift the bit back over the required number of places
        tmpBuffer[(CUBESIZE-1)-i][k] |= ((cubeBuffer[j][k] & (1 << i)) >> i) << j;
      }
    }
  }
  
  moveBuffercube();
}

//rotates the cubeBuffer from a y wall to a x wall
//looking down on 0,0 rotate right
//uses tmpBuffer to store then copies to cubeBuffer
void rotateCWonZ(){
  int i,j,k;
  
  settmpBuffer(0);
  
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      for(i=0;i<CUBESIZE;i++){
        //awful
        //pick out each bit in the row
        //move the bit to the 0 position
        //then shift the bit back over the required number of places
        tmpBuffer[i][k] |= ((cubeBuffer[j][k] & (1 << i)) >> i) << ((CUBESIZE-1)-j);
      }
    }
  }
  
  moveBuffercube();
}

/*
**************************************************************************
SWAP BUFFER COMMANDS
**************************************************************************
*/

//copy tmpBuffer to cubeBuffer
void moveBuffercube(){
  byte i,j;
  
  //disable interrupts so a partial cube is not drawn
  cli();
  
  for(j=0;j<CUBESIZE;j++){
    for(i=0;i<CUBESIZE;i++){
      cubeBuffer[i][j] = tmpBuffer[i][j];
    }
  }
  
  //enable interrupts
  sei();
}

//copy cubeBuffer to tmpBuffer
void moveBuffertmp(){
  byte i,j;
  
  for(j=0;j<CUBESIZE;j++){
    for(i=0;i<CUBESIZE;i++){
      tmpBuffer[i][j] = cubeBuffer[i][j];
    }
  }
}

/*
**************************************************************************
SEND VOXEL COMMANDS
**************************************************************************
*/

//move a voxel to the edge of the cube in the x direction
//left or right in steps
//rain or flow idea
//i is the x coord of the voxel
//j is the y coord of the voxel
//k is the z coord of the voxel
//dir is the direction
//positive # is back
//negative # is front
//time is the delay in between voxel movements in msec
//keeps original pattern of voxels on the row 
//(except the given point and the selected end)
//modifies cubeBuffer
void sendVoxelHoriX(byte i,byte j,byte k,char dir,unsigned int time){
  //a must be a char here since one of the loops subtracts a
  char a;
  //save the initial state of the row
  byte tmp=cubeBuffer[j][k];
  
  //if going in the positive direction
  if(dir > 0){
    //if the voxel to send is on
    if(getVoxelStatecube(i,j,k)){
      //turn the voxel to send off
      tmp &= ~(1 << i);
      //move the voxel in space with a delay
      for(a=i;a<CUBESIZE;a++){
        cubeBuffer[j][k] = tmp | (1 << a);
        delaymSec(time);
      }
    }
    else{
      //if the voxel to send is off
      //move a 0 in space with delay
      for(a=i;a<CUBESIZE;a++){
        cubeBuffer[j][k] = tmp & ~(1 << a);
        delaymSec(time);
      }
    }
  }
  //if going in the negative direction
  else if(dir < 0){
    //if the voxel is on initially
    if(getVoxelStatecube(i,j,k)) {
      //turn the ini voxel off
      tmp &= ~(1 << i);
      //move the voxel in space with delay
      for(a=i;a>=0;a--){
        cubeBuffer[j][k] = tmp | (1 << a);
        delaymSec(time);
      }
    }
    //if the voxel to move is off
    else{
      //move a 0 in space with delay
      for(a=i;a>=0;a--){
        cubeBuffer[j][k] = tmp & ~(1 << a);
        delaymSec(time);
      }
    }
  }
}

//move a voxel to the edge of the cube in the y direction
//front or back in steps
//rain or flow idea
//i is the x coord of the voxel
//j is the y coord of the voxel
//k is the z coord of the voxel
//dir is the direction
//positive # is back
//negative # is front
//time is the delay in between voxel movements in msec
//keeps original pattern of voxels on the row 
//(except the given point and the selected end)
//modifies cubeBuffer
void sendVoxelHoriY(byte i,byte j,byte k,char dir,unsigned int time){
  char a;
  byte b;
  //temp storage
  byte tmp = 0;
  
  //fill the temp storage with the yrow
  for(b=0;b<CUBESIZE;b++){
    tmp |= ((cubeBuffer[b][k] & (1 << i)) << b) >> i;
  }
  
  //going in the postive direction -> back
  if(dir > 0){
    //if the voxel to send is on
    if(getVoxelStatecube(i,j,k)){
      //turn the ini voxel off
      tmp &= ~(1 << j);
      //turn the last voxel on
      tmp |= (1 << (CUBESIZE-1));
      
      //move the voxel in space with delay
      for(a=j;a<CUBESIZE;a++){
        cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) | (1 << i);
        delaymSec(time);
        cubeBuffer[(byte)a][k] = sndVoxHoriYLogic(i,k,tmp,a);
        //cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    }
    else{
      //if the ini voxel is off
      //clr the last voxel
      tmp &= ~(1 << (CUBESIZE-1));
      //move an off in space with delay
      //move the voxel in space with delay
      for(a=j;a<CUBESIZE;a++){
        cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) & (~(1 << i));
        delaymSec(time);
        cubeBuffer[(byte)a][k] = sndVoxHoriYLogic(i,k,tmp,a);
        //cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    }
  }
  //going in the negaive direction forward
  else if(dir < 0){
    //if the voxel to send is on
    if(getVoxelStatecube(i,j,k)){
      //turn the ini voxel off
      tmp &= ~(1 << j);
      //turn the last voxel on
      tmp |= (1 << 0);
      
      //move the voxel in space with delay
      for(a=j;a>=0;a--){
        cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) | (1 << i);
        delaymSec(time);
        cubeBuffer[(byte)a][k] = sndVoxHoriYLogic(i,k,tmp,a);
        //cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    }
    else{
      //if the ini voxel is off
      //clr the last voxel
      tmp &= ~(1 << 0);
      //move an off in space with delay
      //move the voxel in space with delay
      for(a=j;a>=0;a--){
        cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) & (~(1 << i));
        delaymSec(time);
        cubeBuffer[(byte)a][k] = sndVoxHoriYLogic(i,k,tmp,a);
        //cubeBuffer[(byte)a][k] = (cubeBuffer[(byte)a][k] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    }
  }
}

//move a voxel to the edge of the cube in the z direction
//top or bottom in steps
//rain or flow idea
//i is the x coord of the voxel
//j is the y coord of the voxel
//k is the z coord of the voxel
//dir is the direction
//positive # is up
//negative # is down
//time is the delay in between voxel movements in msec
//keeps original pattern of voxels on the row 
//(except the given point and the selected end)
//modifies cubeBuffer
void sendVoxelVert(byte i,byte j,byte k,char dir,unsigned int time){
  char a;
  byte b;
  
  //temp storage
  byte tmp = 0;
  
  //fill the tmp storage
  for(b=0;b<CUBESIZE;b++){
    tmp |= ((cubeBuffer[j][b] & (1 << i)) >> i) << b;
  }
  
  //if the voxel needs to go up
  if(dir > 0){
    //if the voxel is on ini
    if(getVoxelStatecube(i,j,k)){
      //clr the ini voxel
      tmp &= ~(1 << k);
      //set the end voxel
      tmp |= 1 << (CUBESIZE-1); 
      for(a=k;a<CUBESIZE;a++){
        cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) | (1 << i);
        delaymSec(time);
        cubeBuffer[j][(byte)a] = sndVoxVertLogic(i,j,tmp,a);
        //cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    }
    //if the voxel is off ini
    else{
      //clr the end voxel
      tmp &= ~(1 << (CUBESIZE-1));
      for(a=k;a<CUBESIZE;a++){
        cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) & (~(1 << i));
        delaymSec(time);
        cubeBuffer[j][(byte)a] = sndVoxVertLogic(i,j,tmp,a);
        //cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    } 
  }
  //if the voxel needs to go down
  else if(dir < 0){
    //if the voxel is on ini
    if(getVoxelStatecube(i,j,k)){
      //clr the ini voxel
      tmp &= ~(1 << k);
      //set the end voxel
      tmp |= 1 << 0;
      for(a=k;a<CUBESIZE;a--){
        cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) | (1 << i);
        delaymSec(time);
        cubeBuffer[j][(byte)a] = sndVoxVertLogic(i,j,tmp,a);
        //cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    }
    //if the voxel is off ini
    else{
      //clr the end voxel
      tmp &= ~(1 << (CUBESIZE-1));
      for(a=k;a<CUBESIZE;a--){
        cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) & (~(1 << i));
        delaymSec(time);
        cubeBuffer[j][(byte)a] = sndVoxVertLogic(i,j,tmp,a);
        //cubeBuffer[j][(byte)a] = (cubeBuffer[j][(byte)a] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i);
      }
    }
  }
}

/*
**************************************************************************
SEND VOXEL HELPER COMMANDS
**************************************************************************
*/

//HORIY
//(cubeBuffer[(byte)a][k] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i)
byte sndVoxHoriYLogic(byte i,byte k,byte tmp,char a){
 return((cubeBuffer[(byte)a][k] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i)); 
}

//VERT
//(cubeBuffer[j][(byte)a] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i)
byte sndVoxVertLogic(byte i,byte j,byte tmp,char a){
  return((cubeBuffer[j][(byte)a] & (~(1 << i))) | (((tmp & (1 << a)) >> a) << i));
}

/*
**************************************************************************
SET COMMANDS
**************************************************************************
*/

//set a column of leds
//i is the row x position
//j is the row y position
//modifies (j,0:7) of cubeBuffer to be 1 at the ith bit in x
void setCol(byte i,byte j){
  int k;
  for(k=0;k<CUBESIZE;k++){
    cubeBuffer[j][k] |= 1 << i;
  }
}

//set the cubeBuffer to a given value
void setcubeBuffer(byte value){
  int i;
  int j;
  for(i=0;i<CUBESIZE;i++){
    for(j=0;j<CUBESIZE;j++){
      cubeBuffer[i][j] = value;
    }
  }
}

//sets a layer of leds
//i is the z layer to clear
//modifies cubeBuffer to have 0xff in [0:7][i]
void setLayer(byte i){
  int j;
  for(j=0;j<CUBESIZE;j++){
    cubeBuffer[j][i] = B11111111;
  }
}

//set the tmpBuffer to a given value
void settmpBuffer(byte value){
  int i;
  int j;
  for(i=0;i<CUBESIZE;i++){
    for(j=0;j<CUBESIZE;j++){
      tmpBuffer[i][j] = value;
    }
  }
}

//turn on a single led in the cube
//i is the x direction (max of 7)
//j is the y direction (max of 7)
//k is the z direction (max of 7)
//modifies cubeBuffer to have a 1 in the ith position of [j][k]
void setVoxel(byte i,byte j, byte k){
  cubeBuffer[j][k] |= 1 << i;
}

//sets a row of leds
//i is the y row
//j is the z layer
//modifies cubeBuffer to have 0xff in [i][j]
void setxRow(byte i, byte j){
  cubeBuffer[i][j] = B11111111;
}

//sets a verticle layer of leds
//i is the x row to set
//modifies cubeBuffer so 0xff is on [i][0:7]
void setxWall(byte i){
  int j;
  for(j=0;j<CUBESIZE;j++){
    cubeBuffer[i][j] = B11111111;
  }
}

//sets a row of leds
//i is the x row
//k is the z layer
//modifies cubeBuffer to have 0xff in [i][j]
void setyRow(byte i, byte k){
  int j;
  
  for(j=0;j<CUBESIZE;j++){
    setVoxel(i,j,k);
  }
}

//sets a verticle layer of leds
//i is the y row to set
//modifies cubeBuffer so 0xff is on [i][0:7]
void setyWall(byte i){
  int j,k;
  for(k=0;k<CUBESIZE;k++){
    for(j=0;j<CUBESIZE;j++){
      cubeBuffer[j][k] |= 1 << i; 
    }
  }
}

/*
**************************************************************************
SHIFT COMMANDS
**************************************************************************
*/

//shift a layer up or down
//wraps around
//this function is mainly for font shifts through the cube
//the function will overwrite the layer shifted to
//layer is the layer to shift
//n is how many layers to shift up or down
//positive n's are from 0 -> 1
//negative n's are from 1 -> 0
//modifies cubeBuffer and tmpBuffer
void shiftLayer(byte layer,int n){
  settmpBuffer(0);
  
  //copy layer to 0 in tmp
  copyLayertmp(layer,0);
  
  //clr the layer to move to 
  if(layer + n > 8) layer = 0;
  else if(layer + n < 0) layer = 7;
  else layer += n;
  
  clrLayer(layer);
  
  //recopy to the new layer selected
  copyLayercube(0,layer);
}

//shifts a xwall a number of walls over
//wraps around
//this function is mainly for font shifts through the cube
//the function will overwrite the xwall shifted to
//xwall is which xwall to move over
//n is how many walls to shift over 
//positive n's are from 0 -> 1
//negative n's are from 1 -> 0
//modifes cubeBuffer and tmpBuffer
void shiftxWall(byte xwall,int n){   
   settmpBuffer(0);
   
   //copy xwall to xwall0 in tmpBuffer
   copyxWalltmp(xwall,0);
   
   //clr the xwall in cubeBuffer
   if(xwall + n > 8) xwall = 0;
   else if(xwall + n < 0) xwall = CUBESIZE - 1;
   else xwall += n;
   
   clrxWall(xwall);
   
   //recopy xwall to new location in cubeBuffer
   copyxWallcube(0,xwall);
} 

//shifts a ywall a number of walls over
//wraps around
//this function is mainly for font shifts through the cube
//the function will overwrite the ywall shifted to
//ywall is which ywall to move over
//n is how many walls to shift over 
//positive n's are from 0 -> 1
//negative n's are from 1 -> 0
//modifes cubeBuffer and tmpBuffer
void shiftyWall(byte ywall,int n){   
   settmpBuffer(0);
   
   //copy xwall to xwall0 in tmpBuffer
   copyyWalltmp(ywall,0);
   
   //clr the ywall in cubeBuffer
   if(ywall + n > 8) ywall = 0;
   else if(ywall + n < 0) ywall = CUBESIZE - 1;
   else ywall += n;
   
   clryWall(ywall);
   
   //recopy ywall to new location in cubeBuffer
   copyyWallcube(0,ywall);
} 
