/*****************************************************************************
 **  COPYRIGHT (C) 2013, ADVANCER TECHNOLOGIES, ALL RIGHTS RESERVED.
 *****************************************************************************/

//***********************************************
// Muscle Controlled Iron Man Repulsor - 29March2013
//
// Developed by Brian E. Kaminski - Advancer Technologies, 
// www.AdvancerTechnologies.com
//
// WaveHC code borrowed from Adafruit Industries WaveHC library
// examples (indicated below).
//***********************************************


//***************************************************************************
// Example code from wavehc_play6.ino as part of WaveHC Library
//***************************************************************************
#include <FatReader.h>
#include <SdReader.h>
#include <avr/pgmspace.h>
#include "WaveUtil.h"
#include "WaveHC.h"

SdReader card;    // This object holds the information for the card
FatVolume vol;    // This holds the information for the partition on the card
FatReader root;   // This holds the information for the filesystem on the card
FatReader f;      // This holds the information for the file we're play

WaveHC wave;      // This is the only wave (audio) object, since we will only play one at a time

#define DEBOUNCE 100  // button debouncer
//***************************************************************************

// We need to setup a muscle sensor reading value which will be used to tell 
// at what level of muscle flexion we want the repulsor to be triggered.
// Adjust higher to make less sensative, adjust lower to make more sensitive
int iThreshold			= 100;    

const int iLEDPin 		= 13;      // pin of the LED on the protoboard
const int iGloveLEDPin          = 6;      // pin connected to the Glove
const int iMuscleSensorPin 	= A0;      // pin connected to the muscle sensor
bool bPreviousState		= false;
bool firstLoop                  = true;
long POWERUP_SFX_LENGTH = 1080; //ms
long POWERDWN_SFX_LENGTH = 1250; //ms
long LED_MAX = 255;


//----------------------------------------------------------------------------
//
///	SETUP
///
///	@desc	
//----------------------------------------------------------------------------
void setup() 
{
  //***************************************************************************
  // Example code from wavehc_play6.ino as part of WaveHC Library
  //***************************************************************************
  // set up serial port
  Serial.begin(9600);
  putstring_nl("WaveHC with 6 buttons");

  putstring("Free RAM: ");       // This can help with debugging, running out of RAM is bad
  Serial.println(freeRam());      // if this is under 150 bytes it may spell trouble!

  // Set the output pins for the DAC control. This pins are defined in the library
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  // enable pull-up resistors on switch pins (analog inputs)
  digitalWrite(14, HIGH);
  digitalWrite(15, HIGH);
  digitalWrite(16, HIGH);
  digitalWrite(17, HIGH);
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);

  //  if (!card.init(true)) { //play with 4 MHz spi if 8MHz isn't working for you
  if (!card.init()) {         //play with 8 MHz spi (default faster!)  
    putstring_nl("Card init. failed!");  // Something went wrong, lets print out why
    sdErrorCheck();
    while(1);                            // then 'halt' - do nothing!
  }

  // enable optimize read - some cards may timeout. Disable if you're having problems
  card.partialBlockRead(true);

  // Now we will look for a FAT partition!
  uint8_t part;
  for (part = 0; part < 5; part++) {     // we have up to 5 slots to look in
    if (vol.init(card, part)) 
      break;                             // we found one, lets bail
  }
  if (part == 5) {                       // if we ended up not finding one  :(
    putstring_nl("No valid FAT partition!");
    sdErrorCheck();      // Something went wrong, lets print out why
    while(1);                            // then 'halt' - do nothing!
  }

  // Lets tell the user about what we found
  putstring("Using partition ");
  Serial.print(part, DEC);
  putstring(", type is FAT");
  Serial.println(vol.fatType(),DEC);     // FAT16 or FAT32?

  // Try to open the root directory
  if (!root.openRoot(vol)) {
    putstring_nl("Can't open root dir!"); // Something went wrong,
    while(1);                             // then 'halt' - do nothing!
  }
  //***************************************************************************

  //Setup Muscle Sensor Shield  
  pinMode(iGloveLEDPin, OUTPUT);
  pinMode(iLEDPin, OUTPUT);
  pinMode(iMuscleSensorPin, INPUT);

  putstring_nl("Ready!");
}

//----------------------------------------------------------------------------
//
///	loop
///
///	@desc	
//----------------------------------------------------------------------------
void loop() 
{
  if(firstLoop)
  {
    playcomplete("IMPORT.WAV");		//play the J.A.R.V.I.S. "Importing Preferences" sound effect
    delay(1000);
    playcomplete("ONLINE.WAV");		//play the J.A.R.V.I.S. "Online and Ready" sound effect
    firstLoop = false;
  }

  boolean bState = ReadMuscleSensor(iMuscleSensorPin);

  if(bState && !bPreviousState)
  {
    digitalWrite(iLEDPin, HIGH);	//turn the LED on when flexing
    powerUp();		                //play the Repulsor Power Up sound effect
  }
  else if(!bState && bPreviousState)
  {
    digitalWrite(iLEDPin, LOW);	        //turn the LED off when relaxed    
    fire();				//play the Repulsor Firing and Power Down sound effects    
  }

  bPreviousState = bState;

  delay(10);
}

//----------------------------------------------------------------------------
//
///	powerUp
///
///	@desc	Plays the repulsor power up sound effect while reading the muscle sensor.
///             The sound effect will be interupted if the muscle sensor signal drops below 
///             the threshold. This allows for rapid firing of the repulsor.
///
//----------------------------------------------------------------------------
void powerUp() 
{
  playfile("PWRUP1.WAV");

  // setup fade in variables
  int fadeValue = 0;
  long currTime = millis();
  long prevTime = currTime;
  double brightnessLevel = 0.25;  // 25% brightness -- Change this to increase or lower brightness for power-up phase	
  long timeDivision = POWERUP_SFX_LENGTH/long(LED_MAX*brightnessLevel); // clip length in milliseconds divided by the desired LED value (25% brightness)

  while (wave.isplaying) 
  {
    boolean bState = ReadMuscleSensor(iMuscleSensorPin);

    // fade in from min to max over length of clip:
    currTime = millis();
    if(currTime-prevTime >= timeDivision)
    {
      fadeValue +=1;

      // update glove LEDs
      if(fadeValue<=long(LED_MAX*brightnessLevel))
        analogWrite(iGloveLEDPin, fadeValue);		   

      prevTime = currTime;
    }  	  

    if(!bState)
      wave.stop();
  }
}

//----------------------------------------------------------------------------
//
///	fire
///
///	@desc	Plays the repulsor firing sound effect while reading the muscle sensor.
///             The sound effect will be interupted if the muscle sensor signal goes above 
///             the threshold (meaning another fire sequence is being initiated. If the firing
///             sound effect is not interupted, the repulsor power down sound effect is played.
///             The power down sound effect can also be interupted. This allows for rapid firing 
///             of the repulsor.
///
//----------------------------------------------------------------------------
void fire() 
{
  double brightnessLevel = 1.0;
  int fadeValue = LED_MAX*brightnessLevel; // full brightness
  boolean bState = false;

  // turn on the glove LEDs
  analogWrite(iGloveLEDPin, fadeValue);	

  playfile("FIRE1.WAV");
  while (wave.isplaying) 
  {
    bState = ReadMuscleSensor(iMuscleSensorPin);

    if(bState)
      wave.stop();
  }

  // turn LED brightness down by 50% after firing
  brightnessLevel = 0.5;
  fadeValue *= brightnessLevel; 
  analogWrite(iGloveLEDPin, fadeValue);	// update glove LEDs brightness

  if(!bState)
  { 
    // now power down
    playfile("PWRDOWN1.WAV");

    //setup fade out variables
    long currTime = millis();
    long prevTime = currTime;
    brightnessLevel = 0.0;
    long timeDivision = POWERDWN_SFX_LENGTH/long(fadeValue*(1-brightnessLevel)); // clip length in milliseconds divided by the desired LED brightness value

    while (wave.isplaying) 
    {
      bState = ReadMuscleSensor(iMuscleSensorPin);

      // fade out from max to min over length of clip:
      currTime = millis();
      if(currTime-prevTime >= timeDivision)
      {
        fadeValue -=1;

        // update glove LEDs
        if(fadeValue>long(fadeValue*brightnessLevel))
          analogWrite(iGloveLEDPin, fadeValue);		   

        prevTime = currTime;
      }  	  

      if(bState)
        wave.stop();
    }
  }     

  analogWrite(iGloveLEDPin, 0);	// turn off the glove LEDs
}

//-----------------------------------------------------------------------------------------------------------------------------------
//
///	ReadMuscleSensor
///
///	@desc		This method reads each game button's state. If a button is pressed, it will change the button's
///		        state to true. If not, it will change the button's state to false.
///
///	@param	        iMuscleSensorPin  // Analog pin reading muscle sensor output
///
///	@return 	true if the muscle sensor value is greater than the threshold
//-----------------------------------------------------------------------------------------------------------------------------------
bool ReadMuscleSensor(int iMuscleSensorPin)
{
  int val = analogRead(iMuscleSensorPin);    

  if(val >= iThreshold)
    return true;
  else
    return false;
}

//***************************************************************************
// Example code from wavehc_play6.ino as part of WaveHC Library
//***************************************************************************
//----------------------------------------------------------------------------
//
///	freeRam
///
///	@desc	Will return the number of bytes currently free in RAM, great for debugging
///
///	@returns number of bytes currently free in RAM
//----------------------------------------------------------------------------
int freeRam(void)
{
  extern int  __bss_end; 
  extern int  *__brkval; 
  int free_memory; 
  if((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__bss_end); 
  }
  else {
    free_memory = ((int)&free_memory) - ((int)__brkval); 
  }
  return free_memory; 
} 

//----------------------------------------------------------------------------
//
///	sdErrorCheck
///
///	@desc	
//----------------------------------------------------------------------------
void sdErrorCheck(void)
{
  if (!card.errorCode()) return;
  putstring("\n\rSD I/O error: ");
  Serial.print(card.errorCode(), HEX);
  putstring(", ");
  Serial.println(card.errorData(), HEX);
  while(1);
}

//----------------------------------------------------------------------------
//
///	playcomplete
///
///	@desc	Plays a full file from beginning to end with no pause.
///
///	@param	name
//----------------------------------------------------------------------------
void playcomplete(char *name) 
{
  // call our helper to find and play this name
  playfile(name);
  while (wave.isplaying) 
  {
  }
  // now its done playing
}

//----------------------------------------------------------------------------
//
///	playfile
///
///	@desc	
///
///	@param	name
//----------------------------------------------------------------------------
void playfile(char *name) 
{
  // see if the wave object is currently doing something
  if (wave.isplaying) {// already playing something, so stop it!
    wave.stop(); // stop it
  }
  // look in the root directory and open the file
  if (!f.open(root, name)) {
    putstring("Couldn't open file "); 
    Serial.print(name); 
    return;
  }
  // OK read the file and turn it into a wave object
  if (!wave.create(f)) {
    putstring_nl("Not a valid WAV"); 
    return;
  }

  // ok time to play! start playback
  wave.play();
}
//***************************************************************************

