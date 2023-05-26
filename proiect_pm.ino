#include "Arduino.h"
#if !defined(SERIAL_PORT_MONITOR)
#error "Arduino version not supported. Please update your IDE to the latest version."
#endif

#if defined(__SAMD21G18A__)
// Shield Jumper on HW (for Zero, use Programming Port)
#define port SERIAL_PORT_HARDWARE
#define pcSerial SERIAL_PORT_MONITOR
#elif defined(SERIAL_PORT_USBVIRTUAL)
// Shield Jumper on HW (for Leonardo and Due, use Native Port)
#define port SERIAL_PORT_HARDWARE
#define pcSerial SERIAL_PORT_USBVIRTUAL
#else
// Shield Jumper on SW (using pins 12/13 or 8/9 as RX/TX)
#include "SoftwareSerial.h"
SoftwareSerial port(12, 13);
#define pcSerial SERIAL_PORT_MONITOR
#endif

#include "EasyVR.h"
#include <Adafruit_NeoPixel.h>

EasyVR easyvr(port);

//Groups and Commands
enum Groups {
  GROUP_0 = 0,
  GROUP_1 = 1,
};

enum Group0 {
  G0_LANTERN = 0,
};

enum Group1 {
  G1_WHITE = 0,
  G1_RED = 1,
  G1_PURPLE = 2,
  G1_YELLOW = 3,
  G1_GREEN = 4,
  G1_AZURE = 5,
  G1_INCREASE_BRIGHTNESS = 6,
  G1_HELP = 7,
  G1_STOP = 8,
  G1_REDUCE_LIGHT = 9,
  G1_LIGHT = 10,
  G1_ADAPT = 11,
  G1_OFF = 12,
  G1_PARTY = 13,
  G1_MERI_MII = 14
};

// use negative group for wordsets
int8_t group, idx;

// Pin definitions
#define PIN_PIR 4
#define PIN_BUZZER 5
#define PIN_LEDS 6
#define PIN_PHOTO_A0 A0
#define PIN_PHOTO_A1 A1
#define PIN_PHOTO_A2 A2
#define PIN_PHOTO_A3 A3

#define RX_PIN 12
#define TX_PIN 13

#define LED_COUNT 7

// Global variables to store the state of the system
bool isSOS = false;
bool isAdapting = false;
bool isLightOn = false;
int brightness = 50;               // Initialize brightness
int color[3] = { 255, 255, 255 };  // Initialize color (white)

unsigned long lastMotionDetectedTime = 0;
unsigned long previousTime = 0;            // Variable to store the time
unsigned long adaptationInterval = 10000;  // Interval for adaptation mode delay
unsigned long lastPrintTime = 0;           // Keep track of the last time we printed

// Time intervals for different parts of the signal
struct MorseSymbol {
  unsigned long duration;
  bool state;
};

const MorseSymbol SOS[] = {
  { 300, HIGH }, { 300, LOW }, { 300, HIGH }, { 300, LOW }, { 300, HIGH }, { 300, LOW },  // S: ...
  { 700, HIGH }, { 300, LOW }, { 700, HIGH }, { 300, LOW }, { 700, HIGH }, { 300, LOW },  // O: ---
  { 300, HIGH }, { 300, LOW }, { 300, HIGH }, { 300, LOW }, { 300, HIGH }, { 300, LOW },   // S: ...
  { 60000, LOW }  // Pause for a minute
};

const int SOS_LENGTH = sizeof(SOS) / sizeof(MorseSymbol);

volatile int sosState = 0;
unsigned long sosStartTime = 0;

// Initialize LED ring
Adafruit_NeoPixel pixels(LED_COUNT, PIN_LEDS, NEO_GRB + NEO_KHZ800);

void setup() {
  // setup PC serial port
  pcSerial.begin(9600);
bridge:
  // bridge mode?
  int mode = easyvr.bridgeRequested(pcSerial);
  switch (mode) {
    case EasyVR::BRIDGE_NONE:
      // setup EasyVR serial port
      port.begin(9600);
      // run normally
      pcSerial.println(F("Bridge not requested, run normally"));
      pcSerial.println(F("---"));
      break;

    case EasyVR::BRIDGE_NORMAL:
      // setup EasyVR serial port (low speed)
      port.begin(9600);
      // soft-connect the two serial ports (PC and EasyVR)
      easyvr.bridgeLoop(pcSerial);
      // resume normally if aborted
      pcSerial.println(F("Bridge connection aborted"));
      pcSerial.println(F("---"));
      break;

    case EasyVR::BRIDGE_BOOT:
      // setup EasyVR serial port (high speed)
      port.begin(115200);
      pcSerial.end();
      pcSerial.begin(115200);
      // soft-connect the two serial ports (PC and EasyVR)
      easyvr.bridgeLoop(pcSerial);
      // resume normally if aborted
      pcSerial.println(F("Bridge connection aborted"));
      pcSerial.println(F("---"));
      break;
  }

  // initialize EasyVR
  while (!easyvr.detect()) {
    pcSerial.println(F("EasyVR not detected!"));
    for (int i = 0; i < 10; ++i) {
      if (pcSerial.read() == '?')
        goto bridge;
      delay(100);
    }
  }

  pcSerial.print(F("EasyVR detected, version "));
  pcSerial.print(easyvr.getID());

  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, LOW);  // Shield 2.0 LED off

  if (easyvr.getID() < EasyVR::EASYVR)
    pcSerial.print(F(" = VRbot module"));
  else if (easyvr.getID() < EasyVR::EASYVR2)
    pcSerial.print(F(" = EasyVR module"));
  else if (easyvr.getID() < EasyVR::EASYVR3)
    pcSerial.print(F(" = EasyVR 2 module"));
  else
    pcSerial.print(F(" = EasyVR 3 module"));
  pcSerial.print(F(", FW Rev."));
  pcSerial.println(easyvr.getID() & 7);

  easyvr.setDelay(0);  // speed-up replies

  easyvr.setTimeout(5);
  easyvr.setLanguage(0);  //<-- same language set on EasyVR Commander when code was generated

  group = EasyVR::TRIGGER;  //<-- start group

  // Initialize LEDs
  pixels.begin();
  pixels.setBrightness(50);  // Set initial brightness (0-255)
  pixels.show();             // Initialize all pixels to 'off'

  // Initialize PIR sensor
  pinMode(PIN_PIR, INPUT);

  // Initialize photoresistors
  pinMode(PIN_PHOTO_A0, INPUT);
  pinMode(PIN_PHOTO_A1, INPUT);
  pinMode(PIN_PHOTO_A2, INPUT);
  pinMode(PIN_PHOTO_A3, INPUT);

  // Initialize Buzzer
  pinMode(PIN_BUZZER, OUTPUT);

  Serial.print("Setup exit!");
}

void checkSOS() {
  if (isSOS) {
    if (millis() - sosStartTime > SOS[sosState].duration) {
      sosState = (sosState + 1) % SOS_LENGTH;  // Move to the next state, wrapping back to 0 if we've completed the sequence
      sosStartTime = millis();                 // Start the timer for the next state
    }

    bool state = SOS[sosState].state;
    digitalWrite(PIN_BUZZER, state);  // Turn buzzer on or off depending on the state
    if (state) {
      pixels.fill(pixels.Color(255, 0, 0), 0, LED_COUNT);  // Turn on LEDs
    } else {
      pixels.clear();  // Turn off LEDs
    }
    pixels.show();
  }
}

void setColor(int r, int g, int b) {
  color[0] = r;
  color[1] = g;
  color[2] = b;
  if (isLightOn) {
    pixels.fill(pixels.Color(color[0], color[1], color[2]), 0, LED_COUNT);
    pixels.show();
  }
}

void adjustBrightness(int adjustment) {
  brightness = max(0, min(255, brightness + adjustment));
  pixels.setBrightness(brightness);
  pixels.fill(pixels.Color(color[0], color[1], color[2]), 0, LED_COUNT);
  pixels.show();
}

void loop() {

  // playMelody();

  // Check for motion
  if (digitalRead(PIN_PIR) == HIGH) {
    lastMotionDetectedTime = millis();
  }

  // If no motion was detected for 30 seconds, turn off the light
  if (isLightOn && millis() - lastMotionDetectedTime > 30000) {
    pixels.clear();  // Turn off LEDs
    pixels.show();
    isLightOn = false;
  }

  checkSOS();

  // If adaptation mode is active, adjust brightness based on LDR readings
  if (isAdapting) {
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= adaptationInterval) {
      int LDRs[] = { PIN_PHOTO_A0, PIN_PHOTO_A1, PIN_PHOTO_A2, PIN_PHOTO_A3 };
      int total = 0;
      for (int i = 0; i < 4; i++) {
        total += analogRead(LDRs[i]);
      }
      int average = total / 4;
      brightness = map(average, 1010, 1023, 255, 10);                         // Map LDR reading to brightness value
      pixels.fill(pixels.Color(color[0], color[1], color[2]), 0, LED_COUNT);  // Change LEDs color
      pixels.setBrightness(brightness);
      pixels.show();
      previousTime = currentTime;
    }
  }

  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, HIGH);  // LED on (listening)

  if (group < 0)  // SI wordset/grammar
  {
    pcSerial.print("Say a word in Wordset ");
    pcSerial.println(-group);
    easyvr.recognizeWord(-group);
  } else  // SD group
  {
    pcSerial.print("Say a command in Group ");
    pcSerial.println(group);
    easyvr.recognizeCommand(group);
  }

  do {
    // allows Commander to request bridge on Zero (may interfere with user protocol)
    if (pcSerial.read() == '?') {
      setup();
      return;
    }
    // <<-- can do some processing here, while the module is busy
  } while (!easyvr.hasFinished());

  easyvr.setLevel(1);
  easyvr.setMicDistance(2);

  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, LOW);  // LED off

  idx = easyvr.getWord();
  if (idx == 0 && group == EasyVR::TRIGGER) {
    
    // print debug message
    pcSerial.println("Word: ROBOT");
   
    return;
  } else if (idx >= 0) {
    
    // print debug message
    uint8_t flags = 0, num = 0;
    char name[32];
    pcSerial.print("Word: ");
    pcSerial.print(idx);
    if (easyvr.dumpGrammar(-group, flags, num)) {
      for (uint8_t pos = 0; pos < num; ++pos) {
        if (!easyvr.getNextWordLabel(name))
          break;
        if (pos != idx)
          continue;
        pcSerial.print(F(" = "));
        pcSerial.println(name);
        break;
      }
    }
    
    action();
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0) {
    // beep
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // print debug message
    uint8_t train = 0;
    char name[32];
    pcSerial.print("Command: ");
    pcSerial.print(idx);
    if (easyvr.dumpCommand(group, idx, name, train)) {
      pcSerial.print(" = ");
      pcSerial.println(name);
    } else
      pcSerial.println();
    // perform some action
    action();
  } else  // errors or timeout
  {
    if (easyvr.isTimeout())
      pcSerial.println("Timed out, try again...");
    int16_t err = easyvr.getError();
    if (err >= 0) {
      pcSerial.print("Error ");
      pcSerial.println(err, HEX);
    }
  }
}

void action() {

  Serial.println("In action");
  switch (group) {
    case GROUP_0:
      switch (idx) {
        case G0_LANTERN:
          group = GROUP_1;
          break;
      }
      break;
    case GROUP_1:
      switch (idx) {
        case G1_WHITE:
          setColor(255, 255, 255);
          break;
        case G1_RED:
          setColor(255, 0, 0);
          break;
        case G1_PURPLE:
          setColor(128, 0, 128);
          break;
        case G1_YELLOW:
          setColor(255, 255, 0);
          break;
        case G1_GREEN:
          setColor(0, 255, 0);
          break;
        case G1_AZURE:
          setColor(0, 255, 255);
          break;
        case G1_INCREASE_BRIGHTNESS:
          adjustBrightness(51);  // Increase brightness by 20%
          break;
        case G1_HELP:
          isSOS = true;
          break;
        case G1_STOP:
          isSOS = false;
          isAdapting = false;
          break;
        case G1_REDUCE_LIGHT:
          adjustBrightness(-51);  // Decrease brightness by 20%
          break;
        case G1_LIGHT:
          isLightOn = true;
          pixels.fill(pixels.Color(color[0], color[1], color[2]), 0, LED_COUNT);  // Turn on LEDs with current color
          pixels.show();
          break;
        case G1_ADAPT:
          isAdapting = true;
          break;
        case G1_PARTY:
          partyMode();
          break;
        case G1_OFF:
          isLightOn = false;
          isAdapting = false;
          isSOS = false;
          pixels.clear();  // Turn off LEDs
          pixels.show();
          break;
        case G1_MERI_MII:
          playMelody(2);
          break;
      }
      break;
  }
}

void partyMode() {
  const int cycles = 1;       // Number of cycles
  const int delayTime = 500;  // Time to delay in milliseconds

  // Define colors
  uint32_t colors[] = {
    pixels.Color(255, 0, 0),     // Red
    pixels.Color(255, 255, 0),   // Yellow
    pixels.Color(0, 255, 0),     // Green
    pixels.Color(128, 0, 128),   // Purple
    pixels.Color(0, 255, 255),   // Azure
    pixels.Color(255, 255, 255)  // White
  };

  const int colorCount = sizeof(colors) / sizeof(uint32_t);  // Get number of colors

  // Cycle through colors
  for (int cycle = 0; cycle < cycles; cycle++) {
    for (int colorIndex = 0; colorIndex < colorCount; colorIndex++) {
      pixels.fill(colors[colorIndex], 0, LED_COUNT);
      pixels.show();
      playMelody(1);
    }
  }
}

/* 
  songs available at https://github.com/robsoncouto/arduino-songs                                            
*/

#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978
#define REST 0

// change this to make the song slower or faster
int tempo = 140;

// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!

int melody[] = {

  REST, 2, NOTE_D5, 8, NOTE_B4, 4, NOTE_D5, 8,  //1
  REST, 8, NOTE_A4, 8, NOTE_FS5, 8, NOTE_E5, 4, NOTE_D5, 8

};

// i wanna merry you bruno mars
int melody2[] = {
  NOTE_D5, -8, NOTE_D5, -8, NOTE_G5, -8, NOTE_A5, -8, NOTE_A5, -8, NOTE_A5, -4, REST, -4, // Line 1
  NOTE_A5, 8, NOTE_A5, 8, NOTE_G5, 4, NOTE_F5, 8, NOTE_F5, 8, NOTE_G5, 4, NOTE_A5, 8, NOTE_D4, -4, REST, -8, // Line 2
  NOTE_A5, 8, NOTE_G5, 8, NOTE_G5, 8, NOTE_F5, -4, REST, -4, // Line 3
  NOTE_C5, 8, NOTE_F5, 8, NOTE_F5, 8, NOTE_F5, 8, NOTE_F5, 8, NOTE_G5, 8, NOTE_F5, 8, NOTE_F5, 4 // Line 4
};


// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

void playMelody(int i) {
  // iterate over the notes of the melody.
  // Remember, the array is twice the number of notes (notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;  // increases the duration in half for dotted notes
    }

    if(i == 1) {
        // we only play the note for 90% of the duration, leaving 10% as a pause
      tone(PIN_BUZZER, melody[thisNote], noteDuration * 0.9);
    } else {
      tone(PIN_BUZZER, melody2[thisNote], noteDuration * 0.9);
    }

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(PIN_BUZZER);
  }
}
