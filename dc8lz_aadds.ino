/***************************************************************************\

   dc8lz_aadds.ino

   Antenna Analyzer & DDS  AADDS
   Stephan Lauffer, DC8LZ

    Partly based on DDS_Sweeper.BAS from Beric Dunn (K6BEZ)
    Copyright (c) 2013,2018  CC-BY-SA
    Creative Commons Attribution-ShareAlike 3.0 Unported License

    Date    : 2018/06/15
*/
    
#define VERSION   0.22

/*    Notes : Written using for the Arduino Nano in 5 volt version

            :   Pins:
            :    A0   - Reverse Detector Analog in
            :    A1   - Forward Detector Analog in
            :    5    - Speaker
            :    6-8  - Three buttons 0 - Previous, 1 - Next, 3 - Select
            :    9-12 - Comminucate with the AD9850 DDS
            
 ***************************************************************************/
#include "U8glib.h"
#include <EEPROM.h>

// OLED display connected via A4 (SDA), A5 (SCL)

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI

// Debug via Serial...
uint8_t Debug = 2;                // 0 -> NO SERIAL (!) and no debugging, 1 -> state, key, 2 -> debug more...

// Define Pins used to control AD9850 DDS

#define FQ_UD   10
#define SDAT    11
#define SCLK    9
#define RESET   12

// Define Pins used to read voltage value, two analog pins:

#define PIN_A0  A0
#define PIN_A1  A1

// Define Pin for speaker

#define SPEAKER       3

// Define Pins for the three buttons

#define uiKeySelect   6
#define uiKeyPrev     7
#define uiKeyNext     8

// Non-Arduino pin definitions...

#define KEY_NONE    0
#define KEY_PREV    1
#define KEY_NEXT    2
#define KEY_SELECT  3

uint8_t uiKeyCodeFirst  = KEY_NONE;
uint8_t uiKeyCodeSecond = KEY_NONE;
uint8_t uiKeyCode       = KEY_NONE;

#define MENU_ITEMS  6
const char *menu_strings[MENU_ITEMS] = { "Hold", "Start Freq", "End Freq", "Sweep Steps", "Run Graph", "Run Text"};

#define INT_VALS    11
const char *int_val[INT_VALS] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" };

uint8_t menu_current  = 0;
uint8_t num_current   = 0;
uint8_t last_key_code = KEY_NONE;

char redraw_required  = 't';      // menu_redraw_required         -> m
                                  // text_redraw_required         -> t
                                  // drawIntInput_redraw_required -> i
                                  // drawGraph_redraw_required    -> g
                                  // draw nothing...              -> x

uint8_t SweepMode     = 0;        // 0 -> hold an meassure current (start) frequence and vswr
                                  // 1 -> sweep from start to stop freq (one time)
                                  // 2 -> sweep from start to stop freq (endles loop)
                                  
uint8_t In_State      = 0;        // 0 -> hold text mode , 1 -> In Menue, 2 -> Set Start F, 3 -> Set End F, 4 -> Sweep Steps, 5 -> graphic run, 6 -> text run, 7 -> Calibration

unsigned int NumSteps;            // Number of steps to use in the sweep
unsigned int Step;                // actual step in the sweep

char charBuf[9];                  // here we store the array of the values to change (f.e. Fstart_MHz or NumSteps)
uint8_t Input_Length  = 9;        // length of the charBuf. Depends on the value to change

double Fstart_MHz;                // Start Frequency for sweep. Stored in eeprom.
double Fstop_MHz;                 // Stop Frequency for sweep. Stored in eeprom.
double current_freq_MHz;          // Temp variable used during sweep.
double VSWR_Min;                  // Best SWR from the last measure(s). Stored in eeprom.
double VSWR_Max;
double Fres_MHz;                  // Frequency at the best SWR. Stored in eeprom.
double Fset_MHz       = 0;        // Temp variable...
double Fstep_MHz      = 0;        // Step length during a sweep.

double VSWR;                      // current VSWR

                                  // The measure failure is very close to a linear function.
                                  // So in calibration mode we get the curve by measure two points:
#define CalMHzMin   5             // low calibration frequence
#define CalMHzMax   20            // hight calibration frequencve
                                  // The curve is defined by:
double CalM;                      // round about 0.004827;
double CalB;                      // round abot  0.525;

#define MeasureRuns 5             // default/min runs of internal analog read measures

uint8_t SpeakerOn;                // 0 -> Speaker for tuning in hold off/on
uint8_t toneUp        = 0;        // current speaker state...

#define SpeakerFreq        5100    // the min speaker freq at best swr == 1
#define SoundPauseInteval  1000   // longest sound pausing -> worst SWR. Continously sound -> best SWR.
                                  // SoundInterval in millis.
unsigned long TimeLast  = 0;      // ...

unsigned int VSWR_Sweep[128];     // array for the SWR measures in the graph mode

long serial_input_number;         // Used to build number from serial stream

char incoming_char;               // Character read from serial stream

void calibrate() {

  // Sweep from CalMHzMin MHz to CalMHzMax MHz with a 50R dummy ends closely in a linear function.
  // So take min. and max... get the linear curve...

  double y0, y1;
  
  SetDDSFreq(CalMHzMin);
  y0 = MeasureVoltage(PIN_A0, 10);                    // average if ten measures...
  
  SetDDSFreq(CalMHzMax);
  y1 = MeasureVoltage(PIN_A0, 10);

  // find out linear function... 
  
  CalM = ( y1 - y0 ) / (CalMHzMax - CalMHzMin);       // ...slope...

  CalB = y1 - (CalM * CalMHzMax);                     // ...intercept..

  if (Debug > 1) {
      Serial.print("Debug: calibrate: y0, y1, CalM, CalB: "); Serial.print(y0,6); Serial.print (" "); Serial.print(y1,6); Serial.print (" "); Serial.print(CalM,6); Serial.print (" "); Serial.print(CalB,6); Serial.println();
  }

  updateEEPROM();

}

double fixREV(double REV) {

  if (Debug > 1 ) {
    Serial.print("Debug: fixREV: REV, Offset "); Serial.print(REV);
  }
  
  REV = (REV - ((current_freq_MHz * CalM + CalB)) ) ;
 
  if (REV < 0)
    REV = 0;

  if (Debug > 1 ) {
    Serial.print(" "); Serial.print(current_freq_MHz * CalM + CalB); Serial.println();
  }
  return REV;
}

void setFont(uint8_t Font) {

  // 0 == tiny... 3 large

  if (Font == 0)
    //u8g.setFont(u8g_font_4x6);
    u8g.setFont(u8g_font_04b_03br);
/*
  if (Font == 1)
    u8g.setFont(u8g_font_5x7);

  if (Font == 2)
    u8g.setFont(u8g_font_5x8);
*/

  if (Font == 3)
    u8g.setFont(u8g_font_6x12);

  if (Font == 4)
    u8g.setFont(u8g_font_9x15B);

  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

}

void updateEEPROM(void) {

  int eeAddress = 0;
  EEPROM.put(eeAddress, Fstart_MHz);

  eeAddress = sizeof(double);
  EEPROM.put(eeAddress, Fstop_MHz);

  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, VSWR_Min);

  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, Fres_MHz);

  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, NumSteps);

  eeAddress += sizeof(unsigned int);    // size offset is value of previous address size (!)
  EEPROM.put(eeAddress, CalM);

  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, CalB);

  eeAddress += sizeof(double);
  EEPROM.put(eeAddress, SpeakerOn);
}

void getEEPROM(void) {

  int eeAddress = 0;
  EEPROM.get(eeAddress, Fstart_MHz);

  eeAddress = sizeof(double);
  EEPROM.get(eeAddress, Fstop_MHz);

  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, VSWR_Min);

  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, Fres_MHz);

  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, NumSteps);

  eeAddress += sizeof(unsigned int);    // size offset is value of previous address size (!)
  EEPROM.get(eeAddress, CalM);

  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, CalB);

  eeAddress += sizeof(double);
  EEPROM.get(eeAddress, SpeakerOn);
  
  if ( (Fstart_MHz <= 0.000001) || (Fstart_MHz >= 99) )
    Fstart_MHz = 7;

  if ( (Fstop_MHz <= 0.000001) || (Fstop_MHz >= 99) )
    Fstop_MHz = 9;

  if (Fstart_MHz > Fstop_MHz)
    Fstop_MHz = Fstart_MHz + 1;

  if ( (NumSteps <= 0) || (NumSteps >= 999) )
    NumSteps = 100;

  if (VSWR_Min < 1)
    VSWR_Min = 1234;

  if ( (Fres_MHz <= 0.000001) || (Fres_MHz >= 99) )
    Fres_MHz = 12.3456789;

  if ( (CalM < 0.00001) || (CalM > 200) )
    CalM = 0.5;

  if ( (CalB < 0.00001) || (CalB > 200) )
    CalB = 12;

  if ( (SpeakerOn != 0) || (SpeakerOn != 1) )
    SpeakerOn = 0;
  
  // save changed setting...
  updateEEPROM();
//Serial.print("get: "); Serial.print(CalM); Serial.print (" "); Serial.print(CalB); Serial.print(" "); Serial.print(SpeakerOn); Serial.println();
}

void calcStep() {
  Fstep_MHz = (Fstop_MHz - Fstart_MHz) / (NumSteps -1);
}

double char2double() {

  String inString(charBuf);
  double Fnew_MHz;

  Fnew_MHz = inString.toDouble();

  return Fnew_MHz;

}

unsigned int char2Int() {

  String inString(charBuf);
  unsigned int Steps;

  Steps = atoi(inString.c_str());

  return Steps;

}

void clearOLED() {

  u8g.firstPage();
  do {
  } while ( u8g.nextPage() );

  u8g.setDefaultForegroundColor();
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

}

void drawText(void) {

  drawHeader();
  
  u8g_uint_t d;

  u8g.setDefaultForegroundColor();

  u8g.setFontRefHeightText();
  u8g.setFontPosTop();
  
  setFont(3);
  
  d = u8g.getStrWidth("0");

  u8g.setPrintPos(0 , 13);
  if (SweepMode == 0) {
    u8g.print ("H");
  } else if (SweepMode == 1) {
    u8g.print ("S");
  } else if (SweepMode == 2) {
    u8g.print ("L");
  }
  
  if (In_State == 6) {
    u8g.setPrintPos(d * 13, 13); u8g.print (Step);
    u8g.setPrintPos(d * 16, 13); u8g.print ("/");
    u8g.setPrintPos(d * 18, 13); u8g.print (NumSteps);
  }
  
  u8g.setPrintPos(0, 26); u8g.print (current_freq_MHz, 6);
  u8g.setPrintPos(d * 13, 26); u8g.print (Fstep_MHz, 6);

  u8g.setPrintPos(0, 39 ); u8g.print (VSWR_Max,3);
  u8g.setPrintPos(d * 12 , 39 ); u8g.print (VSWR, 3);

  u8g.setPrintPos(0, 52 ); u8g.print (Fres_MHz, 6);
  u8g.setPrintPos(d * 12 , 52); u8g.print (VSWR_Min, 3);

}

void drawHeader(void) {

  uint8_t i, h;
  u8g_uint_t w, d;

  setFont(0);

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();
  d = u8g.getStrWidth("0");

  u8g.setDefaultForegroundColor();

  u8g.setPrintPos(2, 0); u8g.print (Fstart_MHz, 3);
  u8g.setPrintPos(d * 6 , 0); u8g.print ("-");
  u8g.setPrintPos(d * 8, 0); u8g.print (Fstop_MHz, 3);
  u8g.setPrintPos(d * 16, 0); u8g.print (Fres_MHz, 3);
  u8g.setPrintPos(d * 23 - 2, 0); u8g.print (VSWR_Min, 1);
  
  u8g.setPrintPos(123 , 0);
  
  u8g.setColorIndex(1);
  u8g.drawLine (0, h + 1 , 128, h + 1);

}

void drawMenu(void) {

  uint8_t i, h, y;    // y offset to take care of the header
  u8g_uint_t w, d;

  y = 10;

  drawHeader();

  setFont(3);

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();

  u8g.setDefaultForegroundColor();

  for ( i = 0; i < MENU_ITEMS; i++ ) {

    d = (w - u8g.getStrWidth(menu_strings[i])) / 2;
    u8g.setDefaultForegroundColor();

    if ( i == menu_current ) {
      u8g.drawBox(0, i * h + y, w, h);
      u8g.setDefaultBackgroundColor();
    }

    u8g.drawStr(d, i * h + y, menu_strings[i]);
  }

}

void drawIntInput() {

  uint8_t i, h;
  unsigned int Num ;
  u8g_uint_t w, d;

  uint8_t x, y;   // offset for the position of the text

  x = 20;
  y = 30;

  String inString = "";

  drawHeader();

  setFont(4);

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();

  d = u8g.getStrWidth("0");

  for ( i = 0 ; i < Input_Length; i++) {

    inString = String (charBuf[i]);
    
    if ( ((i == 0) and (inString == " ")) ) {   // skip the first|null values. This is neededs since charBuf may
      //continue;                               // be much longer than f.e. the length of the sweep number.
      Num = 0;
    } else {
      Num = inString.toInt();
    }
    
    u8g.setDefaultForegroundColor();

    if ( inString == "." ) {
      u8g.drawStr( i * d + x, y, ".");
      continue;
    } else {

      if ( i == num_current ) {                   // highlight the current number (draw a box)
        u8g.drawBox( i * d + x, y , d, h);
        u8g.setDefaultBackgroundColor();
      }

      u8g.drawStr( i * d + x, y, int_val[Num]);   // print the current bumber
    }
  }

}

void drawGraph() {

  uint8_t i, h;
  int Num ;
  u8g_uint_t w, d;

  uint8_t x, y0, y1;   // offset for the position of the text

  double y_fact;

  x = 0;
  y0 = 0;
  y1 = 0;

  h = u8g.getHeight();
  w = u8g.getWidth();                             // get width of the display

  y_fact = (h - 5)  / (VSWR_Max - VSWR_Min) ;

  for ( i = 0 ; i < 127 ; i++) {

    y0 = h - 3 - (((double (VSWR_Sweep[i])   / 1000) - VSWR_Min)  * y_fact);
    y1 = h - 3 - (((double (VSWR_Sweep[i+1]) / 1000) - VSWR_Min)  * y_fact);
    
    u8g.drawLine (i, y0, i + 1 , y1 ); 

    if (Debug > 1) {
      Serial.print("Debug: drawGraph done: i, y0, y1, VSWR_Sweep[i], VSWR_Sweep[i+1]: ");
      Serial.print(i); Serial.print(", ");Serial.print(y0); Serial.print(", "); Serial.print(y1);Serial.print(", "); Serial.print(VSWR_Sweep[i]);Serial.print(", "); Serial.println(VSWR_Sweep[i+1]);
    }
  }

  // print scale...
  setFont(0);

  d = u8g.getStrWidth("0");

  u8g.setDefaultBackgroundColor();
  
  u8g.drawBox( 0,  0 , 20 , 7);
  u8g.drawBox( w / 2 - 2 * d ,  0, 20 , 7);
  u8g.drawBox( w - 1 - 4 * d , 0, 4 * d, 7);
  
  u8g.drawBox( 0, 10 , 20, 7);
  u8g.drawBox( 0, 55 , 20, 7);
  
  u8g.setDefaultForegroundColor();
  
  u8g.setPrintPos(0, 0);              u8g.print (Fstart_MHz, 2);
  u8g.setPrintPos(w / 2 - 2 * d , 0);  u8g.print (Fstart_MHz + (Fstop_MHz - Fstart_MHz) / 2, 2);
  u8g.setPrintPos(w - 5 * d, 0);      u8g.print (Fstop_MHz, 2);

  u8g.setPrintPos(0, 10);             u8g.print (VSWR_Max);
  u8g.setPrintPos(0, 55);             u8g.print (VSWR_Min);
  
  u8g.drawLine(w/2, h - 1 , w/2, h - 4);
  u8g.drawLine (0 , h - 1 , w, h - 1);

}

void SWRSound() {

  int Pulse = SoundPauseInteval -  (SoundPauseInteval / VSWR);
  
  if ( Pulse < 47 ) {                              // this means tune down at swr < 1.05... continously sound...
//    if (toneUp == 0) {                             // and if speaker was off...
      toneUp = 1;
      tone(SPEAKER, SpeakerFreq);
//    }
  } else {                                        // Not that perfekt SWR ... pulsed sound with increased freq if swr increases
    if ( (millis() - TimeLast) > (Pulse/4) ) {
      if (toneUp == 1) {                          // if already on, mute...
        toneUp = 0;
        noTone(SPEAKER);
      }  else {
        toneUp = 1;
        tone(SPEAKER, SpeakerFreq / VSWR);        // better swr -> higher swr (getting close to 5100Hz)
      }
    TimeLast = millis();
    }
  }
}

void SwitchSpeaker() {

  if (SpeakerOn == 0 ) {    // only in text hold enable/diasble speaker
    SpeakerOn = 1;
  } else {
    SpeakerOn = 0;
    noTone(SPEAKER);
  }
   
}

void SwitchRunHold() {

  SweepMode++;
  
  if (SweepMode == 3) {
    SweepMode = 0;
  }

  switch (SweepMode) {

    case 0:
            break;
    case 1:
    case 2:
            VSWR_Max = 0;
            VSWR_Min = 1234;
            Switch2TextRun();
            break;
            
  }

  redraw_required = 't';

}

void SwitchMenuNext() {

  menu_current++;

  if ( menu_current >= MENU_ITEMS )
    menu_current = 0;

  redraw_required = 'm';

}

void SwitchMenuPrev() {

  if ( menu_current == 0 )
    menu_current = MENU_ITEMS;

  menu_current--;

  redraw_required = 'm';
}

void SwitchIncChar() {

  int Num;
  String inString;

  inString = String(charBuf[num_current]);
  Num = inString.toInt();

  Num++;

  if ( Num >= 10 ) {
    Num = 0;
  }

  charBuf[num_current] = (char) Num + 48;

  redraw_required = 'i';

}

void SwitchDecChar() {

  int Num;
  String inString;

  inString = String ( charBuf[num_current] );
  Num = inString.toInt();

  Num--;

  if ( Num < 0 ) {
    Num = 9;
  }

  charBuf[num_current] = (char) Num + 48;

  redraw_required = 'i';

}

void Switch2SetFstart() {

  In_State = 2;
  Input_Length = 9;

  dtostrf(Fstart_MHz, 9, 6, charBuf);

  num_current = 0;

  redraw_required = 'i';

}

void Switch2SetFstop() {

  In_State = 3;
  Input_Length = 9;

  dtostrf(Fstop_MHz, 9, 6, charBuf);

  num_current = 0;

  redraw_required = 'i';

}

void Switch2SetSweepSteps() {

  In_State = 4;
  Input_Length = 3;

  dtostrf((double) NumSteps, 3, 0, charBuf);

  num_current = 0;

  redraw_required = 'i';

  calcStep();
  
}

void Switch2Menu() {

  In_State = 1;

  SweepMode = 0;

  // menu_current = 0;
  num_current = 0;

  noTone(SPEAKER);
  
  redraw_required = 'm';
}

void Switch2TextHold() {

  In_State = 0;

  SweepMode = 0;

  if (Step >= NumSteps) {
     Step= 0;
  }

  menu_current = 0;
  num_current = 0;

  current_freq_MHz = Fstart_MHz;
  SetDDSFreq(Fstart_MHz);

  redraw_required = 't';
  
}

void Switch2GraphMode() {

  In_State = 5;

  SweepMode = 1;

  Fstep_MHz = (Fstop_MHz - Fstart_MHz) / ( NumSteps -1) ;
  
  VSWR_Max = 0;
  VSWR_Min = 1234;
  
  menu_current = 0;
  num_current = 0;
  
  redraw_required = 'g';

}

void Switch2TextRun() {

  In_State = 6;

  Fstep_MHz = (Fstop_MHz - Fstart_MHz) / ( NumSteps -1) ;

  menu_current = 0;
  num_current = 0;

  redraw_required = 't';

}

void ZoomMeas() {

  // getting closer to the res frequence...
  
  /*
   
  double Fgap_MHz = ( round (( Fstop_MHz - Fstart_MHz ) * 100) / 100 ) / 10;
 
  Fstart_MHz = round (Fres_MHz * 100 ) / 100 - Fgap_MHz;
  Fstop_MHz = round (Fres_MHz * 100 ) / 100 + Fgap_MHz;

  */

  double Fgap_MHz = ( Fstop_MHz - Fstart_MHz )  / 10;
 
  Fstart_MHz  = Fres_MHz - Fgap_MHz;
  Fstop_MHz   = Fres_MHz + Fgap_MHz;

  if (Fstart_MHz <= 0)
    Fstart_MHz = 0.0001;                                    // at least 100Hz should be fine

  if ((Fstop_MHz <= 0) or (Fstop_MHz <= Fstart_MHz) )
    Fstop_MHz = Fstart_MHz + Fgap_MHz;
    
  Fstep_MHz = (Fstop_MHz - Fstart_MHz) / ( NumSteps -1) ;
  
}

void updateMenu(void) {

  if ( uiKeyCode != KEY_NONE && last_key_code == uiKeyCode ) {
    return;
  }

  last_key_code = uiKeyCode;

  switch ( uiKeyCode ) {

    // In_State 0 display values
    // In_State 1 change settings
    //       ...2 start freq
    //       ...3 stop freq
    //       ...4 sweep steps
    //       ...5 Graph run
    //       ...6 Text run

    case KEY_NEXT:

      if ( In_State == 0 ) {
        Switch2SetFstart();
        break;
      }

      if ( In_State == 1 ) {
        SwitchMenuNext();
        break;
      }

      if ( (In_State == 2) || (In_State == 3) || (In_State == 4)) {
        SwitchDecChar();
        break;
      }

      if ( In_State == 5 ) {
        Switch2TextRun();
        break;
      }

      if ( In_State == 6 ) {
        ZoomMeas();
        Switch2TextRun();
        break;
      }

      break;

    case KEY_PREV:

      if ( In_State == 0 ) {
        // SwitchRunHold();
        SwitchSpeaker();
        break;
      }

      if ( In_State == 1 ) {
        SwitchMenuPrev();
        break;
      }

      if ( (In_State == 2) || (In_State == 3) || (In_State == 4)) {
        SwitchIncChar();
        break;
      }

      if ( In_State == 5 ) {
        Switch2GraphMode();
        break;
      }

      if ( In_State == 6 ) {
        SwitchRunHold();
        break;
      }

      break;

    case KEY_SELECT:

      if ( In_State == 0 ) {
        Switch2Menu();
        break;
      }

      if ( In_State == 1 ) {

        if (menu_current == 0) {
          Switch2TextHold();
          break;
        }

        if (menu_current == 1 ) {
          Switch2SetFstart();
          break;
        }

        if (menu_current == 2 ) {
          Switch2SetFstop();
          break;
        }

        if (menu_current == 3 ) {
          Switch2SetSweepSteps();
          break;
        }

        if (menu_current == 4 ) {
          Switch2GraphMode();
          break;
        }

        if (menu_current == 5 ) {
          SweepMode = 1;
          Switch2TextRun();
          break;
        }

      }

      if ( (In_State == 2) || (In_State == 3) || (In_State == 4) ) {

        num_current++;

        if (num_current == 2 ) {

          if ( (In_State == 2) || ( In_State == 3 ) )
            num_current = 3;   // skip the "."

          redraw_required = 'i';

          break;
        }

        if (num_current == Input_Length) {      // we just changed the last number of the frequence... store and exit to main menue

          num_current = 0;

          if ( In_State == 2 )
            Fstart_MHz = char2double();

          if ( In_State == 3 )
            Fstop_MHz = char2double();

          if ( In_State == 4 )
            NumSteps = char2Int();

          // chech new frequences and calc steps
          if (Fstart_MHz >= Fstop_MHz ) {
            Fstop_MHz = Fstart_MHz + 1;
          }
          
          calcStep();
  
          redraw_required = 'm';
          In_State = 1;

          break;

        } else {

          redraw_required = 'i';
          break;
        }
      }

      if ( In_State == 5 ) {
        Switch2Menu();
        break;
      }

      if ( In_State == 6 ) {
        Switch2Menu();
        break;
      }

      break;
  }

}

void uiStep(void) {

  uiKeyCodeSecond = uiKeyCodeFirst;
  if ( digitalRead(uiKeyPrev) == LOW )
    uiKeyCodeFirst = KEY_PREV;
  else if ( digitalRead(uiKeyNext) == LOW )
    uiKeyCodeFirst = KEY_NEXT;
  else if ( digitalRead(uiKeySelect) == LOW )
    uiKeyCodeFirst = KEY_SELECT;
  else
    uiKeyCodeFirst = KEY_NONE;

  if ( uiKeyCodeSecond == uiKeyCodeFirst )
    uiKeyCode = uiKeyCodeFirst;
  else
    uiKeyCode = KEY_NONE;

}

void Comm_Serial() {
  
      incoming_char = Serial.read();
      switch(incoming_char){
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        serial_input_number=serial_input_number*10+(incoming_char-'0');
        break;
      case 'A':
        //Turn frequency into FStart
        Fstart_MHz = ((double)serial_input_number/1000000);
        calcStep();
        serial_input_number = 0;
        PrintSerialHelp();
        redraw_required = 't';
        break;
      case 'B':
        //Turn frequency into FStop
        Fstop_MHz = ((double)serial_input_number/1000000);
        calcStep();
        serial_input_number = 0;
        PrintSerialHelp();
        redraw_required = 't';
        break;
/*      case 'C':
        //Turn frequency into FStart and set DDS output to single frequency
        Fstart_MHz = ((double)serial_input_number);
        SetDDSFreq(Fstart_MHz);
        SweepMode = 0;
        break;
*/
      case 'N':
        // Set number of Step in the sweep
        NumSteps = serial_input_number;
        calcStep();
        serial_input_number = 0;
        PrintSerialHelp();
        redraw_required = 't';
        break;
      case 'S':
        SweepMode = 1;
        Switch2TextRun();
        
        break;
      case '?':
        PrintSerialHelp();
        break;
      }
      Serial.flush();
}

void PrintSerialHelp() {
        // Report current configuration to PC
        Serial.println();
        Serial.print("A - Start (Hz): "); Serial.println(Fstart_MHz * 1000000, 0);
        Serial.print("B - Stop  (Hz): "); Serial.println(Fstop_MHz * 1000000, 0);
        Serial.print("N - Number of Steps: "); Serial.println(NumSteps);
        Serial.println("S - Start");
        Serial.println();
        Serial.println("Start to 7.11MHz: 7110000 <Enter> A <Enter>");
        Serial.println();
}

double MeasureVoltage(char PinA, uint8_t numRuns) {

  double MesReturn = 0;
  double MesV[MeasureRuns];                      // in general we run MeasureRuns "internal" measures...
  

  for (int i = 0 ; i < numRuns; i++) {          // depends on the requirement we run some more measures over all....

    double MesT0 = 0;
    double MesT1 = 0;
    
    for (int ii = 0; ii < MeasureRuns; ii++) {  // take the 5 measures, store values in MesV
//      delay(10);
      MesV[ii] = analogRead(PinA);
      MesT0 += MesV[ii];
    }
    
    MesT0 = MesT0 / MeasureRuns;                  // ...average...
    
    // Check if there was a "peak" and do not use this measure if it differs more than 30%.
    // If so ignore this value. If all differs take the average
    
    int n = 0;

    for (int ii = 0; ii < MeasureRuns; ii++) {
      
      if ( (sqrt ( pow ( MesT0 - MesV[ii], 2))) < (MesT0 * 0.2) ) {         // only take the measures with not more than 20% diff...
        MesT1 += MesV[ii];
        n++;
      }
    }

    if ( n == 0 ) {                            // if all measures differs a lot and... take average... :-(
      MesReturn += MesT0;
    } else {
      MesReturn += MesT1 / (n + 1);
    }

  }

  MesReturn = MesReturn / numRuns;
  
  return MesReturn;
}

void ReadVSWR() {

  double REV = 0;
  double FWD = 0;
  
  REV = MeasureVoltage(PIN_A0, 2);
  FWD = MeasureVoltage(PIN_A1, 2);
  
  if (REV >= FWD) {
    // To avoid a divide by zero or negative VSWR then set to max 999
    // this should not happen :-/
    VSWR = 999;
  } else {
    
    if (In_State != 7 )                 // In calibration mode do not fix the voltage measure
       REV = fixREV(REV);
  
    // Calculate VSWR
    VSWR = (FWD + REV) / (FWD - REV);

    // Remeber best and worse VSWR
    if (VSWR < VSWR_Min) {
      Fres_MHz = current_freq_MHz;
      VSWR_Min = VSWR;
    }

    if (VSWR > VSWR_Max)
       VSWR_Max = VSWR;

     updateEEPROM();
      
  }
  
  if (Debug > 2) {
    Serial.print("Debug: ReadVSWR done. FWD, REV, VSWR, MHz: "); Serial.print(FWD, 6); Serial.print(", "); Serial.print(REV, 6); Serial.print(", "); Serial.print(VSWR,6); Serial.print(", "); Serial.print(current_freq_MHz,6); Serial.println("");
  }
}

void Perform_sweep(uint8_t Step ) {
  
  // Calculate current frequency
  current_freq_MHz = Fstart_MHz + ( Fstep_MHz * Step) ;

  // Set DDS to current frequency

  SetDDSFreq(current_freq_MHz);

  // Wait a little for settling

  if (Step == 0 ) {
    delay(500);
  } else {
    delay(50);
  }

  // Read the forawrd and reverse voltages
  ReadVSWR();

  if (Debug > 2) {
    Serial.print("Debug: Perform_sweep done: VSWR_Min: "); Serial.print(VSWR_Min); Serial.print(" VSWR_Max: "); Serial.println(VSWR_Max);
  }
}

void SetDDSFreq(double Freq_Hz) {

  // don't know... but sometimes this must be done more than one time... :-(
  for (int i = 0; i < 2; i++) {
    
    // Calculate the DDS word - from AD9850 Datasheet
    int32_t f = Freq_Hz * 1000000 * 4294967295 / 125000000;
    // Send one byte at a time
    for (int b = 0; b < 4; b++, f >>= 8) {
      send_byte(f & 0xFF);
    }
  
    // 5th byte needs to be zeros
    send_byte(0);
  
    // Strobe the Update pin to tell DDS to use values
    digitalWrite(FQ_UD, HIGH);
    digitalWrite(FQ_UD, LOW);

  }
  
  if (Debug > 1) {
    Serial.print("Debug: SetDDSFreq done: "); Serial.println(Freq_Hz, 6);
  }
}

void send_byte(byte data_to_send) {
  // Bit bang the byte over the SPI bus
  for (int i = 0; i < 8; i++, data_to_send >>= 1) {
    // Set Data bit on output pin
    digitalWrite(SDAT, data_to_send & 0x01);
    // Strobe the clock pin
    digitalWrite(SCLK, HIGH);
    digitalWrite(SCLK, LOW);
  }
}

void setup() {

  if (Debug > 0 ) {
    Serial.begin(115200);
    delay(100);
    Serial.println("Enter ? for help!");
  }

  //Initialise the incoming serial number to zero
  serial_input_number = 0;

  // get last values or defaults...
  getEEPROM();

  // Configiure DDS control pins for digital output
  pinMode(FQ_UD, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(SDAT, OUTPUT);
  pinMode(RESET, OUTPUT);

  pinMode(uiKeySelect, INPUT_PULLUP);         // set pin to input with pullup
  pinMode(uiKeyPrev, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyNext, INPUT_PULLUP);           // set pin to input with pullup

  // what kind of mode/part should be initial displayed? Text mode...
  redraw_required = 't';

  // Set up analog inputs on A0 and A1, internal reference voltage
  pinMode(PIN_A0, INPUT);
  pinMode(PIN_A1, INPUT);
  analogReference(INTERNAL);

  // Reset the DDS
  digitalWrite(RESET, HIGH);
  digitalWrite(RESET, LOW);

//  clearOLED();

  setFont(3);
  
  // run calibration mode if KEY_SELECT is pressed...
  if ( digitalRead(uiKeySelect) == LOW ) {

    In_State = 7;
  
    u8g.firstPage();
    do {
      u8g.setPrintPos(8, 10); u8g.print ("Please wait!");
      u8g.setPrintPos(8, 30); u8g.print ("Calibration...");
    } while ( u8g.nextPage() );
    
    calibrate(); 
    
    do {
      u8g.setPrintPos(8, 40); u8g.print ("...done!");
    } while ( u8g.nextPage() );
    delay (2000);
    
    clearOLED();
    In_State = 0;
  }

  // greetings...
  //setFont(2);
  u8g.firstPage();
    do {
      u8g.setPrintPos(4, 4); u8g.print ("Analyzer V"); u8g.setPrintPos(65, 4); u8g.print (VERSION);
      u8g.setPrintPos(4, 22); u8g.print ("DC8LZ - feindas.de");
      u8g.setPrintPos(4, 40); u8g.print ("based on K6BEZ");
    } while ( u8g.nextPage() );
  delay (3000);

  current_freq_MHz = Fstart_MHz;
  
  Fstep_MHz = (Fstop_MHz - Fstart_MHz) / ( NumSteps -1);

  SetDDSFreq(Fstart_MHz);
  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;                
  OCR1A = 4000;             // change this to set the interrupt intervall
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

  menu_current = 0;
  
  drawText();

  if (Debug > 2) {
    Serial.println("Debug: setup done");
  }

  // ready beep...
  tone(SPEAKER, SpeakerFreq);
  delay(600); noTone(SPEAKER); delay(100);
  tone(SPEAKER, SpeakerFreq); delay(50);
  delay(80); noTone(SPEAKER);delay(50);
  tone(SPEAKER, SpeakerFreq); 
  delay(80); noTone(SPEAKER);
}

// interrupt service routine
ISR(TIMER1_COMPA_vect) {
 
  TCNT1 = 0;          // preload timer

  switch ( In_State ) {
    case 0:
    case 6:
        if (SpeakerOn == 1)
          SWRSound();
        break;
    }   
}

void loop() {

  uiStep();                                     // check for key press
  updateMenu();                                 // update menu bar

  if (Debug > 1) {
    Serial.print("Debug: In_State: ");
    Serial.print(In_State);
    Serial.print(" Button: ");
    Serial.print(uiKeyCode);
    Serial.print(" SweepMode: ");
    Serial.print(SweepMode);
    Serial.print(" NumSteps: ");
    Serial.print(NumSteps);
    Serial.print(" menu_current: ");
    Serial.print(menu_current);
    Serial.print(" num_current: ");
    Serial.print(num_current);
    Serial.println();
  }

  // >>>>>>> Freq && Measure <<<<<<<<<<<

  if (SweepMode == 0) {                     // stay on current frequence...

    ReadVSWR();                             // reading VSWR

    switch ( In_State ) {
      case 0:
        redraw_required = 't';              // redraw in text mode makes sense to check SWR on fix frequence...
        break;
      case 5:
        redraw_required = 'x';              // do not redraw graph...
        break;
      case 6:
        redraw_required = 't';              // redraw in text mode makes sense to check SWR on fix frequence...

        break;
    }
  }
     
  if (SweepMode > 0) {                        // run... (text or graphic)

    if (Debug > 0) {                          // send measure data to serial..
      Serial.print("#"); Serial.print(Step); Serial.print(" "); Serial.print (current_freq_MHz, 6); Serial.print("MHz "); Serial.print(VSWR); Serial.println(" VSWR");
    }

    ///////////////// graph /////////////////////
    
    if (In_State == 5) {                      

      VSWR_Max = 0;                           // reset VSWR Min Max for rigth scaling...
      VSWR_Min = 1234;
      
      clearOLED();                            // print "please wait..." notice
      setFont(3);

      u8g.firstPage();
      do {
        u8g.setPrintPos(8, 10); u8g.print ("Please wait!");
        u8g.setPrintPos(8, 30); u8g.print ("Measure in progress!");
      } while ( u8g.nextPage() );

      
      // Steps = u8g.getWidth();        // atm hardcoaded to 128
      Fstep_MHz = (Fstop_MHz - Fstart_MHz) / (128 -1);
            
      for (int i = 0 ; i <= 128; i++ ) {      // performe measures...
        
        Perform_sweep(i);

        if (VSWR != 999) {
          VSWR_Sweep[i] = VSWR * 1000;
        }
        
        if (Debug > 1) {
          Serial.print("Debug: Measure Nr., VSWR, VSWR_Sweep[i] "); Serial.print(i); Serial.print(" :"); Serial.print(VSWR,6); Serial.print(", "); Serial.println(VSWR_Sweep[i]); 
        }
        
      }

      calcStep();
      
      if (SweepMode == 1 ) {          // In SweepMode 1 we only run once... so stop and switch to SweepMode 0.
        SweepMode = 0;
      } 
      
      redraw_required = 'g';

    ///////////////////// Text /////////////////
    
    } else if (In_State == 6) {           
      redraw_required = 't';

      if ( Step == 0 ) {                 // reset res frequence and best VSWR before we start the measure.
        Fres_MHz = 0;
        VSWR_Min = 9999;
      }
      
      Perform_sweep(Step++);

      if (Step >= NumSteps) {            // we have finished one sweep run..!
        
        Step = 0;
        
        if (SweepMode == 1 ) {          // In SweepMode 1 we only run once... so stop and switch to SweepMode 0.
           SweepMode = 0;
        } else {
          Perform_sweep(Step++);
        }
      }
    }
  }

  if (redraw_required != 'x') {                // i f there is something to sent to the display...
    
    u8g.firstPage();
    do {
      
      switch (redraw_required) {
        case 't':
          drawText();
          break;
        case 'm':
          drawMenu();
          break;
        case 'i':
          drawIntInput ();
          break;
        case 'g':
          drawGraph();
          break;
        default:
          break;
      }
      
    }  while ( u8g.nextPage() );
    
    redraw_required = 'x';
  }

  // check serial... 
  if (Debug > 0) {
    Comm_Serial();                           // Communication via searial/usb to PC...
  }

  if (Debug > 1) {
    Serial.println("Debug: Loop done!");
  }
}

