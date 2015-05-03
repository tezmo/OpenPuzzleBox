#include "AnythingEEPROM.h"
#include <EEPROM.h>
#include <PWMServo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Juerd Waalboer - https://github.com/revspace/airhockey/tree/2bb4cf1c4dbf15a13e4d3c9a125fea36f564c969/libraries
#include <PowerPin.h>
#include <Button.h>

const byte MAX_WAYPOINT  = 9;
const byte MAX_ROUTE     = 9;
const byte TRIES[]       = { 15, 25, 35 };

// PCD8544( SCLK, DIN, D/C, CS, RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(12, 11, 9, 7, 8);

SoftwareSerial  nss(/*RX*/2, /*TX*/14);
TinyGPS         gps;
PWMServo        servo;
PowerPin        servo_power(6);
PowerPin        backlight(13);
Button          button(4); 


struct Waypoint {
  // Don't alter struct; stored in EEPROM
  float lat;
  float lon;
  byte tolerance;
  byte flags;
};


void setup() {
  Serial.begin(9600);
  servo.attach(10);
  nss.begin(9600);
  delay(500);

  display.begin();
  display.setContrast(30);
  display.clearDisplay();

  intro();
   
}

void loop() {
  static enum {
                          CRASHED,
     SELECT_ROUTE_SETUP,  SELECT_ROUTE,
     SELECT_TRIES_SETUP,  SELECT_TRIES,
            CLOSE_SETUP,  CLOSE,
     /*
     WAIT_FOR_FIX_SETUP,  WAIT_FOR_FIX_UPDATE, WAIT_FOR_FIX,
            ROUTE_SETUP,                                      ROUTE_DONE,
         WAYPOINT_SETUP,      WAYPOINT_UPDATE, WAYPOINT,      WAYPOINT_DONE,
             FAIL_SETUP,                       FAIL,
    PROGRAM_YESNO_SETUP,                       PROGRAM_YESNO,
          PROGRAM_SETUP,       PROGRAM_UPDATE, PROGRAM,       PROGRAM_DONE*/
  } state = SELECT_ROUTE_SETUP;

  static byte          route, waypoint, tries_left, triesidx;
  static byte          dotstate = 0;
  static boolean       programming = false, yesno;
  static unsigned long fix = 0;
  static float         distance;
  static Waypoint      there;
  static float         here_lat, here_lon;

  backlight.check();
  servo_power.check();

  // Read GPS date
  while (nss.available()) {
    char c = nss.read();
    if (gps.encode(c)) {
      unsigned long age;
      if (!fix) fix = millis();
      gps.f_get_position(&here_lat, &here_lon, &age);
      distance = TinyGPS::distance_between(here_lat, here_lon, there.lat, there.lon);
    }
  }
  
  switch (state) {
    case CRASHED: break;
    
    case SELECT_ROUTE_SETUP: {
      open_lock();;
      route = 1;

      display.clearDisplay();
      set_text(0,0,"Choose route: ",BLACK);
      set_text(0,8,"Route #: ",BLACK);
      display.print(route, DEC);
      set_text(0,32,"next        OK",BLACK);
      set_text(0,40,"short     long",BLACK);
      display.display();      

      state = SELECT_ROUTE;
      break;
    }
    
     case SELECT_ROUTE: {
      switch (button.pressed()) {
        case SHORT:
          route++;
          if (route > MAX_ROUTE) route = 1;
          state = SELECT_ROUTE;
          break;
        case LONG:
          state = SELECT_TRIES_SETUP;
          break;
      }
      break;
    }
    
    case SELECT_TRIES_SETUP: {
      triesidx = 0;
  
      display.clearDisplay();
      set_text(0,0,"Choose level. ",BLACK);
      display.setCursor(0,8);
      display.print(TRIES[triesidx], DEC);
      display.print(" attempts  ");
      set_text(0,32,"next        OK",BLACK);
      set_text(0,40,"short     long",BLACK);
      display.display();
  
      state = SELECT_TRIES;  
    }
    
    
    case SELECT_TRIES: {
      switch (button.pressed()) {
        case SHORT:
          triesidx++;
          if (triesidx >= sizeof(TRIES)) triesidx = 0;
          state = SELECT_TRIES;
          break;
        case LONG:
          state = CLOSE_SETUP;
          break;
      }
      break;
    }
    
    case CLOSE_SETUP: {
      display.clearDisplay();
      set_text(0,0,"Can the box be",BLACK);
      display.setTextSize(2);
      set_text(0,8,"LOCKED?",BLACK);
      display.setTextSize(1);      
      set_text(0,32,"            OK",BLACK);
      set_text(0,40,"          long",BLACK);
      display.display();
      state = CLOSE;
      break;
    }
    
    case CLOSE: {
      if (button.pressed() != LONG) break;
      close_lock();
      state = CRASHED;
      break;
    }
    
    
  }   
}

void intro() {
  Serial.println("intro");
  display.setCursor(0,8);
  display.setTextSize(2);
  display.print(" REV     GEO");
  display.display();
  delay(1000);
  display.setTextSize(1);
  display.clearDisplay();
  display.display();
}

void set_text(int x,int y,String text,int color){
  
  display.setTextColor(color); // Set text colour (black or white)
  display.setCursor(x,y);      // Set cursos for starting position
  display.println(text);       // Set text
  display.display();           // Update display
}

void open_lock() {
  servo_power.on(1000);
  servo.write(180);
}

void close_lock() {
  servo_power.on(1000);
  servo.write(90);
}

int address_for(byte route, byte waypoint) {
  return (route - 1) * 100 + (waypoint - 1) * 10;
}


