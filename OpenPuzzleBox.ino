

#include "AnythingEEPROM.h"
#include "bitmaps.h"
#include <EEPROM.h>
#include <PWMServo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <SoftwareSerial.h>

// to be replaced
#include <TinyGPS.h>
#include <TinyGPS++.h>

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
  enum: char {
                 CRASHED,
     SELECT_ROUTE_SETUP,  SELECT_ROUTE,
     SELECT_TRIES_SETUP,  SELECT_TRIES,
            CLOSE_SETUP,  CLOSE,
     WAIT_FOR_FIX_SETUP,  WAIT_FOR_FIX,
         WAYPOINT_SETUP,  WAYPOINT,
        WAYPOINT_UPDATE,  WAYPOINT_DONE,
             FAIL_SETUP,  FAIL,
          PROGRAM_SETUP,
    PROGRAM_YESNO_SETUP,  PROGRAM_YESNO,
        PROGRAM_UPDATE,   PROGRAM,
            ROUTE_DONE,   PROGRAM_DONE
  } state = SELECT_ROUTE_SETUP;

  static byte          route, waypoint, tries_left, triesidx;
  static byte          dotstate = 0;
  static boolean       programming = false, yesno, full;
  static unsigned long fix = 0;
  static float         distance;
  static Waypoint      there;
  static float         here_lat, here_lon;
  static int           last;

  backlight.check();
  servo_power.check();

/*
  while (nss.available() > 0)
  gps.encode(nss.read());
*/
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
      state = WAIT_FOR_FIX_SETUP;
      break;
    }
    
    
    case WAIT_FOR_FIX_SETUP: {
      fix = 0;
      backlight.off(5000);
      display.clearDisplay();
      
      set_text(0,0,"Wait for",BLACK);
      set_text(0,8,"GPS-fix.",BLACK);
     
      if (last - millis() > 500){
        if (full){
          display.fillCircle(43, 23, 4, WHITE);
          display.drawCircle(43, 23, 4, BLACK);
          full =false;
        }
        else{
          display.fillCircle(43, 23, 4, BLACK);
          full = true;
        }
      last = millis();
      }
      display.display();
      state = WAIT_FOR_FIX;
      break;
    }
    
// UP TO HERE    
    
    
     case WAIT_FOR_FIX: {
      switch(button.pressed()) {
        case SHORT:
          backlight.off(1500);
          break;
        case LONG:
          state = PROGRAM_YESNO_SETUP;
          break;
        default:
          // to be placed in TinyGPS++ instead
          if (fix && gps.hor_acc() < 500 && millis() - fix > 2000)
            state = programming ? PROGRAM_SETUP : WAYPOINT_SETUP;
          else {
            state = WAIT_FOR_FIX_SETUP;
          }
      }
      break;
    }   
    
    
    case WAYPOINT_SETUP: {
      waypoint = 1;
      EEPROM_readAnything(address_for(route, waypoint), there);
      tries_left = TRIES[triesidx];
      distance   = -1;
      state = there.lat ? WAYPOINT_UPDATE : ROUTE_DONE;
      break;
    }
 
 
     case WAYPOINT_UPDATE: {
      backlight.off(15000);
      if (distance >= 0) {
        display.clearDisplay();
        display.print("En route to  waypoint ");
        display.print(waypoint, DEC);
      
        display.setCursor(0, 16);
        display.print("Distance:\n");
        display.print(distance, 0);
        display.print("m    ");
        display.display();
      }
      display.clearDisplay();
      display.print("En route to  waypoint ");
      display.print(waypoint, DEC);
 
      display.setCursor(0, 24);
      display.print("You have ");
      display.print(tries_left, DEC);
      display.print(" \nattempts left.");
      display.display();
      state = WAYPOINT;
      break;
    }   
    
  
    case WAYPOINT: {
      if (distance < 0 || !button.pressed()) break;
      state = distance <= there.tolerance ? CRASHED
            : --tries_left                ? WAYPOINT_UPDATE
            :                               CRASHED;
      break;
    }    
    case WAYPOINT_DONE: {
      display.clearDisplay();
      backlight.off();
      display.print("Waypoint ");
      display.print(waypoint);
      display.print("\nreached.");
      display.display();
      delay(4000);
      if (waypoint == MAX_WAYPOINT) {
        state = ROUTE_DONE;
        break;
      }      
      waypoint++;      
      state = WAYPOINT_SETUP;
      break;
    }
    
    case ROUTE_DONE: {
      display.clearDisplay();
      backlight.off(10000);
      open_lock();
      display.print("Congrats!");
      display.print("You made it! :)");
      display.display();
      state = CRASHED;
      break;
    }
      
    case FAIL_SETUP: {
      display.clearDisplay();
      display.print("GAME OVER :(");
      display.display();
      while (there.lat && waypoint <= MAX_WAYPOINT)
        EEPROM_readAnything(address_for(route, ++waypoint), there);
      EEPROM_readAnything(address_for(route, --waypoint), there);
      display.setCursor(0, 8);
      display.print("Distance to\nend goal:\n");
      display.display();
      state = FAIL;
      break;
    }
    
    case FAIL: {
      switch (button.pressed(60000)) {
        case SHORT:
          backlight.off(1500);
          break;
        case LONG:
          open_lock();  // backdoor
          break;
      }
      display.print("\r");
      display.print(distance, 0);
      display.print(" m     ");
      display.display();
      break;
    }
    
    case PROGRAM_YESNO_SETUP: {
      open_lock();  // Pre-game backdoor
      yesno = false;
      display.clearDisplay();
      backlight.off();
      display.print("Route ");
      display.print(route, DEC);
      display.print("\nprogram? ");
      display.setCursor(0,24);
      display.print("back        OKshort     long");
      display.display();
      state = PROGRAM_YESNO;
      break;
    }
     
    case PROGRAM_YESNO: {
      switch (button.pressed()) {
        case SHORT:
          state = CLOSE_SETUP;
          programming = 0;
          break;
        case LONG:
          state = WAIT_FOR_FIX_SETUP;
          programming = 1;
          break;
      }
      break;
    }

    case PROGRAM_SETUP: {
      waypoint = 1;
      state = PROGRAM_UPDATE;
      break;
    }
  
    case PROGRAM_UPDATE: {
      backlight.off(15000);
      display.clearDisplay();
      display.print("Go to\nwaypoint ");
      display.print(waypoint, DEC);
      display.print(".\nReached:\n\n");
      display.display();
      if (waypoint > 1) {
        display.print("\nWP ");
        display.print(waypoint - 1, DEC);
        display.print(" -> here:\n");
        display.display();
      }
      state = PROGRAM;
      break;
    }
      
    case PROGRAM: {
      switch (button.pressed()) {
        case NOT:
          display.setCursor(0, 24);
            display.print(gps.hor_acc()); display.print("   ");
            display.display();
          if (waypoint < 2) break;
          display.setCursor(0, 24);
          display.print(distance, 0);
          display.print("m      ");
          display.display();
          break;
        case LONG:
          state = PROGRAM_DONE;
          break;
        case SHORT:
          there.lat = here_lat;
          there.lon = here_lon;  
          there.tolerance = 30;  // FIXME
          there.flags = 0;  // FIXME
          EEPROM_writeAnything(address_for(route, waypoint), there);
          state = ++waypoint > MAX_WAYPOINT ? PROGRAM_DONE : PROGRAM_UPDATE;
          break;
      }
      break;
    }
      
    case PROGRAM_DONE: {
      if (waypoint <= MAX_WAYPOINT) {
        there.lat = 0;
        there.lon = 0;
        there.tolerance = 0;
        there.flags = 0;
        EEPROM_writeAnything(address_for(route, waypoint), there);
      }
      display.clearDisplay();
      display.print("That's all,\nfolks!");
      display.display();
      delay(5000);
      programming = false;
      state = SELECT_ROUTE_SETUP;
      break;
    }    
    
  }
  delay(10);  
}

void intro() {
  display.clearDisplay();
  display.drawBitmap(0, 0, splash, 84, 48, BLACK);
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


