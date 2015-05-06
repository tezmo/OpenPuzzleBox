#include "AnythingEEPROM.h"
#include <EEPROM.h>
#include <PWMServo.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Juerd Waalboer - https://github.com/revspace/airhockey
#include <PowerPin.h>
#include <Button.h>

const byte MAX_WAYPOINT  = 10;
const byte MAX_ROUTE     = 10;
const byte TRIES[]       = { 6, 12, 18 };
const byte TOLERANCE[]   = { 30, 20, 10 };

// PCD8544( SCLK, DIN, D/C, CS, RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(12, 11, 9, 7, 8);

SoftwareSerial  nss(/*RX*/2, /*TX*/14);
TinyGPS         gps;
PWMServo        servo;

PowerPin        servo_power(6);
PowerPin        backlight(13);
Button          button(4); 
long            previousMillis = 0;            // previous checked time
int             ani;


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
  display.clearDisplay();
  display.setContrast(30);
  display.clearDisplay();
  intro();
}

void loop() {
  static enum:char {
                                               CRASHED,
     SELECT_ROUTE_SETUP,  SELECT_ROUTE_UPDATE, SELECT_ROUTE,
     SELECT_TRIES_SETUP,  SELECT_TRIES_UPDATE, SELECT_TRIES,
 SELECT_TOLERANCE_SETUP,                       SELECT_TOLERANCE_UPDATE, 
       SELECT_TOLERANCE,
            CLOSE_SETUP,                       CLOSE,
     WAIT_FOR_FIX_SETUP,  WAIT_FOR_FIX_UPDATE, WAIT_FOR_FIX,
            ROUTE_SETUP,                                      ROUTE_DONE,
         WAYPOINT_SETUP,      WAYPOINT_UPDATE, WAYPOINT,      WAYPOINT_DONE,
             FAIL_SETUP,                       FAIL,
    PROGRAM_YESNO_SETUP,                       PROGRAM_YESNO,
          PROGRAM_SETUP,       PROGRAM_UPDATE, PROGRAM,       PROGRAM_DONE
  } state = SELECT_ROUTE_SETUP;

  static byte          route, waypoint, tries_left, triesidx, tolidx;
  static byte          dotstate = 0;
  static boolean       programming = false, yesno;
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
    case CRASHED: {
       switch(button.pressed(5000)) {
        case SHORT:
          backlight.off(1500);
          break;
        case LONG:
          state = SELECT_ROUTE_SETUP;
          break;
       }
      break;       
     }
    
    case SELECT_ROUTE_SETUP: {
      open_lock();;
      route = 1;
      display.clearDisplay();
      display.print("Choose route.\nRoute #\n\n\nnext        OKshort     long");
      display.display();
      state = SELECT_ROUTE_UPDATE;
      break;  
    }
    
    case SELECT_ROUTE_UPDATE: {
      display.fillRect(42,8,6,8,WHITE);
      display.setCursor(42,8);
      display.print(route, DEC);
      display.print(" ");
      display.display();      
      state = SELECT_ROUTE;
      break;
    }
    
    case SELECT_ROUTE: {
      switch (button.pressed()) {
        case SHORT:
          route++;
          if (route > MAX_ROUTE) route = 1;
          state = SELECT_ROUTE_UPDATE;
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
      display.print("Choose level.\n\n\n\nnext        OKshort     long");
      display.display();
      state = SELECT_TRIES_UPDATE;
      break;  
    }
    
    case SELECT_TRIES_UPDATE: {
      display.fillRect(0,8,12,8,WHITE);    
      display.setCursor(0,8);
      display.print(TRIES[triesidx], DEC);
      display.setCursor(18,8);
      display.print("attempts  ");
      display.display();
      state = SELECT_TRIES;
      break;
    }
    
    case SELECT_TRIES: {
      switch (button.pressed()) {
        case SHORT:
          triesidx++;
          if (triesidx >= sizeof(TRIES)) triesidx = 0;
          state = SELECT_TRIES_UPDATE;
          break;
        case LONG:
          state = SELECT_TOLERANCE_SETUP;
          break;
      }
      break;
    }
 
    case SELECT_TOLERANCE_SETUP: {
      tolidx = 0;
      display.clearDisplay();
      display.print("Tolerance:\n\n\n\nnext        OKshort     long");
      display.display();
      state = SELECT_TOLERANCE_UPDATE;
      break;  
    }
   
    case SELECT_TOLERANCE_UPDATE: {
      display.fillRect(0,8,12,8,WHITE);    
      display.setCursor(0,8);
      display.print(TOLERANCE[tolidx], DEC);
      display.print(" meters  ");
      display.display();
      state = SELECT_TOLERANCE;
      break;
    }
    
    case SELECT_TOLERANCE: {
      switch (button.pressed()) {
        case SHORT:
          tolidx++;
          if (tolidx >= sizeof(TOLERANCE)) tolidx = 0;
          state = SELECT_TOLERANCE_UPDATE;
          break;
        case LONG:
          state = CLOSE_SETUP;
          break;
      }
      break;
    } 
 
    
    case CLOSE_SETUP: {
      display.clearDisplay();
      display.print("Can the box be");
      display.setTextSize(2);
      display.setCursor(0,12);
      display.print("locked?");
      display.setTextSize(1);
      display.setCursor(0,32);
      display.print("            OK          long");
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
      display.print("Wait for \nGPS signal.");
      display.drawTriangle(30,35,45,42,51,21,BLACK);
      display.display();
      state = WAIT_FOR_FIX_UPDATE;
      break;
    }
      
    case WAIT_FOR_FIX_UPDATE: {
      
      
      if (millis()-previousMillis>1000){
           ani = (ani) ? 0 : 1;
           display.fillTriangle(31,35,44,41,50,22,ani);
           previousMillis = millis();
           display.display();
      }
      state = WAIT_FOR_FIX;
      break;
    }
    
    case WAIT_FOR_FIX: {
      switch(button.pressed()) {
        case SHORT:
          backlight.off(1500);
          break;
        case LONG:
          state = PROGRAM_YESNO_SETUP;
          break;
        default:
          if (fix && gps.hdop() < 500 && millis() - fix > 2000)
            state = programming ? PROGRAM_SETUP : ROUTE_SETUP;
          else 
            state = WAIT_FOR_FIX_UPDATE;
      }
      break;
    }
      
    case ROUTE_SETUP: {
      waypoint = 1;
      state = WAYPOINT_SETUP;
      break;
    }
      
    case WAYPOINT_SETUP: {
      display.clearDisplay();
      display.print("En route to\nwaypoint ");
      display.print(waypoint, DEC);
      display.display();
      EEPROM_readAnything(address_for(route, waypoint), there);
      tries_left = TRIES[triesidx];
      there.tolerance = TOLERANCE[tolidx];
      distance   = -1;
      state = there.lat ? WAYPOINT_UPDATE : ROUTE_DONE;
      break;
    }
    
    case WAYPOINT_UPDATE: {
      backlight.off(15000);
      if (distance >= 0) {
        display.setCursor(0, 16);
        display.print("Distance:\n");
        display.fillRect(0,24,24,8,WHITE);
        display.setCursor(0, 24);
        display.print(distance, 0);
        display.setCursor(30, 24);
        display.print("m");
        display.display();
      }
      display.setCursor(0, 32);
      display.print("You have ");
      display.fillRect(54,32,12,8,WHITE);  
      display.print(tries_left, DEC);
      display.print(" \nattempts left.");
      display.display();
      state = WAYPOINT;
      break;
    }
      
    case WAYPOINT: {
      if (distance < 0 || !button.pressed()) break;
      state = distance <= there.tolerance ? WAYPOINT_DONE
            : --tries_left                ? WAYPOINT_UPDATE
            :                               FAIL_SETUP;
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
      display.setTextSize(2);
      display.print("Yaaaay!\n");
      display.setTextSize(1);
      display.print("  The box is \n   now open.");
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
      display.setCursor(0, 16);
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
      display.fillRect(0,32,30,8,WHITE);
      display.setCursor(0, 32);
      display.print(distance, 0);
      display.setCursor(36, 32);
      display.print(" m     ");
      display.display();
      break;
    }
    
// THIS FAR   
    
    case PROGRAM_YESNO_SETUP: {
      open_lock();  // Pre-game backdoor
      yesno = false;
      display.clearDisplay();
      backlight.off();
      display.print("Route ");
      display.print(route, DEC);
      display.print("\nprogram? ");
      display.setCursor(0,32);
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
      display.print("Go to WP #");
      display.print(waypoint, DEC);
      display.print(".\n\nSignal: \n");
      if (waypoint > 1) {
        display.print("\nWP ");
        display.print(waypoint - 1, DEC);
        display.print(" -> here:\n");
        display.display();
      }
      display.display();
      state = PROGRAM;
      break;
    }
      
    case PROGRAM: {
      switch (button.pressed()) {
        case NOT:
          display.fillRect(0,24,30,8,WHITE);
          display.setCursor(48, 16);
          print_signal(); 
          if (waypoint < 2) {
            display.display();
            break;
          }
          display.fillRect(0,40,42,8,WHITE);
          display.setCursor(0, 40);
          display.print(distance, 0);
          display.setCursor(42, 40);
          display.print("m");
          display.display();
          break;
        case LONG:
          state = PROGRAM_DONE;
          break;
        case SHORT:
          there.lat = here_lat;
          there.lon = here_lon;  
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
  display.setTextSize(3);
  
  for (int i=0;i<3;i++){
    display.clearDisplay();
    set_text(4,1,"REV",BLACK);
    set_text(24,25,"GEO",BLACK);
    display.display();
    delay(250);
    display.fillRect(0,0,display.width(),display.height(),BLACK);
    set_text(4,1,"REV",WHITE);
    set_text(24,25,"GEO",WHITE);
    display.display();
    delay(250);
  }
  display.setTextSize(1);
  display.setTextColor(BLACK);
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

int print_signal() {
  int accuracy = gps.hdop();
  display.print(
      accuracy ==   0 ? "none"
    : accuracy <  300 ? "great"
    : accuracy <  500 ? "good"
    : accuracy < 1000 ? "bad"
    : accuracy < 2000 ? "worst"
    :                   "none"
  );
}


