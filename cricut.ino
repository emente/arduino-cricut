#include <Servo.h> 

// page ca. 15cm * 30cm

/*
    
              cricut ! avr ! arduino port
 ====================+=====+================ 
 size dial    PF0    !     !
 speed dial   PF1    !     !
 press dial   PF2    !     !
 --------------------+-----+----------------
 home         PD1    !     ! 2 (low=home)
 --------------------+-----+----------------
 motor pen    PORTC  ! PAx ! 20-27 (stepper x)
 motor mat    PORTA  ! PCx ! 28-35 (stepper y)
 --------------------+-----+----------------
 pen press    PB6    !     ! 5 (low=on)
 pen u/d      PE2    !     ! 9 (255=bottom/loose, 0=top/firm)
 servo               !     ! 7 OPTIONAL (85=touching, less=press)
 */

#define HOME 2
#define BEEP 12
#define PEN 9
#define PEND 5
#define PENS 7

#define SPEED A6
#define PRESSURE A7
#define DEPTH A5

#define MAX_X		2340
#define MAX_Y		4800 //3120 = dina4
#define MAT_EDGE 250

#define H0	0x01	// half current, coil 0 
#define F0	0x02    // full current, coil 0 
#define H1	0x04    // half current, coil 1 
#define F1	0x08    // full current, coil 1  
#define H2	0x10	// half current, coil 2
#define F2	0x20    // full current, coil 2 
#define H3	0x40    // half current, coil 3
#define F3	0x80    // full current, coil 3 

static uint8_t phase[] = 
{
  F0, F0|H1, F0|F1, H0|F1,
  F1, F1|H2, F1|F2, H1|F2,
  F2, F2|H3, F2|F3, H2|F3,
  F3, F3|H0, F3|F0, H3|F0,
};

unsigned long pentimeout=0;

static int x=0, y=0, pressure=128, speed=128, depth=80;

Servo myservo;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);

  pinMode(SPEED,INPUT);
  pinMode(PRESSURE,INPUT);

  pinMode(BEEP,OUTPUT);
  digitalWrite(BEEP,LOW);

  pinMode(HOME,INPUT_PULLUP);

  DDRA = 0xff;
  DDRC = 0xff;

  myservo.attach(PENS);
  myservo.write(85);

  pentimeout = millis();

  Serial.println("!");

  updateDials();
  penUp();
  home();
  stepperOff();
}

///////////////////////////////////////////////////////////

boolean penIsDown = false;

void penDown() {
  if (penIsDown) {
//    analogWrite(PEN,pressure);
    return; 
  }

  analogWrite(PEN,255-pressure);
  digitalWrite(PEND,LOW);
  delay(80);
  myservo.write(depth);
  delay(300);
/*
  for (int i=0;i<=pressure;i++) {
    analogWrite(PEN,i);    
    delayMicroseconds(50);
  }
*/
  penIsDown = true;
}

void penUp() {
  if (!penIsDown) {
    return; 
  }

  myservo.write(90);
  delay(300);
  analogWrite(PEN,255);
  digitalWrite(PEND,HIGH);
  delay(35);

  penIsDown = false;  
}

void beep(int d) {
  digitalWrite(BEEP,HIGH);
  delay(d);
  digitalWrite(BEEP,LOW);  
}

boolean didLoad=false;

void loadPaper() {
  penUp();
  moveToNoClip(x,y+400);
  stepperOff();
  y=0;
  didLoad=true;
}

void unloadPaper() {
  penUp();
  if (didLoad) {
      moveToNoClip(x,-y-600);
  } else {
      moveToNoClip(x,MAX_Y);   
  }
  stepperOff();
  y=0;
  didLoad=false;
}

boolean isHome() {
  return digitalRead(HOME)==LOW;   
}

/////////////////////////////////////////////////////////////

void stepperOff() {
  PORTA = 0;
  PORTC = 0;   
}

void stepperTick() {
  PORTA = phase[x & 0x0f];
  PORTC = phase[y & 0x0f]; 
}

void home() {
  int step=x & 0x0f;
  
  penUp();

  while (isHome()==false) {
    step--;
    if (step<0) {
      step=15;   
    }
    PORTA = phase[step];            
    delay(3);
  }

  delay(50);

  while (isHome()==true) {
    step++;
    if (step>15) {
      step=0;   
    }
    PORTA = phase[step];            
    delay(3);
  }

  stepperOff();
  x=0;
}

///////////////////////////////////////////////////////////

// depth 1=180 5.5=0 (11 steps)
void updateDials() {
    speed = 500+(analogRead(SPEED)*11);
    pressure = 255-analogRead(PRESSURE);
    depth = 90-((180-analogRead(DEPTH))/5);
}

///////////////////////////////////////////////////////////

void moveToNoClip(int nx,int ny) {
  nx=constrain(nx,0,MAX_X);
  moveToLoop(nx,ny); 
}

void moveTo(int nx,int ny) {
  nx=constrain(nx,0,MAX_X);
  ny=constrain(ny,0,MAX_Y);
  moveToLoop(nx,ny);   
}

void moveToLoop(int nx,int ny) {
  while (nx != x || ny != y) {
    if (nx > x) {
      x++;   
    } 
    else if (nx < x) {
      x--;   
    }

    if (ny > y) {
      y++;   
    } 
    else if (ny < y) {
      y--;   
    }

    updateDials();
    stepperTick();        
    delayMicroseconds(speed);
  }    
}

int sgn(int x){
  return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}

void lineTo(int xend,int yend) {
  int xx, yy, t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, es, el, err;

  dx = xend - x;
  dy = yend - y;

  incx = sgn(dx);
  incy = sgn(dy);
  if(dx<0) dx = -dx;
  if(dy<0) dy = -dy;

  if (dx>dy)
  {
    pdx=incx; 
    pdy=0;    /* pd. ist Parallelschritt */
    ddx=incx; 
    ddy=incy; /* dd. ist Diagonalschritt */
    es =dy;   
    el =dx;   /* Fehlerschritte schnell, langsam */
  } 
  else
  {
    pdx=0;    
    pdy=incy; /* pd. ist Parallelschritt */
    ddx=incx; 
    ddy=incy; /* dd. ist Diagonalschritt */
    es =dx;   
    el =dy;   /* Fehlerschritte schnell, langsam */
  }

  xx = x;
  yy = y;
  err = el/2;
  //   moveTo(xx,yy);

  for(t=0; t<el; ++t) /* t zaehlt die Pixel, el ist auch Anzahl */
  {
    err -= es;
    if(err<0)
    {
      err += el;
      xx += ddx;
      yy += ddy;
    } 
    else
    {
      xx += pdx;
      yy += pdy;
    }
    moveTo(xx,yy);
  }
}

///////////////////////////////////////////////////////////

char buffer[64];
byte bufferPos = 0;

void parseCoords(boolean relative) {
  char * p = strchr(buffer, ',');
  if (p != NULL) {
    int ny = atoi(p + 1);
    *p = '\0';
    int nx = atoi(&buffer[2]);
    if (relative) {
        lineTo(x-nx,y+ny);    
    }else{
        lineTo(MAX_X-nx,ny);
    }
  }
}

void parseSerial(HardwareSerial ser) {
  if (strncmp("PU",buffer,2)==0) {
    updateDials();
    penUp();
    parseCoords(false);
  } 
  else if (strncmp("PD",buffer,2)==0) {
    updateDials();
    penDown();
    parseCoords(false);
  } 
  else if (strncmp("PR",buffer,2)==0) {
    penUp();
    parseCoords(true);
  } 
  else if (strncmp("PA",buffer,2)==0) {
    penUp();
    parseCoords(false);
  } 
  else if (strcmp("IN",buffer)==0) {
    beep(5);
  } 
  else if (strcmp("SP1",buffer)==0) {
    penUp();
  } 
  else if (strcmp("LOAD",buffer)==0) {
    loadPaper();    
  } 
  else if (strcmp("UNLOAD",buffer)==0) {
    unloadPaper();
    home();
  } 
  else if (strcmp("HOME",buffer)==0) {
    home();
  }
  else if (strcmp("UP",buffer)==0) {
    penUp();
  } 
  /*
  else if (strncmp("SV",buffer,2)==0) {
    int foo=atoi(&buffer[2]);
    myservo.write(foo);
  } 
  */
  else if (strcmp("BEEP",buffer)==0) {
    beep(10);
  } 
  else if (strcmp("?",buffer)==0) {
    ser.println("!");
  }
  else if (strcmp("pos",buffer)==0) {
    ser.print("x=");
    ser.print(x);
    ser.print(" y=");
    ser.println(y);
  }

  ser.write("#");
}

void serialLoop(HardwareSerial ser) {
  char r;

  while (ser.available() != 0) {
    r=ser.read();

    if (r=='\n' || r=='\r' || r==';') {
      if (bufferPos>0) {
        parseSerial(ser);
        pentimeout = millis();
      }
      bufferPos=0;
      return;
    }

    buffer[bufferPos] = r;
    buffer[++bufferPos] = '\0';

    if (bufferPos>=sizeof(buffer)) {
      bufferPos=0;
    }
  }
}


void echo() {
     if (Serial.available()) {
          Serial1.write(Serial.read());
     }   
     if (Serial1.available()) {
          Serial.write(Serial1.read());
     }   
}


///////////////////////////////////////////////////////////

void loop() {
  serialLoop(Serial);
  if (pentimeout + 60000 < millis()) {
       penUp();
  }
}
