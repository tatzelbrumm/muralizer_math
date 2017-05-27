/* Step motor drive (direct drive from digital outputs) */

/* CM 2009NOV23 add subroutines to drive stepper motors directly from digital outputs */

/* stepper motor interface setup */
#include "WProgram.h"
static void drive_pins(int pattern, int *p);
static inline void setup_step();
static inline void finish_step();
static inline int halbschritt(int richtung, int *index, int *p);
void setup();
void loop();
int move= 0;
int dir= 1;
int pins_nums[2][4]=
{
  {4, 5, 6, 7},
  {8, 9, 10, 11}
};
int *pins[2]= 
{
  pins_nums[0],
  pins_nums[1]
};

int richtung[2]=
{0, 1};

int omega= 1;

long dividend= 1048576;

long maxomega= dividend/262144;

int stepindex[2];  /* step indices for axes */
int halfstep[8]= { 1, 3, 2, 6, 4, 12, 8, 9 };  /* active outputs for half step sequences */

static void drive_pins(int pattern, int *p)
{
  int pin;
  // Serial.print("[");
  for (pin= 0; pin<4; pin++)
  {
    // Serial.print(pattern & (1<<pin) ? "L" : "H");
    digitalWrite(p[pin], pattern & (1<<pin) ? LOW : HIGH);
  }
  // Serial.println("]");
}

static inline void setup_step()
{
  int axis;
  int pin;
  for (axis= 0; axis<2; axis++)
  {
    for (pin= 0; pin<4; pin++)
    {
      int p= pins[axis][pin];
      pinMode(p, OUTPUT);
      digitalWrite(p, HIGH);
    }
    drive_pins(halfstep[stepindex[axis]= 0], pins[axis]); 
  }
}

static inline void finish_step()
{
  int axis;
  int pin;
  for (axis= 0; axis<2; axis++)
  {
    for (pin= 0; pin<4; pin++)
    {
      int p= pins[axis][pin];
      digitalWrite(p, HIGH);
      pinMode(p, INPUT);
    }
  }
}

static inline int halbschritt(int richtung, int *index, int *p)
{
  int i;
  i= (*index + (richtung? -1 : 1)) & 7;
  *index = i;
  drive_pins(halfstep[i], p);
  return i;
}

void setup()                    // run once, when the sketch starts
{
  setup_step();
}

void loop()                     // run over and over again
{
  int axis;
  long stepdelay;
  volatile long d;
  for (axis= 0; axis<2; axis++)
  {
    halbschritt(richtung[axis], stepindex+axis, pins[axis]);
  }
  stepdelay= dividend/omega;
  for (d= 0; d<stepdelay; d++);
  if (omega <= maxomega)
  {
    ++omega;
  } 
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

