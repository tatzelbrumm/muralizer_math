/* Draw a straight line using the Bresenham algorithm */

/* CM 2009MAY25 */

/* Function pointers will only work if we sneak them in through a header */
#include "straightline.h"

#include "WProgram.h"
static inline int sign(int in);
static inline void schritt(int richtung, int achse);
static inline void draw(int x, int y);
int bresenham_line(     
  int *start,           
  int *end);
void cereal_init(int baudrate);
void setup();
void loop();
const byte LED_PIN=13;
const int wait=20;

/* start and end of line to draw */
int start[2]= {-40,50};
int end[2]= {20,-30};

/* sign function that never returns zero */
static inline int sign(int in)
{
  return in < 0 ? -1 : 1;
}

/* step function that operates stepper motor */
static inline void schritt(int richtung, int achse)
{
  Serial.print(richtung<0 ? '-' : '+');
  Serial.println(achse==0 ? 'x' : 'y');
}

/* draw function that dumps the information to the serial port */
static inline void draw(int x, int y)
{
  digitalWrite(LED_PIN, HIGH);
  Serial.print("(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println(")");
  delay(wait);
  digitalWrite(LED_PIN, LOW);
  delay(wait);
}

/* Bresenham algorithm for a straight line */
int bresenham_line(    // return number of points drawn
  int *start,          // start coordinates
  int *end)            // end coordinates
{
  int n;
  int Dx[2];
  int dx[2];
  int x[2];
  int derror[2];
  int error;
  int i;
  int fast;
  int slow;
  int threshold;

  for (i=0; i<2; i++)
  {
    Dx[i]= end[i] - start[i];
    dx[i]= sign(Dx[i]);
    x[i]= start[i];
  }
  fast= abs(Dx[0]) > abs(Dx[1])? 0 : 1;
  slow= 1 - fast;
  threshold= abs(Dx[fast]);
  derror[fast]= abs(Dx[slow]) * 2;
  derror[slow]= -threshold * 2;
  error= 0; 
  draw(x[0], x[1]);
  for (n= 1; x[0] != end[0] || x[1] != end[1]; n++)
  {
    schritt(dx[fast], fast);
    x[fast]+= dx[fast];
    error+= derror[fast];
    if (error > threshold)
    {
      schritt(dx[slow], slow);
      x[slow]+= dx[slow];
      error+= derror[slow];
    }
    draw(x[0], x[1]);
  }
  return n;
}

/* initialize serial interface */
void cereal_init(int baudrate)
{
  byte k;

  pinMode(LED_PIN, OUTPUT);
  for (k=0; k<5; k++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(wait<<2);
    digitalWrite(LED_PIN, LOW);
    delay(wait<<2);
  }
  Serial.begin(baudrate);
}

/* arduino setup function */
void setup()
{
  cereal_init(9600);
  if (true)
  {
    bresenham_line(start,end);
  }
}

/* arduino loop function */
void loop()
{
    ;
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

