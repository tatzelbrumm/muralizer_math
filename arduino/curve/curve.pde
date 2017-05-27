/* Draw a curve that will result in a straight line using the Bresenham algorithm */

/* CM 2009NOV23 add subroutines to drive stepper motors directly from digital outputs */
/* CM 2009JUL04 */
/* CM 2009MAY27 */
/* CM 2009MAY25 */

/* allow for easy exchange of float and double formats*/
#define real double

/* Function pointers will only work if we sneak them in through a header */
#include "curve.h"

const byte LED_PIN=13;
const int wait=20;

/* attachment points of threads */
real attach_p[4]=
{
  -10000, 0,
   10000, 0
};
real *attach_pt[2]=
{
  attach_p,
  attach_p+2
};

/* start and end of line to draw */
real start[2]= {-7000,-12000};
real end[2]= {8000,-2000};

/* safeguard: maximum number of iterations */
static int maxiterations= 32767;

/* stepper motor interface setup */
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

/* sign function that never returns zero */
static inline int sign(real in)
{
  return in < 0 ? -1 : 1;
}

/* step function that operates stepper motor */
static inline void schritt(int richtung, int achse)
{
  int *pinset;
  pinset= pins[achse];
  Serial.print(richtung>=0 ? '+' : '-');
  halbschritt(richtung>=0, stepindex+achse, pinset);
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
//  delay(wait);
  digitalWrite(LED_PIN, LOW);
//  delay(wait);
}

/* round floating point vector to integer */
static inline void round_vector(real *x, int *k)
{
	int i;
	for (i= 0; i<2; i++)
	{
		k[i]= lround(x[i]);
	}
}

/* transform cartesian coordinates to radii */
static inline void cartesian_to_radii(real **attach_point, real *cartesian, real *radii)
{
	static real d[2][2];
	real accu;
	int i,j;
	for (i= 0; i<2; i++)
	{
		for (j=0; j<2; j++)
		{
			d[i][j]= cartesian[j]-attach_point[i][j];
		}
		radii[i]= hypot(d[i][0],d[i][1]);
	}
}

/* transforma cartesian coordinates to radii and convert to integer */
static inline void round_cartesian_to_radii(real **attach_point, real *cartesian, int *radii)
{
	real floatingpoint[2];
	cartesian_to_radii(attach_point, cartesian, floatingpoint);
	round_vector(floatingpoint, radii);
}

/* criterion to continue drawing a line */
static inline int continueDrawing(int *point, int *endpoint, real *gradient)
{
	int v[2];	// distance vector between point and endpoint
	real d;		// projection of distance on curve tangent
	int i;

	for (i= 0; i<2; i++)
	{
		v[i]= point[i]-endpoint[i];
	}
	d= gradient[0]*v[1]-gradient[1]*v[0];
	return d*d*2 > gradient[0]*gradient[0]+gradient[1]*gradient[1];
}

/* Bresenham algorithm for the muralizer curve */
int bresenham_muralizer(  // return number of points drawn
  real *start,
  real *end,
  real **attach_point)
{
	int iterations;

	static real X0[2];
	static real DX[2];
	static real x[2];
	static real dx[2];
	static real skew[2];
	static real dx2[2];
	real D2;
	real d2;
	real DXdx;
	real DXdx2;
	real aDXdx;  // antisymmetric term
	real askew;  // antisymmetric skew term
	real sV;     // symmetric vector term
	real aV;     // antisymmetric vector term
	static real V[2];   // vector elements
	real C;      // constant
real a, d, e;

	int i,j;

	real discriminant;
	static real gradient[2];
	static int point[2];
	static int endpoint[2];

	static real R[2];	// coordinates squared
	real M;		// matrix term
	int dir;	// direction of line drawing

	static int stepdir[2];	// direction of step
	int fastidx;
	int slowidx;
	static int newpoint[2][2];
	static real newR[2][2];
	static real newdiscriminant[2];
	int selection;

	// set up coefficients
	for (i= 0; i<2; i++)
	{
		DX[i]= attach_point[1][i]-attach_point[0][i];
		X0[i]= (attach_point[0][i]+attach_point[1][i])/2;
		dx[i]= end[i]-start[i];
		x[i]= (end[i]+start[i])/2;
		skew[i]= x[i]-X0[i];
		dx2[i]= dx[i]*dx[i];
	}
	D2= DX[0]*DX[0]+DX[1]*DX[1];
	d2= dx2[0]+dx2[1];
	DXdx= DX[0]*dx[0]+DX[1]*dx[1];
	DXdx2= DXdx*DXdx;
	aDXdx= DX[0]*dx[1]-DX[1]*dx[0];
	askew= skew[0]*dx[1]-skew[1]*dx[0];
	sV= DXdx2*(-2);
	aV= aDXdx*askew*4;

	V[0]= sV - aV;
	V[1]= sV + aV;

	C= D2*(DXdx2+askew*askew*4);
	a= d2/C;
	d= V[0]/C;
	e= V[1]/C;
Serial.println(a);
Serial.println(d);
Serial.println(e);

	// set up initial conditions
	round_cartesian_to_radii(attach_point, end, endpoint);
	round_cartesian_to_radii(attach_point, start, point);

	draw(point[0],point[1]);
	draw(endpoint[0],endpoint[1]);

	for (i= 0; i<2; i++)
	{
		R[i]= real(point[i])*point[i];
	}
Serial.println(R[0]);
Serial.println(R[1]);
	dir= sign(DXdx);
	M= a*(R[0]-R[1]);
Serial.println(M);
Serial.println(d);
Serial.println(e);
	discriminant= R[0]*(d+M) + R[1]*(e-M) + 1.0;
Serial.println(discriminant);
	gradient[0]= a*R[0] + M + d;
	gradient[1]= a*R[1] - M + e;

	// iteration
	for (iterations= 0; continueDrawing(point, endpoint, gradient) && iterations < maxiterations; iterations++)
	{
		stepdir[0]= -dir*sign(gradient[1]);
		stepdir[1]=  dir*sign(gradient[0]);
//draw(stepdir[0],stepdir[1]);

		if (abs(gradient[0]*point[0]) < abs(gradient[1]*point[1]))
		{
			fastidx= 0;
		}
		else
		{
			fastidx= 1;
		}
		slowidx= 1 - fastidx;

		for (i= 0; i<2; i++)
		{
			newpoint[i][fastidx]= point[fastidx] + stepdir[fastidx];
		}
		newpoint[0][slowidx]= point[slowidx];
		newpoint[1][slowidx]= point[slowidx] + stepdir[slowidx];
		for (i= 0; i<2; i++)
		{
			for (j= 0; j<2; j++)
			{
				newR[i][j]= real(newpoint[i][j])*newpoint[i][j];
			}
			M= a*(newR[i][0]-newR[i][1]);
			newdiscriminant[i]= newR[i][0]*(d+M) + newR[i][1]*(e-M) + 1.0;
		}
		if (abs(newdiscriminant[0]) < abs(newdiscriminant[1]))
		{
			selection= 0;
		}
		else
		{
			selection= 1;
		}
		for (i= 0; i<2; i++)
		{
			point[i]= newpoint[selection][i];
			R[i]= newR[selection][i];
		}
		discriminant= newdiscriminant[selection];
		M= a*(R[0]-R[1]);
		gradient[0]= a*R[0] + M + d;
		gradient[1]= a*R[1] - M + e;

		schritt(stepdir[fastidx],fastidx);
		if (selection==1)
		{
			schritt(stepdir[slowidx],slowidx);
		}
		draw(point[0],point[1]);
	}
	return iterations;
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
  setup_step();  
  int pixels;

  if (true)
  {
     pixels= bresenham_muralizer(start, end, attach_pt);

     Serial.print("Pixels drawn: ");
     Serial.println(pixels);
  }
  finish_step();
}

/* arduino loop function */
void loop()
{
    ;
}
