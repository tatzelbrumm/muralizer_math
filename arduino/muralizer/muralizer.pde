/* command processing.
 *
 *  cubic curve    - C c0x c0y c1x c1y e1x e1y
 *  set dimensions - D x_max y_max
 *  motor goo      - G <controller> deg_per_step
 *  lineto         - L x y
 *  moveto         - M x y
 *  chattiness     - O <mask>
 *  # ... until eol ...
 *
 * controllers
 *  0 - gecko/direct attachment
 *  1 - adafruit motor controler
 */

#include <ctype.h>

// dimension of surface in steps
float x_max = 400.0;
float y_max = 600.0;

// initial position in steps
float r1 = 150.0;
float r2 = 320.0;

// length of strings in steps
float r1_min = 0.0;
float r1_max = 800.0;
float r2_min = 0.0;
float r2_max = 800.0;

float dr1_dt = 1;
float dr2_dt = 1;

// mumbling during command parsing
#define CHATTY_CMD  (1 << 0)
#define CHATTY_DRAW (1 << 1)
#define CHATTY_TIME (1 << 2)

static unsigned chatty;

/* compile-time selection of controllers */
#define CONTROLLER_GECKO 1
#define CONTROLLER_AF    1

/* stuff for gecko/direct attach controller */
#if CONTROLLER_GECKO
#define PIN_R1_DIR 12
#define PIN_R1_STROBE 13
#define PIN_R2_DIR 9
#define PIN_R2_STROBE 10

#define BLINK_PIN(x) \
  do{ \
    /* Serial.println(x);*/ \
    digitalWrite(x, HIGH); \
    delayMicroseconds(125); \
    digitalWrite(x, LOW); \
    delayMicroseconds(125); \
  } while(0)

#define R1_IN  do { digitalWrite(PIN_R1_DIR, 1); BLINK_PIN(PIN_R1_STROBE); } while(0)
#define R1_OUT do { digitalWrite(PIN_R1_DIR, 0); BLINK_PIN(PIN_R1_STROBE); } while(0)

#define R2_IN  do { digitalWrite(PIN_R2_DIR, 0); BLINK_PIN(PIN_R2_STROBE); } while(0)
#define R2_OUT do { digitalWrite(PIN_R2_DIR, 1); BLINK_PIN(PIN_R2_STROBE); } while(0)

static void
gecko_init(float n_steps)
{
  pinMode(PIN_R1_DIR , OUTPUT);
  pinMode(PIN_R1_STROBE, OUTPUT);
  pinMode(PIN_R2_DIR , OUTPUT);
  pinMode(PIN_R2_STROBE, OUTPUT);
}

static void
gecko_drive(int& dr1, int& dr2)
{
  if (dr1 < 0) { R1_IN; dr1++; }
  else if (dr1 > 0) { R1_OUT; dr1--; }

  if (dr2 < 0) { R2_IN; dr2++; }
  else if (dr2 > 0) { R2_OUT; dr2--; }
}
#endif /* CONTROLLER_GECKO */

/* stuff for ada fruit motor controller */
#if CONTROLLER_AF
#include <AFMotor.h>

static AF_Stepper af_r1, af_r2;

void* operator new(size_t, class AF_Stepper* s)
{
  return (void*)s;
}

static void
af_init(float n_steps)
{
  new (&af_r1) AF_Stepper(n_steps, 1);
  new (&af_r2) AF_Stepper(n_steps, 2);
}

static void
af_drive(int& dr1, int& dr2)
{
  uint8_t dir;

  if (dr1) {
    dir = (dr1 < 0);
    af_r1.onestep(FORWARD + dir, SINGLE);
    dr1 += (dir ? 1 : -1);
  }

  if (dr2) {
    dir = (dr2 < 0);
    af_r1.onestep(FORWARD + dir, SINGLE);
    dr2 += (dir ? 1 : -1);
  }
}
#endif /* CONTROLLER_AF */

/* all our contrllers. */
static const struct {
  void (*init)(float n_steps);
  void (*fini)(void);
  void (*drive)(int&, int&);
} controllers[] = {
#if CONTROLLER_GECKO
  { gecko_init, NULL, gecko_drive },
#endif
#if CONTROLLER_AF
  { af_init, NULL, af_drive },
#endif
},
  /* controllers[0] is the default until changed w/
   * the g/G command.
   */
*controller = controllers + 0;

float x(float r1, float r2) {
  float x = (x_max*x_max + r1*r1 - r2*r2)/(2.0*x_max);

  return x;
}

float y(float r1, float r2) {
  float x = (x_max*x_max + r1*r1 - r2*r2)/(2.0*x_max);

  float y = sqrt(r1*r1 - x*x);

  return y;
}

// similarly, this works for all dimensions
float b3(float p0, float p1, float p2, float p3, float t) {
  return p0 + t*(p1 - 3*p0 + t*(3*p0 - 2*p1 + p2 + t*(p1+p3-p0-p2)));
}

float calc_r1(float x, float y) {
  return sqrt(x*x + y*y);
}


float calc_r2(float x, float y) {
  float dx= x_max-x;
  return sqrt(dx*dx + y*y);
}

int dr1 = 0;
int dr2 = 0;
float t = 0.0;
float t_steps = 1000.0;

float xs[] = { 100, 200, 300, 350 };
float ys[] = { 100, 400, 250, 350 };

static int
get_char()
{
  while(Serial.available() == 0)
    delay(250);
  return Serial.read();
}

/* reads serial data for a floating point value
 * leaves the 'cursor' after the seperator.
 *
 * FixMe - this isn't completely general purpose since we're not
 * expecting general purpose floating point values.
 */
static float
get_arg()
{
  int sign_p;
  int whole;
  int frac, frac_n;
  int c, done_p;
  float rv;

  /* skip any leading whitespace */
  do {
    c = get_char();
  } while(::isspace(c));

  if ((sign_p = (c == '-')) || (c == '+'))
    c = get_char();

  for(whole = 0; /**/; whole *= 10) {
    whole += (c - '0');
    c = get_char();
    if ((done_p = ((c == 0x20) || (c == 0x0A) || (c == 0x0D))) ||
        (c == '.'))
      break;
  }

  /* if we ended the integer part w/ whitespace then no fractional bits */
  if (done_p) {
    frac_n = 0;
  } else {
    for(frac = 0, frac_n = 10; /**/; frac *= 10, frac_n *= 10) {
      frac += (c - '0');
      c = get_char();
      if (done_p = ((c == 0x20) || (c == 0x0A) || (c == 0x0D)))
        break;
    }
  }

  /* integer part */
  rv = float(whole);

  /* any fractional bits? */
  if (frac_n != 0)
    rv += float(frac) / float(frac_n);

  /* signed-ness? */
  if (sign_p) rv *= -1.0;

  return rv;
}

static int
get_next_cmd(float* args)
{
  int cmd, i, nargs;

  while(1) {
    Serial.println();
    Serial.print("> ");

    cmd = get_char();
    if (chatty & CHATTY_CMD)
      Serial.print(cmd, BYTE);

    switch(cmd = tolower(cmd)) {
      /* comment, read until eol */
    case '#':
      /* NB: we assume that LF is the newline character so that we can
       * deal w/ LF or CR+LF, but LF+CF hits the 'unknown command' case.
       */
      while(1) {
        cmd = get_char();
        if (cmd == 0x0A) break;
      }
      /* falltrhough */

      /* unrecognized command */
    default:
      if (chatty & CHATTY_CMD)
        Serial.println();
      continue;

    case 'o':
      nargs = 1;
      break;

    case 'd':
    case 'g':
    case 'l':
    case 'm':
      nargs = 2;
      break;

    case 'c':
      nargs = 6;
      break;
    }
    break;
  }
  if (chatty & CHATTY_CMD) {
    Serial.print(':', BYTE);
    Serial.print(nargs, DEC);
  }
  for(i = 0; i < nargs; i++) {
    args[i] = get_arg();
    if (chatty & CHATTY_CMD) {
      Serial.print(' ', BYTE);
      Serial.print(args[i]);
    }
  }
  if (chatty & CHATTY_CMD)
    Serial.println();

  return cmd;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Welcome to Muralizer...");
}

void loop() {
  int cmd;
  float args[6];

  /* check for command data */
next_cmd:
  cmd = get_next_cmd(args);

  switch(cmd) {
  /* non-drawing commands come first */
  case 'd':
    x_max = args[0];
    y_max = args[1];
    goto next_cmd;

  case 'g':
    if ((args[0] >= 0.0f) &&
        (args[0] < sizeof(controllers) / sizeof(controllers[0])))
    {
      if ((controller != NULL) && (controller->fini != NULL))
        controller->fini();
      controller = controllers + int(args[0]);
      if (controller->init != NULL)
        controller->init(args[1]);
    }
    goto next_cmd;

  case 'o':
    chatty = int(args[0]);
    goto next_cmd;

  /* drawing commands start follow */

    /* cubic from current point
     *
     * control point 0 - args[0], args[1]
     * control point 1 - args[2], args[3]
     * ending end point 1 - args[4], args[5]
     */
  case 'C':
  case 'c':
    xs[1] = args[0];
    ys[1] = args[1];
    xs[2] = args[2];
    ys[2] = args[3];
    xs[3] = args[4];
    ys[3] = args[5];
    break;

    /* move the pen around.
     *
     * FxMe - should this be lazy? so that we don't do the move until
     * an actual drawing command comes in?
     *
     * FixMe - since we can't lift the pen we just fallthrough and
     * draw a line to the new position.
     */
  case 'M':
  case 'm':

    /* line from current point to args[0],args[1]
     *
     * FixMe: currently we use a degenerate cubic.
     */
  case 'L':
  case 'l':
    xs[1] = xs[0];
    ys[1] = ys[0];
    xs[2] = args[0];
    ys[2] = args[1];
    xs[3] = args[0];
    ys[3] = args[1];
    break;
  }

  /* now run through the buffer of commands */
  for(t = 0.0; t < t_steps; /**/) {
    if (dr1 == 0 && dr2 == 0) { // No motion is pending
      float x_t = b3(xs[0],xs[1],xs[2],xs[3], t/t_steps);
      float y_t = b3(ys[0],ys[1],ys[2],ys[3], t/t_steps);

      float r1_t = calc_r1(x_t, y_t);
      float r2_t = calc_r2(x_t, y_t);

      dr1 = (int)(r1_t - r1);
      dr2 = (int)(r2_t - r2);

      if (chatty & CHATTY_DRAW) {
        Serial.print("x_t = "); Serial.print(x_t);
        Serial.print("; y_t = "); Serial.print(y_t);
        Serial.print("r1_t = "); Serial.print(r1_t);
        Serial.print("; r2_t = "); Serial.print(r2_t);
      }

      t++;
    } else { // We need to move the head along
      if (chatty & CHATTY_TIME) {
        Serial.print("dr1 = "); Serial.print(dr1);
        Serial.print("; dr2 = "); Serial.print(dr2);
        Serial.print("; t = "); Serial.println(t);
      }

      controller->drive(dr1, dr2);

      if (r1 >= r1_max) { r1 = r1_max-1.0; dr1 = 0; }
      if (r1 < r1_min) { r1 = r1_min; dr1 = 0; }

      if (r2 >= r2_max) { r2 = r2_max-1.0; dr2 = 0; }
      if (r2 < r2_min) { r2 = r2_min; dr2 = 0; }
    }
  }

  /* last point becomes the new starting point */
  xs[0] = xs[3];
  ys[0] = ys[3];
}
