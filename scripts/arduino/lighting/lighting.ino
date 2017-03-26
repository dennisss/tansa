#include "SoftwareSerial.h"
#include "Adafruit_Pixie.h"

/// PREDEFINED Colors which you can use by name instead of the 3 color syntax
#define Red {255, 0, 0}
#define Green {0, 255, 0}
#define Blue {0, 0, 255}
#define White {255, 255, 255}
#define Yellow {255, 255, 0}
#define Purple {255, 0, 255}
#define Black {0, 0, 0}



#define NUMPIXELS 2
#define PIXIEPIN  8

SoftwareSerial pixieSerial(-1, PIXIEPIN);
Adafruit_Pixie strip = Adafruit_Pixie(NUMPIXELS, &pixieSerial);


int nEnabled = 2;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} Color;

Color Intensity(Color c, float percent) { return { c.r*percent, c.g*percent, c.b*percent }; }


#define LOOP(n, x) for(int ni = 0; ni < n; ni++) { x }

void setcolor(Color color){ for(int i=0; i < nEnabled; i++) strip.setPixelColor(i, color.r, color.g, color.b); strip.show(); }
Color interp(Color c1, Color c2, float a) {
  float a2 = 1.0f - a;
  return { (c1.r*a + c2.r*a2), (c1.g*a + c2.g*a2), (c1.b*a + c2.b*a2) };
}

void SetNumLEDS(int n) { nEnabled = n; }


void Hold(Color c, float t) { for(float ti = 0; ti < t; ti += 0.01) { setcolor(c); delay(10); } }

void Fade(Color c1, Color c2, float t) {
  for(float ti = 0; ti < t; ti += 0.01) {
    float a = 1.0 - (ti / t);
    setcolor(interp(c1, c2, a));
    delay(10);
  }
}

void FlashAdvanced(Color c1, Color c2,  int hz1, int hz2, double t) {
  for(float ti = 0; ti < t; ti += 0.01) {
    float a = ti / t;
    int hz = (1.0f - a) * hz1 + a * hz2;
    Color c = interp(c1, c2, 1.0f - a);

    float tp = 1.0f / hz;
    int i = floor(ti / tp);

    if(i % 2 == 0) {
      setcolor(c);
    } else { setcolor({0,0,0}); }

    delay(10);
  }
  
}

void Flash(Color c, float hz, float t) {
  float tp = 1.0f / hz;
  for(float ti = 0; ti < t; ti += 0.01) {
    int i = floor(ti / tp);
    if(i % 2 == 0) {
      setcolor(c);
    } else { setcolor({0,0,0}); }
    
    delay(10);
  }
}


void setup() {
  pixieSerial.begin(115200);
  strip.setBrightness(200);
}

void loop() {
  // Place the light choreography here

  // Use one of the 2 leds
  SetNumLEDS(1);

  // Flash at blue at 10 Hz for 2 seconds
  Flash({0, 0, 255}, 10, 2); // <- You need a semicolon after most commands

  // Fade from red to blue in 4 seconds
  Fade({255, 0, 0}, {0, 0, 255}, 4);

  // Hold at green for 2 seconds
  Hold({0, 255, 0}, 2);


  // Advanced flash between 2 colors and 2 frequencies
  // Go from Yellow 2Hz to Blue 20Hz to 10 seconds
  // Note: you could have done '{0, 0, 255}' instead of writing 'Blue'
  FlashAdvanced(Yellow, Blue, 2, 20, 10);

  // Use both LEDs for sequential commands
  SetNumLEDS(2);

  // This is an example of fading with intensities (note: anywhere you can put a color C, you can also put Intensity(C, Percentage), where percentage goes from 0 - 1)
  // Fades from 20% White to 80% White over 4 seconds
  // Then it does the yellow hold for 2 seconds and loops that sequence 6 times
  LOOP(6,
    Fade(
      Intensity({255, 255, 255}, 0.2),
      Intensity({255, 255, 255}, 0.8),
      4
    );
    Hold({255, 255, 0}, 2);
  ) // Note that you don't need a semicolon after a loop


/*  <- You can comment/disable a block of commands like this
  Hold({0, 255, 0}, 2);

  Hold({0, 255, 0}, 2);

*/

  // Put a // in front of the next line if you want the choreography to auto-loop
  while(1) { delay(1000); }
}
