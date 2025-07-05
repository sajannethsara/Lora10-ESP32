#include <Arduino.h>
#include "Foo.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Foo fooInstance;
  fooInstance.doSomething();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}