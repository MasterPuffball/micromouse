/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

#define LED_PIN 9
#define BUTTON_PIN 2

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

// the loop function runs over and over again forever
void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // digitalWrite(LED_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // digitalWrite(LED_PIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000);                      // wait for a second

  if (digitalRead(BUTTON_PIN) == LOW) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// Answer to:
// Q1: 3.3v
// Q2: Since we are making our button nominally open, when the button is not pressed the pin isnt connected to anything and is therefore "floating".
//     if Pullup wasnt specified the pin would wait until a certain signal is reached, but since the pin is floating, this may occur even when the button isn't pressed.
//     Therefore by making the pin 'pullup' it ensure that until the button is depressed, the pin will read a HIGH value. Then once the button is pressed, the pin becomes grounded
//     and a logic LOW is registered in the pin.
// Q3: Pull down resistors are increadibly easy to add to a circuit externally and only really increase cost when manufacturing a pcb so arduino has decided to leave the use
//     of a pull down resistor up to the user to place on a circuit.
// Q4: If you at any point want to have a floating pin that is default high, an external resistor should be placed to link the boards 3.3V pin to the desired pulled up pin.