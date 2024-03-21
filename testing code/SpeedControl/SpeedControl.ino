// for the relays
const int power = 2;
const int speedUp = 3;
const int speedDown = 4;
const int enter = 5;

// turns the launcher on/off
void togglePower() {
  digitalWrite(power, !digitalRead(power));
}

// pulses a given relay to simulate a button hold
void changeSpeed(int relay) {
  for (int i = 0; i < 10; i++) {
    digitalWrite(relay, LOW);
    delay(50);
    digitalWrite(relay, HIGH);
    delay(50);
  }
}

void setup() {
  // establish the relays as outputs
  pinMode(power, OUTPUT);
  pinMode(speedUp, OUTPUT);
  pinMode(speedDown, OUTPUT);
  pinMode(enter, OUTPUT);

  // ensure all of them are off
  digitalWrite(power, HIGH);
  digitalWrite(speedUp, HIGH);
  digitalWrite(speedDown, HIGH);
  digitalWrite(enter, HIGH);

  Serial.begin(9600);
}

void loop() {
  // to determine what to do
  int command;

  // read what we want to do from the serial monitor
  if (Serial.available() > 0) {
    command = Serial.parseInt();

    if (command == 1) {
      togglePower();
    } else if (command == 2) {
      changeSpeed(speedUp);
    } else if (command == 3) {
      changeSpeed(speedDown);
    }
  }
}