// for the relays
const int relay1 = 2;
const int relay2 = 3;
const int relay3 = 4;
const int relay4 = 5;
int relays[4] = { relay1, relay2, relay3, relay4 };

// briefly turns on a given relay
void turnOnRelay(int relay) {
  for (int i = 0; i < 10; i++) {
    digitalWrite(relays[relay - 1], LOW);
    delay(50);
    digitalWrite(relays[relay - 1], HIGH);
    delay(50);
  }
}

void setup() {
  // establish the relays as outputs
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);

  // ensure all of them are off
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);

  Serial.begin(9600);
}

void loop() {
  // to determine which relay to turn on
  int relay;

  if (Serial.available() > 0) {
    relay = Serial.parseInt();
    turnOnRelay(relay);
  }
}