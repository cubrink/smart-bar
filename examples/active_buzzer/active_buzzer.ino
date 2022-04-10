// test active buzzer on GIOP36

const int buzzer_pin = 33;

void setup() {
  Serial.begin(115200);

  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);

}

void loop() {
  digitalWrite(buzzer_pin, HIGH);
  Serial.println("buzzer ON");
  delay(500);
  digitalWrite(buzzer_pin, LOW);
  Serial.println("buzzer OFF");
  delay(500);

}
