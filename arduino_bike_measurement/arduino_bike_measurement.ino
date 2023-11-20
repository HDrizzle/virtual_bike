/* Compatible with the virtual_bike rust crate, sends the following information in each line over serial at 9600 baud
 * all times are in microseconds
 * <diff between most recent pulses>,<time since most recent pulse>,<steering>,<brake (in G-force)>
 * TODO: measure brakes
 * 2023-11-3: changed units to microseconds because I am now using an optical sensor with the trainer flywheel which is really fast and millisecs are just not precise enough
*/

#define TACH_PIN 9
#define POT_PIN A0
#define DELAY_T 200// Millis between serial updates
//#define DEBOUNCE_T 40// millisecs

uint32_t t_last_update = 0;
uint32_t t_1 = 0;// earliest pulse on record
uint32_t t_2 = 0;// most recent pulse
bool prev_tach_state = false;
bool tmp_sample = false;

float get_steering() {
  return (float)(analogRead(POT_PIN) - 512) / 512.0;
}

void send_data() {
  Serial.println(String(t_2 - t_1) + "," + String(micros() - t_2) + "," + String(get_steering()) + ",0");// TODO
  t_last_update = millis();
}

void setup() {
  Serial.begin(9600);
  pinMode(TACH_PIN, INPUT_PULLUP);
}

void loop() {
  // Check for rising edge
  prev_tach_state = tmp_sample;
  tmp_sample = digitalRead(TACH_PIN);
  if(!prev_tach_state && tmp_sample) {// Rising edge, doesn't matter though
    t_1 = t_2;
    t_2 = micros();
    // TODO: debouncing
  }
  // Update to serial
  if(millis() - t_last_update > DELAY_T) {
    send_data();
  }
}
