

/* Pin assignments */
#define INTERRUPT_INPUT 14
int pulse_counter = 0;

//void ICACHE_RAM_ATTR handleInterrupt();

void ICACHE_RAM_ATTR interrupt_handler ()
{
  pulse_counter = pulse_counter + 1;
}

void setup()
{
  Serial.begin(9600);

  // For noise suppression, enable pullup on interrupt pin
  digitalWrite(INTERRUPT_INPUT, HIGH);
  attachInterrupt(INTERRUPT_INPUT, interrupt_handler, RISING);
}


void loop() {

  if (pulse_counter > 0)
  {
    Serial.println(pulse_counter);
    delay(100);
  }
}
