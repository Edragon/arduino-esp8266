

#include <TaskScheduler.h>


#define LED 13
#define relay 12

// Callback methods prototypes
void rlCallback();
void ldCallback();
void tmCallback();
//Tasks
Task RL(5000, TASK_FOREVER, &rlCallback);
Task LD(2000, TASK_FOREVER, &ldCallback);
Task TM(2000, TASK_FOREVER, &tmCallback);


bool Relay_state = false;
bool LED_state = false;

Scheduler runner;

void rlCallback() {
  if ( Relay_state ) {
    digitalWrite(relay, HIGH);
    Relay_state = false;
  }
  else {
    digitalWrite(relay, LOW);
    Relay_state = true;
  }
}

void ldCallback() {
  if ( LED_state ) {
    digitalWrite(LED, HIGH);
    LED_state = false;
  }
  else {
    digitalWrite(LED, LOW);
    LED_state = true;
  }
}


void tmCallback() {
  Serial.print("tm: ");
  Serial.println(millis());
}



void setup () {
  Serial.begin(115200); 
  pinMode(LED, OUTPUT);
  pinMode(relay, OUTPUT);

  runner.init();
  runner.addTask(RL);
  runner.addTask(LD);
  runner.addTask(TM);
  delay(2000);
  RL.enable();
  LD.enable();
  TM.enable();
  Serial.println("Initialized scheduler");

}


void loop () {
  runner.execute();

}
