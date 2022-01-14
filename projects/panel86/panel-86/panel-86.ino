
#define input_1 16
#define input_2 3 // RXD should be disabled
#define input_3 4
#define input_4 5

#define output_1 14
#define output_2 12
#define output_3 13
#define output_4 15

int  op_flag1 = 0;
int  op_flag2 = 0;
int  op_flag3 = 0;
int  op_flag4 = 0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(input_1, INPUT);
  pinMode(input_2, INPUT);
  pinMode(input_3, INPUT);
  pinMode(input_4, INPUT);

  pinMode(output_1, OUTPUT);
  pinMode(output_2, OUTPUT);
  pinMode(output_3, OUTPUT);
  pinMode(output_4, OUTPUT);

  digitalWrite(output_1, LOW);
  digitalWrite(output_2, LOW);
  digitalWrite(output_3, LOW);
  digitalWrite(output_4, LOW);

  //Serial.begin(115200);
  //Serial.println("");
  //Serial.println("start ..");


}



// the loop function runs over and over again forever
void loop() {
  read_key1();
  read_key2();
  read_key3();
  read_key4();

}



void read_key1() {
  if (digitalRead(input_1) == LOW) {

    flip1();
    delay(2);
    if ( op_flag1 == 0) {
      digitalWrite(output_1, HIGH); // turn on blue led
    }
    if ( op_flag1 == 1) {
      digitalWrite(output_1, LOW);   //
    }
  }
}

void read_key2() {
  if (digitalRead(input_2) == LOW) {

    flip2();
    delay(2);

    if ( op_flag2 == 0) {
      digitalWrite(output_2, HIGH); // turn on blue led
    }
    
    if ( op_flag2 == 1) {
      digitalWrite(output_2, LOW);   //
    }
  }
}


void read_key3() {
  if (digitalRead(input_3) == LOW) {

    flip3();
    delay(2);

    if ( op_flag3 == 0) {
      digitalWrite(output_3, HIGH); // turn on blue led
    }
    if ( op_flag3 == 1) {
      digitalWrite(output_3, LOW);   //
    }
  }
}
void read_key4() {
  if (digitalRead(input_4) == LOW) {

    flip4();
    delay(2);

    if ( op_flag4 == 0) {
      digitalWrite(output_4, HIGH); // turn on blue led
    }
    if ( op_flag4 == 1) {
      digitalWrite(output_4, LOW);   //
    }
  }
}

void flip1() {
  if (op_flag1 == 0) {
    op_flag1 = 1;
  } else {
    op_flag1 = 0;
  }
}

void flip2() {
  if (op_flag2 == 0) {
    op_flag2 = 1;
  } else {
    op_flag2 = 0;
  }
}

void flip3() {
  if (op_flag3 == 0) {
    op_flag3 = 1;
  } else {
    op_flag3 = 0;
  }
}

void flip4() {
  if (op_flag4 == 0) {
    op_flag4 = 1;
  } else {
    op_flag4 = 0;
  }
}
