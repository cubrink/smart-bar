#include <cppQueue.h>

const int PRESSURE_QUEUE_MAX_SIZE = 10;
cppQueue q(sizeof(int), PRESSURE_QUEUE_MAX_SIZE, FIFO);
float q_sum = 0.0;

void setup() {
  Serial.begin(115200);
  //q.flush();

  delay(2000);
}

void loop() {
  if(q.isFull())
  {
    Serial.println("Queue Full!");
    int popped = 0;
    q.pull(&popped);
    Serial.print("Popped: ");
    Serial.println(popped);
    q_sum -= popped;
  }

  int my_rand_num = random(1,100);
  q.push(&my_rand_num);
  Serial.print("Pushed: ");
  Serial.println(my_rand_num);
  q_sum += my_rand_num;

  Serial.print("Count: ");
  Serial.println(q.getCount());
  Serial.print("AVG: ");
  Serial.println(float(q_sum) / float(q.getCount()));

  
  delay(2000);
}
