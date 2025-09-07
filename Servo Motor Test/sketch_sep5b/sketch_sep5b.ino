#include <Servo.h>
Servo servol;
void setup()
{
 servol.attach(D6, 500, 2400);
// servol.attach(D6); 
}

void loop() 
{
 servol.write(0);
 delay(1000);
 servol.write(90);
 delay(1000);
 servol.write(180);
 delay(1000);
  
}