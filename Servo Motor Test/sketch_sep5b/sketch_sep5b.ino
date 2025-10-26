#include <Servo.h>
Servo servol;
void setup()
{
 servol.attach(D0, 500, 2400);
// servol.attach(D6); 
}

void loop() 
{
 
 servol.write(15);
  delay(1000);
 servol.write(105);
 delay(1000);

  
}