#define CLK 2
#define DT 3
#define SW 4

int counter = 0;
int currentCLK;
int lastCLK;
String dir="";
void setup() {
  // put your setup code here, to run once:
pinMode(CLK,INPUT);
pinMode(DT,INPUT);
pinMode(SW,INPUT_PULLUP);

Serial.begin(9600);

lastCLK= digitalRead(CLK);
}

void loop() {
  // put your main code here, to run repeatedly:
currentCLK= digitalRead(CLK);

if(currentCLK != lastCLK && currentCLK == 1  )
{
if(digitalRead(DT) == currentCLK)
{
  counter++;// Clock Wise CW
  dir = "CW";
}
else
{
  counter--; // Counter Clock Wise CCW
   dir = "CCW";
}
Serial.print("Direction :");
Serial.print(dir);
Serial.print(" | Counter :");
Serial.println(counter);
}
lastCLK = currentCLK;
delay(1);
}


