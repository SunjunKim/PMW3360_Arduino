import processing.serial.*;

Serial myPort;
int[] val = null;
boolean sync = false;
int lf = 10;
int lastSQ = 0;

void setup()
{
  size(360, 380);
  String[] sList = Serial.list();
  String portName = sList[sList.length - 1];
  myPort = new Serial(this, portName, 9600);
}

void draw()
{
  if(myPort.available() > 0)
  {
    String s = myPort.readStringUntil(lf);
    
    if(s != null)
    {
      int[] nums = int(split(trim(s), ' '));
      
      if(nums.length == 36*36+1)
      {
        val = nums;
      }
    }
  }
  
  if(val == null)
  {
    background(0);
    return;
  }
  
  background(0);
 
  noStroke();
  for(int i=0;i<36;i++)
  {
    for(int j=0;j<36;j++)
    {
      int pixel = val[i*36+j];
      fill(pixel);
      rect(i*10, (35-j)*10, 10, 10);
    }
  }
  
  int squal = val[36*36];
  if(squal != 0)
    lastSQ = squal;
  fill(255);
  text("SQUAL = "+lastSQ, 10, 375);
}

void keyPressed()
{
  if(key == 's')
    save("frame.png");
}
