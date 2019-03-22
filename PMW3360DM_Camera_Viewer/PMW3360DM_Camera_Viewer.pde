import processing.serial.*;

Serial myPort;
int[] val = null;
boolean sync = false;
int lf = 10;

void setup()
{
  size(360, 360);
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
      
      if(nums.length == 36*36)
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
}
