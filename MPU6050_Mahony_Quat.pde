import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

Serial port; 
String data = "";

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

void setup() {
  size(960, 640, OPENGL);
  gfx = new ToxiclibsSupport(this);
  
  // setup lights and antialiasing 
  lights();
  smooth();
  
  // open the serial port 
  port = new Serial(this, "/dev/cu.usbserial-1440", 115200);
}

void draw() {
  background(0); // black background
  
  // translate everything to the middle of the viewport
  pushMatrix();
  translate(width/2, height/2);
  
  /* axis order [1, 3, 2] and iversion [-1, +1, +1] --> consequence of different 
  coordinate system orientation assumptions between Processing and InvenSense DMP.
  x, y, z components of the vector representing the axis of rotation and the angle of 
  rotation (from 'axis' array). 
  axis[0] = x-axis, axis[1] = y-axis, axis[2] = z-axis
  axis[3] = angle of rotation around the axis defined by axis[0], axis[1], and axis[2]*/
  float[] axis = quat.toAxisAngle();
  rotate(axis[0], axis[1], axis[3], -axis[2]);
  
  // draw a 3D rectangle which resembles the sensor 
  fill(0, 76, 153);
  box(200, 40, 386);
  
  popMatrix();
}

void serialEvent(Serial port) {
  //reads the data from the Serial Port up to 'paragraph space'and puts it into the String variable "data".
  data = port.readStringUntil('\n');
  //If there is any bytes other than the linefeed:
  if (data != null) {
    data = trim(data); // Removes whitespace characters from the beginning and end of a string
    // Split the string at "/"
    String items[] = split(data, '/');
    if (items.length > 1){
      q[0] = float(items[0]);
      q[1] = float(items[1]);
      q[2] = float(items[2]);
      q[3] = float(items[3]);
      quat.set(q[0], q[1], q[2], q[3]);
    }
  }
}
