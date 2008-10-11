import processing.serial.*;
Serial port;  
int pinStatus = 0;

void setup(){
port = new Serial(this, Serial.list()[0], 9600);
}

void draw() {
}

void keyPressed(){
  if (key == CODED) {
    if (keyCode == UP){
      port.write(1);
      println("UP");
    }

    else if (keyCode == DOWN){
      port.write(0);
      println("DOWN");
    }
    else if (keyCode == RIGHT){
      port.write(2);
      println("RIGHT");
    }
    else if (keyCode == LEFT){
      port.write(3);
      println("LEFT");
    }
  }
  else {
    if (key == 'p' || key == 'p'){
      port.write(4);
      println("PINUP");
    }
    else if (key == 'd' || key == 'd'){
      port.write(5);
      println("PINDOWN");
    }
    else if (key == 'x'){
      port.write(6);
    } 
  }
}

