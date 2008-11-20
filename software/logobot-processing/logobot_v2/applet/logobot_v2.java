import processing.core.*; import processing.serial.*; import java.applet.*; import java.awt.*; import java.awt.image.*; import java.awt.event.*; import java.io.*; import java.net.*; import java.text.*; import java.util.*; import java.util.zip.*; import javax.sound.midi.*; import javax.sound.midi.spi.*; import javax.sound.sampled.*; import javax.sound.sampled.spi.*; import java.util.regex.*; import javax.xml.parsers.*; import javax.xml.transform.*; import javax.xml.transform.dom.*; import javax.xml.transform.sax.*; import javax.xml.transform.stream.*; import org.xml.sax.*; import org.xml.sax.ext.*; import org.xml.sax.helpers.*; public class logobot_v2 extends PApplet {

byte linear_parameter1=2;
byte linear_parameter2=0;
byte rotary_parameter1=1;
byte rotary_parameter2=0;
boolean pen = true;
int del=0;


byte WAITING =1;
byte MSG_START =2;
byte COMMAND =3;
byte FORWARD =4;
byte BACKWARD =5;
byte left =6;
byte right =7;
byte MSG_END =8;
byte ACK =9;
byte ACTION_COMPLETE =10;
byte PARAMETER1 =11;
byte PARAMETER2 =12;
byte REPORTING =13;
byte GET_X =14;
byte GET_Y =15;
byte GET_BATT =16;
byte DATA_IN =17;
byte LISTENING_FOR_THE_END =19;
byte WTF =20;
byte LISTENING_FOR_A_CHECKSUM= 21;
byte LISTENING =22;
byte GET_STATUS=27;
byte PEN_UP=28;
byte PEN_DOWN=29;

Serial myPort;

public void setup()
{
  size(200,200);
    println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 19200);
  
}

public void draw()
{
   background(0);
   while(myPort.available()>0)
     print((char)myPort.read());
   
}
public void check_data()
{
/*     while(myPort.available()>0)
    {
      int a=myPort.read();
      print(a+" ");
      if((a==8)||(a==10))
        println();
    }*/
   while(myPort.available()>0)
    {
      char a=(char)myPort.read();
      print(a);
    }
}
public void send_query(byte query)
{
  myPort.write(MSG_START);
  myPort.write(query);
  myPort.write(query);  //the checksum on queries is just the query again
  myPort.write(MSG_END); 
}

public void pen_up()
{
  myPort.write(MSG_START);
  myPort.write(PEN_UP);
  myPort.write(PEN_UP);  //the checksum on queries is just the query again
  myPort.write(MSG_END); 
}

public void pen_down()
{
  myPort.write(MSG_START);
    delay(del);
  myPort.write(PEN_DOWN);
    delay(del);
  myPort.write(PEN_DOWN);  //the checksum on queries is just the query again
    delay(del);
  myPort.write(MSG_END); 
}

public void send_command(byte command, byte highByte, byte lowByte)
{
    byte checksum=(byte)(((int)command+(int)highByte+(int)lowByte)%255);
    myPort.write(MSG_START);
    delay(del);
    myPort.write(command);
    delay(del);
    myPort.write(highByte);
    delay(del);
    myPort.write(lowByte);
    delay(del);
    myPort.write(checksum);
    delay(del);
    myPort.write(MSG_END);
    
}

public void keyPressed()
{
    if(keyCode==UP)
      send_command(FORWARD, linear_parameter1, linear_parameter2);
    if(keyCode==DOWN)
      send_command(BACKWARD, linear_parameter1, linear_parameter2);
    if(keyCode==LEFT)
      send_command(right, rotary_parameter1, rotary_parameter2);
    if(keyCode==RIGHT)
      send_command(left, rotary_parameter1, rotary_parameter2);
    if(key=='c')
    {
      for(int i=0;i<100;i++)
      {
         send_command(FORWARD,linear_parameter1,linear_parameter2);   
         delay(1000);
         send_command(left, rotary_parameter1, rotary_parameter2);
         delay(1000);
         if(pen)
         {
           pen_up();
         }
         else
           pen_down();
         pen=!pen;
         delay(750);
         check_data();

       }
    }
    if(key=='?')
    {
      send_query(GET_STATUS);
    }  
    
    if(key=='q')
    {
       exit();
    }
    if(key=='u')
      pen_up();
    if(key=='d')
      pen_down();
//    if(key==' ')
 //     send_stop();
}
    

  static public void main(String args[]) {     PApplet.main(new String[] { "logobot_v2" });  }}