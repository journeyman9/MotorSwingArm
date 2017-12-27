/*
unsigned long serialdata;
char serialCMD;
int inbyte;
unsigned long P;
unsigned long I;
unsigned long D;
unsigned long N;
unsigned long W;
unsigned long A;
int switchnum;


void setup()
{
  Serial.begin(9600);
  A = 90; //Define Angle as from 0 deg to 180deg. Assume nominal 90deg for vertical stick.
}

void loop()
{
 if (Serial.available())
 {
   Command();
 }
}

void Command()
{
  serialCMD = 0;
  switchnum = 0;
  getCommand();

  if (serialCMD == 'R' || serialCMD == 'r')
  {
    switchnum = 1;
  }
  else if (serialCMD == 'P' || serialCMD == 'p')
  {
    switchnum = 2;
  }
  else if (serialCMD == 'I' || serialCMD == 'i')
  {
    switchnum = 3;
  }
  else if (serialCMD == 'D' || serialCMD == 'd')
  {
    switchnum = 4;
  }
  else if (serialCMD == 'N' || serialCMD == 'n')
  {
    switchnum = 5;
  }
  else if (serialCMD == 'W' || serialCMD == 'w')
  {
    switchnum = 6;
  }
  else if (serialCMD == 'A' || serialCMD == 'a')
  {
    switchnum = 7;
  }
  else
  {
    switchnum = 8;
  }
  
switch(switchnum)
{
  case 1:
       {
     //Reset PID parameters
          if(serialdata != 'E')
          {
            P = I = D = N = W = 0;
            A = 90;
          }

         break;  
        }
   case 2:
        {
          //update Proportional variable
    
          getSerial();
          if(serialdata != 'E')
          {
            P = serialdata;
          }
          
          break; 
        }
   case 3:
        {
          //Update Integral variable
          getSerial();
           if(serialdata != 'E')
          {
            I = serialdata;
          }
          break;
         
        }
        
   case 4:
       {
         //Update Derivative variable
          getSerial();
          if(serialdata != 'E')
          {
            D = serialdata;
          }
          break;
          
       }
       
   case 5:
       {
       //Update Derivative filter constant
         getSerial();
          if(serialdata != 'E')
          {
            N = serialdata;
          }
         break; 
       }
   case 6:
       {
         //Update anti-Windup limit constant
         getSerial();
          if(serialdata != 'E')
          {
            W = serialdata;
          }
         break; 
        
       }
     case 7:
       {
       
          //Set target angle
         getSerial();       
          if(serialdata != 'E')
          {
            if (serialdata >= 0 && serialdata <= 180)
             {               
               A = serialdata;
             }
             else 
             {
               Serial.print("OUT OF RANGE");
             }
             
         break;
          }
       } 
     case 8:
          {
            Serial.println("Command ERROR!");
          break;
          }
}
       Serial.print("P: ");
       Serial.print(P); 
       Serial.print(" I: ");
       Serial.print(I);
       Serial.print(" D: ");
       Serial.print(D);
       Serial.print(" N: ");
       Serial.print(N);
       Serial.print(" W: ");
       Serial.print(W);
       Serial.print(" A: ");
       Serial.println(A);
}

long getCommand()
{
  serialCMD = 0;
  while (inbyte != '/')
  {
     inbyte = Serial.read();
     if (inbyte > 0 && inbyte != '/')
    {
     serialCMD = inbyte;  
    }
  }
  inbyte = 0;
  return serialCMD;
}

long getSerial()
{
  serialdata = 0;
  inbyte = 0;
  while (inbyte != '\r')
  {
    inbyte = Serial.read(); 
    
    if (inbyte >= 48 && inbyte <= 57 && inbyte != '\r')
    {
      serialdata = serialdata * 10 + inbyte - '0';
    }
    else if (inbyte > 0 && inbyte < 48 && inbyte != '\r' || inbyte > 57  )
    {
        Serial.print("Num Error: ");
        Serial.println(inbyte);
        serialdata = 'E';
        break;
    }
  }

  return serialdata;
}
*/
