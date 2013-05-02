
void sendPlotData(String seriesName, double data)
{
  Serial.print("{");
  Serial.print(seriesName);
  Serial.print(",T,");
  Serial.print(data);
  Serial.println("}");
}

void plot_stuff()
{
  sendPlotData("Temperature",Input);
  sendPlotData("Error",heatPID.GetError());
  sendPlotData("SetPoint",Setpoint);
  sendPlotData("Output",HotOutput);
}

void print_temp(int temp,int x, int y){
  lcd.setCursor(x,y);
    if (temp > 99)
    {
      lcd.print(temp);
    }
    else if (temp > 9)
    {
      lcd.print("0");
      lcd.print(temp);
    }
    else {
      lcd.print("00");
      lcd.print(temp);
    }
  }
  
void heating_print(){
  print_temp(Input,0,0);
  lcd.setCursor(3,0);
  lcd.print('/');
  print_temp(use_temps[counter],4,0);
  lcd.setCursor(0,1);
  //lcd.print(Stages[counter-1]);      
  lcd.setCursor(8,1);
  //lcd.print(millis());
  timekeeper();
}

void timekeeper(){
  lcd.setCursor(8,1);
   print_time(millis() - current_time); 
}

// argument is time in milliseconds
void print_time(unsigned long t_milli)
{
    char timer[20];
    int mins;
    int secs;
    int fractime;
    unsigned long inttime;

    inttime  = t_milli / 1000; // inttime is the total number of number of seconds
    fractime = t_milli % 1000; // fractimeis the number of thousandths of a second
    
    mins     = inttime / 60;
    inttime  = inttime % 60;

    // Now inttime is the number of seconds left after subtracting the number
    // in the number of minutes. In other words, it is the number of seconds.
    
    secs = inttime;

    // Don't bother to print days
    sprintf(timer, "%02d:%02d.%03d", mins, secs, fractime);
    lcd.print(timer);
}
