
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
  print_temp(input_temps[counter],4,0);
  lcd.setCursor(0,1);
  //lcd.print(Stages[counter]);      
  lcd.setCursor(8,1);
  lcd.print(millis());
}
