double read_temp()
{
  double ins;
  double vcc;
  double adcvalue;
  double interm;
  for (int c = 0; c <128; c++)
  {  
    vcc = (double)readVcc();
    adcvalue = (double)analogRead(tmpPin);
    interm = (adcvalue/5115)*vcc;
    ins += interm/128;
  }
  return ins;
}

#define NUM_READS 100
double testread(int sensorpin){
   // read multiple values and sort them to take the mode
   int sortedValues[NUM_READS];
   for(int i=0;i<NUM_READS;i++){
     int value = analogRead(sensorpin);
     int j;
     if(value<sortedValues[0] || i==0){
        j=0; //insert at first position
     }
     else{
       for(j=1;j<i;j++){
          if(sortedValues[j-1]<=value && sortedValues[j]>=value){
            // j is insert position
            break;
          }
       }
     }
     for(int k=i;k>j;k--){
       // move all values higher than current reading up one position
       sortedValues[k]=sortedValues[k-1];
     }
     sortedValues[j]=value; //insert current reading
   }
   //return scaled mode of 10 values
   float returnval = 0;
   for(int i=NUM_READS/2-5;i<(NUM_READS/2+5);i++){
     returnval +=sortedValues[i];
   }
   returnval = returnval/10;
   double vcc = (double)readVcc();
   double out = ((double)returnval/5115)*vcc;
   return out;
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

double read_input()
{
  double vcc = (double)readVcc();
  double adcvalue = (double)analogRead(tmpPin);
  double out = (adcvalue/5115)*vcc;
  return out;
}

