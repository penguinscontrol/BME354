void changeAutoTune()
{
 if(!tuning)
  {
    HotOutput=aTuneStartValue; //Set the HotOutput to the desired starting frequency.
    aTune.SetNoiseBand(aTuneNoise); //What counts as noise?
    aTune.SetOutputStep(aTuneStep); 
    aTune.SetLookbackSec((int)aTuneLookBack);
    aTune.SetControlType(1);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = heatPID.GetMode();
  else
    heatPID.SetMode(ATuneModeRemember);
}

void TuneGains()
{
  now = millis();
  Input = read_input(Input);
  if(tuning)
  {
    byte val = (aTune.Runtime());
    sendPlotData("OUT",HotOutput);
    sendPlotData("Temperature",Input);
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      
      EEPROM.write(0, 10*kp);
      EEPROM.write(1, 10*ki);
      EEPROM.write(2, 10*kd);
      
      heatPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else {
    heatPID.Compute();
    select++;
  }
  
  fake_PWM(heatPin,HotOutput);
}

