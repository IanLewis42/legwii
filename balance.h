//balancing task. Heavily based on code by Laurens Valk (robotsquare.com)
//http://robotsquare.com/2012/02/13/tutorial-segway-with-robotc/

int steering = 0;
int acceleration = 50;
int speed = 0;
bool starting_balancing_task = true;
int waits,tim,last_tim,delta;

//GLOBAL VARIABLE SETUP
float gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,mean_reading,gear_down_ratio,dt;

task balancing()
{
     gear_down_ratio = 1.0;
     dt = 0.012;
    // Customize PID constants. These variables are global, so you can optionally dynamically change them in your main task.
    gn_dth_dt = 0.23;
    gn_th = 25.00;
    gn_y = 0.0;//272.8;
    gn_dy_dt = 0.0;//24.6;
    //kp = 0.0336;
    //ki = 0.2688;
    //kd = 0.000504;

    kp = 0.025;//667;
    ki = 0.3;//0.30;
    kd = 0.001;//0.0005;


    ResetBlockTachoCount(OUT_AC);
    

    
    //SetSensorType(WMP, SENSOR_TYPE_NONE);
    //    Wait(100);
   SetSensorLowspeed(WMP);
    //MSDeenergize(WMP, I2C_ADDR_DEFAULT);
    //Wait(100);
    //MSEnergize(WMP, I2C_ADDR_DEFAULT);
    //Wait(100);
    
    ResetSensor(WMP);
    Wait(100);
    
    SetI2COptions(WMP, I2C_OPTION_FAST);
    
   initWMplus();
   Wait(500);     // Wait some time after init

   if (calibrateOffset())
      Error();
   else
      Success();

      //roll0 = 0;

  const float radius = your_wheel_diameter/1000;
  const float degtorad = PI/180;

  //SETUP VARIABLES FOR CALCULATIONS
  float u = 0;                    // Sensor Measurement (raw)
  float th = 0,//Theta            // Angle of robot (degree)
        dth_dt = 0;//dTheta/dt    // Angular velocity of robot (degree/sec)
  float e = 0,//Error             // Sum of four states to be kept zero: th, dth_dt, y, dy_dt.
        de_dt = 0,//dError/dt     // Change of above error
        _edt = 0,//Integral Error // Accumulated error in time
        e_prev = 0;//Previous Error/ Error found in previous loop cycle
  float pid = 0;                  // SUM OF PID CALCULATION
  float y = 0,//y                     // Measured Motor position (degrees)
        dy_dt = 0,//dy/dt             // Measured motor velocity (degrees/sec)
	      v = 0,//velocity          // Desired motor velocity (degrees/sec)
	      y_ref = 0;//reference pos // Desired motor position (degrees)
  int motorpower = 0,             // Power ultimately applied to motors
      last_steering = 0,          // Steering value in previous cycle
      straight = 0,               // Average motor position for synchronizing
      d_pwr = 0;                  // Change in power required for synchronizing
  #define  n_max  7            // Number of measurement used for floating motor speed average
  int n = 0,n_comp = 0,           // Intermediate variables needed to compute measured motor speed
  encoder[n_max];                 // Array containing last n_max motor positions

  //memset(&encoder[0],0,sizeof(encoder));

  int i,j=0;
  for (i=0 ; i<n_max ; i++)
      encoder[i] = 0;
  starting_balancing_task = false;// We're done configuring. Main task now resumes.

  unsigned long int time = CurrentTick();
  
  while(true)
  {
         readWMplus();
         u = scaled_roll;
         
         //while(CurrentTick() < (time + 2));
         //readWMplus();
         //u += scaled_roll;
         
         
    		//COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE					//gyro gives rate of change of angle
      	dth_dt = u - mean_reading;											//u/2 cos we read it twice and sum? *IPL
      	mean_reading = mean_reading*0.99 + (0.01*(dth_dt+mean_reading));		//leaky int added term is just u/2.....
      	th = th + dth_dt*dt;													//integrate to get angle

      //ADJUST REFERENCE POSITION ON SPEED AND ACCELERATION					//leave for now - this is for movement, not balancing
      if(v < speed*10){
      v = v + acceleration*10*dt;}
      else if(v > speed*10){
      v = v - acceleration*10*dt;}
      y_ref = y_ref + v*dt;
         
  	//COMPUTE MOTOR ENCODER POSITION AND SPEED
  	n++;																	//array of
  	if(n == n_max)															//loop through array, one inc per pass.
  		{n = 0;}

  	encoder[n] =  MotorBlockTachoCount(OUT_A) +  MotorBlockTachoCount(OUT_C) + y_ref;		//array of ints, store sum of 2 motors + 'desired'?? nMotorEncoder[] is position in degrees, stored in S16(I think...) *IPL
  																			//desired comes from movement, above.....
  																			//MotorBlockTachoCount may be NXC equivalent
  	n_comp = n+1;															//next array entry, wrapped
  	if(n_comp == n_max)
  		{n_comp = 0;}

  	y = encoder[n]*degtorad*radius/gear_down_ratio;							//distance moved from start aka current position? (but both encoders??)

  	dy_dt = (encoder[n] - encoder[n_comp])/(dt*(n_max-1)) * degtorad*radius/gear_down_ratio;	//distance moved in last 7 passes / 7*dt so.... speed!

  	//COMPUTE COMBINED ERROR AND PID VALUES
  	e = gn_th * th + gn_dth_dt * dth_dt + gn_y * y + gn_dy_dt * dy_dt;		//all these (angle, rate of cahnge, position, rate of change) should all be zero.
  																			//so value is error, apply gains.
  	de_dt = (e - e_prev)/dt;												//differential of error

  	_edt = _edt + e*dt;														//integral of error

  	e_prev = e;
  	
  	//float p =  kp*e;
  	//float in =  ki*_edt;
  	//float d =   kd*de_dt;
  	
  	//float PID = (p + in + d)/radius*gear_down_ratio;

  	pid = (kp*e + ki*_edt + kd*de_dt)/radius*gear_down_ratio;				//PID calc. /radius to get back from distance to angle....?...

  	//ADJUST MOTOR SPEED TO STEERING AND SYNCHING
    if(steering == 0)
    {
        if(last_steering != 0)
        {
	        straight =  MotorBlockTachoCount(OUT_C) -  MotorBlockTachoCount(OUT_A);
	    }
		d_pwr = ( MotorBlockTachoCount(OUT_C) -  MotorBlockTachoCount(OUT_A) - straight)/(radius*10/gear_down_ratio);
	}
    else
    	{d_pwr = steering/(radius*10/gear_down_ratio);}

    last_steering = steering;
    
    //d_pwr = 0;

  	//CONTROL MOTOR POWER AND STEERING
  	motorpower = 	-1*pid;
  	if (motorpower >  100) motorpower =  100;
  	if (motorpower < -100) motorpower = -100;
  	
    OnFwd(OUT_A, motorpower + d_pwr);
    OnFwd(OUT_C, motorpower - d_pwr);
    
    //OnFwdReg(OUT_A, motorpower + d_pwr, OUT_REGMODE_SPEED);
    //OnFwdReg(OUT_C, motorpower - d_pwr, OUT_REGMODE_SPEED);
    
    j++;
    if (j == 50)
    {
      j=0;
      ClearScreen();
      tim = CurrentTick();
      delta = tim - last_tim;
      last_tim = tim;

      //TextOut(5, LCD_LINE1, "first: ");
      //NumOut(55, LCD_LINE1, first_roll);
      TextOut(5, LCD_LINE1, "roll_off: ");
      NumOut(55, LCD_LINE1, roll_offset);
      TextOut(5, LCD_LINE2, "roll: ");
      NumOut(55, LCD_LINE2, roll);
      //TextOut(5, LCD_LINE4, "scaled: ");
      //NumOut(55, LCD_LINE4, scaled_roll);
      //TextOut(5, LCD_LINE5, "mean: ");
      //NumOut(55, LCD_LINE5, mean_reading);
      TextOut(5, LCD_LINE3, "dth/dt: ");
      NumOut(55, LCD_LINE3, dth_dt);
      TextOut(5, LCD_LINE4, "theta: ");
      NumOut(55, LCD_LINE4, th);
      //TextOut(5, LCD_LINE5, "P: ");
      //NumOut(55, LCD_LINE5, p);
      //TextOut(5, LCD_LINE6, "I: ");
      //NumOut(55, LCD_LINE6, in);
      //TextOut(5, LCD_LINE7, "D: ");
      //NumOut(55, LCD_LINE7, d);

      //TextOut(5, LCD_LINE8, "PID: ");
      //NumOut(55, LCD_LINE8, PID);
      TextOut(5, LCD_LINE5, "waits: ");
      NumOut(55, LCD_LINE5, waits);
      
      TextOut(5, LCD_LINE6, "delta: ");
      NumOut(55, LCD_LINE6, delta);
      TextOut(5, LCD_LINE8, "Motor: ");
      NumOut(55, LCD_LINE8, motorpower);
      waits = 0;
    }

    //ERROR CHECKING OR SHUTDOWN
    if(abs(th)>60 || abs(motorpower) > 2000){
      StopAllTasks();}

    //WAIT THEN REPEAT

  	while(CurrentTick() < (time + dt*1000))
    {
  	  Wait(1);
      waits++;
      }
  	time = CurrentTick();
  }
  

}
