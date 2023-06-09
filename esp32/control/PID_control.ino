//PID CODE GENRAL STRUCTURE

double sum_e = 0, e_n-1 =0; //sum of errors and previous error initialisation
double control_max; //absolute maximum value of actuators

double simple_PID_calc(Ts, set_point, sensor_point, Kp, Ki, Kd){
	//variable finding
	double e = set_point - sensor_point; //current error for proportional term [1]
	sum_e += e; //cumulative error for sum term [2]
	double differential_e = e - e_n-1; //delta error for differential term [3]
	
	//output setting
	double control_input = Kp*e + Ki*T*sum_e + Kd*(1/T)*differential_e; //control law
	if(abs(control_input) >= control_max) control_input = control_max * abs(control_input)/control_input; //in case motor limit reached
	
	//incrementing
	e_n-1 = e;
	
	//return
	return control_input;
}

void setup(){
	//Initialise variables control_max, and Kp Ki Kd, and also control inputs
}

void loop(){
	//Find set_point and sensor_point
	
	control = simple_PID_calc()//relevant variables go here
	
	//send control to actuators
}
