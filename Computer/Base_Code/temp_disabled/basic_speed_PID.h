#ifndef basic_speed_PID_H
#define basic_speed_PID_H

#include<time.h>

// this is written assuming PID control of some DC motor
// the target value is target_speed (rpm)
// the measured value is measured_speed (rpm)
// the PID output value is the PWM val to be fed to the DC motor (not clipped).
//
// The class can control a process involving physical quantities other than speed (temperature, pressure, etc).

class basic_speed_PID
{
	protected:

	// gain values, used for PID contrl:
	// obtained by scaling the ref_values depending on the frequency of control operations
	double kp{0};                  // * (P)roportional Tuning Parameter
	double ki{0};                  // * (I)ntegral Tuning Parameter
	double kd{0};                  // * (D)erivative Tuning Parameter

								// reference values, set by the user (default provided)
	double ref_kp{0};                  // * (P)roportional Tuning Parameter
	double ref_ki{0};                  // * (I)ntegral Tuning Parameter
	double ref_kd{0};                  // * (D)erivative Tuning Parameter

									// reference interval, used to scale the reference control paramters
	double ref_contr_inter_time_ms{0};

	// range of the signal driving the speed, e/g/ PWM
	double PIDoutMin{0}, PIDoutMax{0};

	// used for PID calculation
	unsigned long last_control_ms{0};
	double last_error{0};
	double cumulative_error{0};

	//
	double const REF_CONTR_INTER_MS = 100;

	bool enabled{0};

	//
	bool echopidcontrol{0};


	void set_defaultvals()
	{
		// set some default vals
		ref_kp = 0.7;
		ref_ki = 0.45;
		ref_kd = 0.5;
		ref_contr_inter_time_ms = REF_CONTR_INTER_MS;
		PIDoutMin = 0.0;
		PIDoutMax = 355.0;

		// initialize params
		last_control_ms = clock();
		last_error = 0.0;
		cumulative_error = 0.0;

		//
		echopidcontrol = false;
		//
		enabled = true;
	}



public:
	basic_speed_PID() { set_defaultvals(); }
	basic_speed_PID(double inp_ref_kp, double inp_ref_ki, double inp_ref_kd, double inp_PIDoutMin, double inp_PIDoutMax, int inp_ref_contr_inter_time_ms = 1000)
	{
		set_defaultvals();
		set_gainvals(inp_ref_kp, inp_ref_ki, inp_ref_kd);
		set_ref_control_interval_ms(inp_ref_contr_inter_time_ms);
		set_bounds(inp_PIDoutMin, inp_PIDoutMax);
	}

	void set_gainvals(double inp_ref_kp, double inp_ref_ki, double inp_ref_kd)
	{
		ref_kp = inp_ref_kp;
		ref_ki = inp_ref_ki;
		ref_kd = inp_ref_kd;
	}
	void set_ref_control_interval_ms(int inp_ref_contr_inter_time_ms)
	{
		ref_contr_inter_time_ms = inp_ref_contr_inter_time_ms;
	}
	void set_bounds(double inp_PIDoutMin, double inp_PIDoutMax)
	{
		PIDoutMin = inp_PIDoutMin;
		PIDoutMax = inp_PIDoutMax;
	}

	// this is the main funciton to call
	double ComputePID_output(double target_speed, double curr_speed)
	{
		unsigned long now = clock();
		double output = 0.0;

		unsigned long tempcontrol_interval = (now - last_control_ms);
		int contr_inter_time_ms = (int)tempcontrol_interval;


		// working variables
		double error = target_speed - curr_speed;
		double contr_inter_time_ratio = ref_contr_inter_time_ms / contr_inter_time_ms;
		double error_diff = error - last_error;
		double error_derivative = error_diff / ((double)contr_inter_time_ms);

		//cumulative_error+=error;
		//kp = ref_kp * contr_inter_time_ratio;
		//ki = ref_ki * contr_inter_time_ratio;
		//kd = ref_kd * contr_inter_time_ratio;

		// account for difference in sampling time
		cumulative_error += (error*contr_inter_time_ratio);
		kp = ref_kp;
		ki = ref_ki;
		kd = ref_kd;



		//Compute PID Output
		output = kp*error + ki*cumulative_error + kd*error_derivative;

		// keep within bounds
		if (output > PIDoutMax)
			output = PIDoutMax;
		else
			if (output < PIDoutMin)
				output = PIDoutMin;

		//print to screen
		// store vals for the next iteration
		last_control_ms = now;
		last_error = error;
		//
		return output;
	}

	double GetKp() { return kp; }
	double GetKi() { return ki; }
	double GetKd() { return kd; }

	void set_echopidcontrol(bool inp_echo) { echopidcontrol = inp_echo; }
	bool get_echopidcontrol() { return echopidcontrol; }
	void reset_pidcontrol()
	{
		// set params as when initialized
		last_control_ms = clock();
		last_error = 0.0;
		cumulative_error = 0.0;
	}
};

#endif
