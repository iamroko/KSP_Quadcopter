// Parameters for Quadcopter flight on Kerbin. Kerbal's top scientists aren't sure what any of them mean, but they do break things if you set them wrong. 
// Note, not all parameters are nessecarily used. Some are wishful thinking.
// These generally work well on Kerbin, but have a lot of room for improvement. 

declare global blade_pitch is 9.

// More torque provides for a snapier response, especially in thicker atmospheres. However, there is a power tradeoff. 
declare global torque_limit is 100.

// PID parameters are defined below. The *_rpm_limit fields in pitch roll and yaw are used to define how much of an RPM change is allowed in the output of the PID.
// This is important to avoid saturating the rotors, as they're somewhat limited in top RPM. This is especially a concern in thinner atmospeheres.

// Pitch Control
declare global pitch_kp is 1.3.
declare global pitch_ki is 0.5.
declare global pitch_kd is 1.0.

declare global pitch_rpm_limit is 45.

// Roll Control
// Nominall identical to Pitch gains. 
declare global roll_kp is 1.3.//1.3.
declare global roll_ki is 0.5.
declare global roll_kd is 1.0.//.1.0.

declare global roll_rpm_limit is 45.

declare global pitch_limit is 45.


// Yaw / Heading Control
declare global yaw_kp is 1.0.
declare global yaw_ki is 0.4.
declare global yaw_kd is 0.6.

declare global yaw_rpm_limit is 20.

declare global roll_limit is 45.

// Velocity Control
// Forward/Lateral can be nominally similar gains. 
declare global forward_velocity_kp is 4.
declare global forward_velocity_ki is 1. //1
declare global forward_velocity_kd is 2.5.// 2.5.

declare global lateral_velocity_kp is 4.
declare global lateral_velocity_ki is 1. //1
declare global lateral_velocity_kd is 2.5.// 2.5.

declare global forward_velocity_limit is 30.
declare global lateral_velocity_limit is 30.
declare global vertical_velocity_limit is 10.

// Altitude Control
// Two sets because gravity suuuccckkkss. 
declare global altitude_up_kp is 15.//15.
declare global altitude_up_ki is 1.5.//1.5.//.
declare global altitude_up_kd is 15.//10.

declare global altitude_down_kp is 5.//5.
declare global altitude_down_ki is 0.05.//0.01.//.
declare global altitude_down_kd is 35.//35.

declare global max_climb_rate is 15.
declare global max_descent_rate is 5.

// Positional Control
// declare global x_kp is 0.5.//1.
// declare global x_ki is 0.25.
// declare global x_kd is 1.//2.

// declare global y_kp is 0.5.//1.
// declare global y_ki is 0.25.
// declare global y_kd is 1.//2.

declare global x_kp is 2.//1.
declare global x_ki is 0.75.
declare global x_kd is 4.//2.

declare global x_speed_limit to 2.

declare global y_kp is 2.//1.
declare global y_ki is 0.75.
declare global y_kd is 4.//2.

declare global y_speed_limit to 2.

// Comms Manager Settings. Set to False if not using the Comms manager to actively control directional antennas. If True, ensure the list of satellites is accurate.
declare global use_comms_manager is True.
declare global commsat_list is LIST("CommSat I Block II", "CommSat II Block II", "CommSat III Block II").

// How high should I be allowed to go.
declare global ceiling is 20000.

// If going into emergency landing how far away can a potential landing spot be. 
declare global emergency_search_range is 500.

// RPM required to almost, but not quite, take off. 
declare global baseline_thrust is 85.