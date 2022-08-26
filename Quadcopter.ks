// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along with this program. If not, see 
// <https://www.gnu.org/licenses/>. 




clearScreen.

run lib_navball.
run lib_comm_manager.
run lib_slope.

// When using remotetech, sometimes it will be a long time between issueing the command and starting flight. However, flight does not respond well 
// to timewarp. Check if we're in timewarp, and cancel before proceeding .
if warp <> 0 {
    print("Cancelling Timewarp").
    set warp to 0.
    wait until ship:unpacked. 
    wait 1.
}

// Flight controls are heavily dependant upon atmospheric pressure and gravity. Each planet requires it's own tuning.
// Determine which body the vessel is on, and get appropriate input parameters. 
print("Loading flight control parameters for " + SHIP:BODY:NAME).
if SHIP:BODY:NAME = "Kerbin" {
    run "Parameters/Quadcopter_Kerbin.ks".
} else if SHIP:BODY:NAME = "Duna" {
    run "Parameters/Quadcopter_Duna.ks".
} else if SHIP:BODY:NAME = "Eve" {
    // To Do
} else if SHIP:BODY:NAME = "Laythe" {
    // To Do
} else if SHIP:BODY:NAME = "Jool" {
    // To Do, Optimism!
} else {
    print("ERROR! No header defined for this body. Program will crash shortly.").
}


set default_mission to QUEUE(
        lexicon("State", "takeoff",
                "Altitude", 50),
        lexicon("State", "fly", 
                "Destination", list("navigation_target"),
                "Altitude", 50,
                "MaxSpeed", 30),
        lexicon("State", "Hover",
                "Destination", list("navigation_target"),
                "Altitude", 50,
                "Time", 0),
        lexicon("State", "land"),
        lexicon("State", "exit")
). 

set start_position to ship:geoposition.

// Setup the vessel and it's control parameters. 
set rotors to LIST("leftFrontRotor", "leftRearRotor", "rightRearRotor", "rightFrontRotor").
set blades to SHIP:PARTSNAMED("mediumFanBlade").

// Set low battery threshold. Should be enough to rapidly come to a stop (Recovery) and fly to safe landing zone.
// check to see if it has bene defined in the mission script or not. 
if defined low_battery_threshold {} else set low_battery_threshold to 100.

if defined min_solar_exposure {} else set min_solar_exposure to 0.5.

SET heading_pid to PIDLOOP(yaw_kp,yaw_ki,yaw_kd, -yaw_rpm_limit,yaw_rpm_limit).
SET altitude_pid to PIDLOOP(altitude_up_kp,altitude_up_ki,altitude_up_kd, 50-baseline_thrust, 410-baseline_thrust).

SET roll_pid to PIDLOOP(roll_kp,roll_ki,roll_kd, -roll_rpm_limit, roll_rpm_limit).
SET pitch_pid to PIDLOOP(pitch_kp,pitch_ki,pitch_kd, -pitch_rpm_limit, pitch_rpm_limit).

SET forward_velocity_pid to PIDLOOP(forward_velocity_kp,forward_velocity_ki,forward_velocity_kd,-pitch_limit,pitch_limit).
SET lateral_velocity_pid to PIDLOOP(lateral_velocity_kp,lateral_velocity_ki,lateral_velocity_kd,-roll_limit,roll_limit).

SET x_pid to PIDLOOP(x_kp, x_ki, x_kd,-x_speed_limit,x_speed_limit).
SET y_pid to PIDLOOP(y_kp, y_ki, y_kd,-y_speed_limit,y_speed_limit).

function set_rpm {
    parameter name.
    parameter rpm.

    set motor to quadcopter:PARTSDUBBED(name)[0].
    set mod to motor:getmodule("ModuleRoboticServoRotor").
    mod:SETFIELD("RPM Limit", round(rpm)).
}

function set_torque {
    parameter name.
    parameter torque.

    set motor to quadcopter:PARTSDUBBED(name)[0].
    set mod to motor:getmodule("ModuleRoboticServoRotor").
    mod:SETFIELD("Torque Limit(%)", torque).
}

// Get the great-circle bearing between two geopositions. This enables better navigation over longer distances on spherical bodies. 
function get_bearing {
    parameter A.
    parameter B.

    local X is cos(B:LAT)*sin(B:LNG - A:LNG).
    local Y is ( cos(A:LAT)*sin(B:LAT) ) - (sin(A:LAT)*cos(B:LAT)*COS(B:LNG -A:LNG)).
    local bearing is arctan2(X, Y).

    if bearing < 0 {
        set bearing to bearing + 360.
    }
 //   print bearing.
    return bearing.
}

// Get great-circle distance between two geopositions, in meters. 
function get_distance {
    parameter A.
    parameter B.
    local distance is vang(A:position-body:position,B:position-body:position)*Constant:DegToRad*body:radius.

 //   print distance.
    return distance.
}

// Get the ship's velocity in X (east/Longitude) and Y (north/latitude)
function get_velocity {

    local sinYaw is sin(ship:up:yaw).
    local cosYaw is cos(ship:up:yaw).
    local sinPitch is sin(ship:up:pitch).
    local cosPitch is cos(ship:up:pitch).

    local unitVectorEast is V(-cosYaw, 0, sinYaw).
    local unitVectorNorth is V(sinYaw*sinPitch, cosPitch, cosYaw*sinPitch).
    
    local shipVelocitySurface is ship:velocity:surface.
    local speedEast is vdot(shipVelocitySurface, unitVectorEast).
    local speedNorth is vdot(shipVelocitySurface, unitVectorNorth).


    // Vector arrows to help debug this code.

    // SET eastArrow TO VECDRAWARGS(
    // v(0,0,0),
    // unitVectorEast,
    // RED,
    // "East",
    // 1,
    // TRUE
    // ).

    // SET northArrow TO VECDRAWARGS(
    // v(0,0,0),
    // unitVectorNorth,
    // BLUE,
    // "North",
    // 1,
    // TRUE
    // ).


    return list(speedEast, speedNorth).

}

// Position error between two lat/long pairs, returns an error in meters for x-axis (longitude error) and y-axis (latitude error)
// Used for precsion positioning (i.e. for precise hover, docking, etc.)
function position_error {
    parameter A.
    parameter B.

    local x_err is (A:LNG - B:LNG)*Constant:DegToRad*body:radius.
    local y_err is (A:LAT - B:LAT)*Constant:DegToRad*body:radius.

    return (list(x_err, y_err)).
}


// Rotate the vector X1, Y1 by theta. 
function vectorRotate {
    parameter X1.
    parameter Y1.
    parameter theta.

    // x2=cosβx1−sinβy1
    // y2=sinβx1+cosβy1

    local X2 is cos(theta)*X1 - sin(theta)*Y1.
    local Y2 is sin(theta)*X1 + cos(theta)*Y1.

    return list(X2, Y2).

}

// Clamp A value to greater than min, lower than max. 
function clamp {
    parameter A.
    parameter MIN.
    parameter MAX.

    return MAX(MIN(A, MAX), MIN).
}

function navigation_loop {
    set forward_velocity_pid:setpoint to speed_target.
    return -forward_velocity_pid:update(TIME:SECONDS, ship:groundspeed).
}


// The Stability Loop is used to maintain the quadcopter's stability. Nominall this will keep it level and will ignore velocities. Earlier loops 
// generate the inputs for this loop to enable controlled flight. 
// Altitude has been removed from this loop's control, as controlling altitude directly caused over-speed conditions resulting in loss of stability 
// (due to saturating the rotor RPM values). Instead, the altitude PID now controls the craft's vertical velocity, allowing controlled ascent/descent rates. 
function stability_loop {
    parameter pitch_setpoint.
    parameter roll_setpoint.
    parameter heading_setpoint.
    parameter altitude_setpoint.

    // Fix heading_setpoint if passed negative. 
    if heading_setpoint < 0 {
      set heading_setpoint to heading_setpoint + 360.
    }

    set current_compass to compass_for(ship).

    set steering_error to current_compass - heading_setpoint.

    if steering_error > 180 {
        set steering_error to steering_error - 360.
    } else if steering_error < -180 {
        set steering_error to steering_error + 360.
    }
     
    // Adjust PID parameters for thrust when going up vs going down. (Gravity compensation!)

    local altitude_comp_setpoint is root_control(altitude_setpoint, alt:radar, 2).

    //print(alt:radar - altitude_comp_setpoint).

    if altitude_comp_setpoint < alt:radar - 2 {
        set altitude_pid:kp to altitude_down_kp.
        set altitude_pid:ki to altitude_down_ki.
        set altitude_pid:kd to altitude_down_kd.
    } else {
        set altitude_pid:kp to altitude_up_kp.
        set altitude_pid:ki to altitude_up_ki.
        set altitude_pid:kd to altitude_up_kd.
    }

    set altitude_pid:setpoint to altitude_comp_setpoint.
    // set heading_pid:setpoint to heading_setpoint.
    set heading_pid:setpoint to 0.
    set pitch_pid:setpoint to pitch_setpoint.
    set roll_pid:setpoint to roll_setpoint.

    set roll to roll_pid:UPDATE(TIME:SECONDS, roll_for(ship)).
    set pitch to pitch_pid:UPDATE(TIME:SECONDS, pitch_for(ship)).
    // set yaw to heading_pid:UPDATE(TIME:SECONDS, current_compass).
    set yaw to heading_pid:UPDATE(TIME:SECONDS, steering_error).
    set thrust to altitude_pid:UPDATE(TIME:SECONDS, ALT:RADAR) + baseline_thrust.
    //set thrust to thrust_setpoint + baseline_thrust.

    set_rpm("leftFrontRotor", thrust + yaw + pitch + roll).
    set_rpm("rightFrontRotor", thrust - yaw + pitch - roll).
    set_rpm("rightRearRotor", thrust + yaw - pitch - roll).
    set_rpm("leftRearRotor", thrust - yaw - pitch + roll).

    //print("Thrust: " + thrust).

}


// The Velocity loop generates the pitch roll and yaw settings to maintain desired velocity. Doing it this not only enables fine speed control, 
// but also prevents over-speed conditions, which can cause instability if trying to control pitch/roll/yaw directly. (This is due to some KSP limitations
// making it easy to saturate the rotor RPM limits, thus losing control authority)
function velocity_loop {

    local parameter forward_velocity_setpoint.
    local parameter lateral_velocity_setpoint.
    //local parameter vertical_velocity_setpoint.

    set forward_velocity_pid:setpoint to forward_velocity_setpoint.
    set lateral_velocity_pid:setpoint to lateral_velocity_setpoint.
    //set vertical_velocity_pid:setpoint to vertical_velocity_setpoint.
    
    local v1 is get_velocity.
    set v1 to vectorRotate(v1[1],v1[0],-compass_for(ship)).

    local forward_velocity is v1[0].
    local lateral_velocity is v1[1].
    //local vertical_velocity is SHIP:VERTICALSPEED.

    //print(forward_velocity).

    set roll_target to lateral_velocity_pid:update(TIME:SECONDS, lateral_velocity).
    set pitch_target to -forward_velocity_pid:update(TIME:SECONDS, forward_velocity).
    
}


// The altitude loop controls the quadcopter's altitude through velocity. Input the target altitude and current altitude.
// Inputting the current_altitude manually enables passing altitude reference to sea level, ground, or any other arbitrary reference. 
function altitude_loop {
    parameter altitude_setpoint.
    //parameter current_altitude.

    //set altitude_pid:setpoint to altitude_setpoint.

    //set vertical_velocity_target to altitude_pid:update(TIME:SECONDS, current_altitude).

    local altitude_setpoint is 0.

    return altitude_setpoint.

}

function root_control {
    // Idea taken from Arducopter, a way to tame the PID response by pre-processing the proportional error term. 
    local parameter setpoint.
    local parameter current_value.
    local parameter gain.
    if setpoint-current_value <= 0 {
        if setpoint-current_value >= -2 {
            return current_value - gain*0.5*abs(current_value - setpoint).
        } else {
            return current_value - gain*sqrt(abs(setpoint-current_value) - 1).
        }
    } if setpoint-current_value <= 2 {
        return current_value + gain*0.5*abs(setpoint - current_value).
    } else {
        return current_value + gain*sqrt(abs(setpoint-current_value) - 1).
    }
}

function keep_position { 
    local parameter position.
    
    local new_err is position_error(position, SHIP:GEOPOSITION).
        
    local x_err is new_err[0].
    local y_err is new_err[1].

    set x_pid:setpoint to 0.
    set y_pid:setpoint to 0.

    local x_correction is x_pid:update(TIME:SECONDS, x_err).
    local y_correction is y_pid:update(TIME:SECONDS, y_err).

    set corrections to vectorRotate(x_correction, y_correction, compass_for(ship)).

    set lateral_velocity_target to -corrections[0].
    set forward_velocity_target to -corrections[1].
}

function waypoint_exists{
  PARAMETER a.
  for item in AllWaypoints() {
    IF item:NAME = a {
      RETURN TRUE.
    }
  }
  RETURN FALSE.
}

function parse_destination {
    // Parse destination command from mission manager. Currently implemented destinations:
    // Passed as a list. Index [0] is always the type of destination.
    // Navigation Target: list("navigation_target")
    // Waypoint: list("waypoint", "waypoint_name")
    // Docking target: list("dock", "vessel_name", "docking_port_tag")
    // Potentially in the future this can be converted to a lexicon for readability?

    local parameter input.

    if input[0] = "navigation_target" {
        for location in allwaypoints() {
            if location:ISSELECTED{
                print("Waypoint is " + location:GEOPOSITION).
                return waypoint(location:name):GEOPOSITION.
            } 
        }
    } else if input[0] = "waypoint" {
        // Code to set destination to a waypoint. 
        print("Waypoint is " + input[1]).
        //print(allwaypoints()).
        if waypoint_exists(input[1]) {
            return waypoint(input[1]):geoposition.
        } else {
            return ship:geoPosition.
        }

    } else if input[0] = "dock" {
        // Code to set destnation to the docking port.
        // Note, if the vessel is too far away (i.e. not unpacked), can't target docking port, so will target the general vessel itself. 
        // This can then be corrected with a "Hover" mode for find position adjustment. 

        if vessel(input[1]):unpacked {
            set target_dock to vessel(input[1]):partstagged(input[2])[0].
            return body:geopositionof(target_dock:nodeposition).
            //set dockAltitude to body:altitudeof(dock:nodeposition).
        } else {
            return vessel(input[1]):geoposition.
        }

    } else if input[0] = "geoposition" {
        // If entering a raw geoposition, don't need to process anything. Just return it. 
        return input[1].   
    } else if input[0] = "return to start" {
        return start_position.
    } else {
        // Something is wrong (i.e. wrong destination format), just return ship geoposition to prevent crashing. 
        return ship:geoPosition.
    }

    // Catch-all incase nothing selected, just return ship's position. 
    return ship:geoposition.
}


function parse_altitude {
    local parameter altitude_command.

    if altitude_command:istype("Scalar") {
    // Altitude command is just raw number. This indicated desired height above AGL, and terrain hugging. 

    } else if altitude_command:istype("List") {
    // Altitude command is a list. This will take the format of: list(altitude, reference)
    // where reference can by any of: AGL, ASL, AGL_Origin AGL_Slope

    }
}


// Function to automatically point the LIDAR to nadir. Useful when maneuvering. 
function lidar_nadir {
    local parameter lidar.

}


set quadcopter to ship. // For future use.

function start_up_sequence {

    SET blade to blades:ITERATOR.
    UNTIL NOT blade:next {
        local M is blade:value:getmodule("ModuleControlSurface").
        M:setfield("deploy angle", blade_pitch).
    }

    SET rotor TO rotors:ITERATOR.
    UNTIL NOT rotor:NEXT {
        // Strong torque means more reactive controls and more stable control. Tradeoff for consumption from battery. 
        set_torque(rotor:VALUE, torque_limit).
        set_rpm(rotor:VALUE, 50).
    }
}

function shut_down_sequence {
    SET rotor TO rotors:ITERATOR.
    UNTIL NOT rotor:NEXT {
        set_torque(rotor:VALUE, 0).
        set_rpm(rotor:VALUE, 0).
    }
}

function laser_on {
    SET laser_module TO SHIP:MODULESNAMED("LaserDistModule")[0].
    IF not laser_module:GETFIELD("Enabled") {
        laser_module:SETFIELD("Enabled",true).
    }
}

function laser_off {
    SET laser_module TO SHIP:MODULESNAMED("LaserDistModule")[0].
    IF laser_module:GETFIELD("Enabled") {
    laser_module:SETFIELD("Enabled",false).
    }
}

function get_laser_altitude {
    SET laser_module TO SHIP:MODULESNAMED("LaserDistModule")[0].

    laser_module:SETFIELD("Bend X", pitch_for(quadcopter)).
    laser_module:SETFIELD("Bend Y", -roll_for(quadcopter)).

    return laser_module:GETFIELD("Distance").
}


set state to "takeoff".
//set state to "land".

// Initialize and set some defaults.
set pitch_target to 0.
set roll_target to 0.
set heading_target to 90.
//set thrust_target to 0.

set altitude_target to 50.

set forward_velocity_target to 0.
set lateral_velocity_target to 0.
//set vertical_velocity_target to 0.

set speed_target to forward_velocity_limit.

lock altitude_reference to alt:radar.

set destination to SHIP:GEOPOSITION.
//
//set destination to dockLocation.

//print(SHIP:GEOPOSITION).
//print(destination).

set timer to 0.

// Bootstrap state machine to start executing. 
set state to "task_complete".

// Set this to 0 to force running as soon as the loop starts (assuming you're starting more than 30 seconds after the creation of Kerbin)
set comms_manager_lastrun to 0.

set emergency to False.

set gear_bypass to False.

ON ABORT {
    set emergency to True.
}

// The Main Loop
until state = "exit" {

    // Mission Manager -- Controls the state machine driven by the mission Queue. 
    if state = "task_complete" {

        // Check if a mission was loaded. If not, execute the default mission. 
        if defined mission {
            set current_task to mission:pop.
            if mission:length > 1 {
                set next_task to mission:peek.
            }

        } else {
            set current_task to default_mission:pop.
            if default_mission:length > 1 {
                set next_task to default_mission:peek.
            }
        }
        
        set state to current_task:State.

        set pre_start to true.

    }

    // Run Comms Manager to ensure connectivity. Currently only runs every 30 seconds to reduce load. 
    // This should be fine for most MEO to GEO comms constellations. 
    if TIME:SECONDS - comms_manager_lastrun > 30 and use_comms_manager {
        comms_manager(commsat_list, SHIP:PARTSDUBBED("auto_antenna")).
        SET comms_manager_lastrun TO TIME:SECONDS.
    }

    if state = "takeoff" {

        if pre_start {

            wait 1.

            // Undock
            // There is a Kraken risk here. If Krakening, comment out this code and manually undock before starting the program.

            //print(ship:partstagged("copterDockingPort")[0]:STATE).

            local undock is ship:partstagged("copterDockingPort").
            if undock:length > 0 and undock[0]:state = "Docked (dockee)" {
                print("Undocking Now").
                undock[0]:partner:undock().
                set gear_bypass to True. 
            } else if undock:length > 1 {
                //Too many tagged docking ports. Something will go wrong. 
            }
            //UNTIL NOT ship:partstagged("copterDockingPort")[0]:HASPARTNER {
                // Can't do anything while still docked. Need to manually undock since I can't make it not explode when undocking with code above. 
                // This will hold the program until manually undocked. 
                wait 0.
            //}

            // Check if min_solar_exposure is set (should be False if RTG equipped) If set, then check to make sure we have that minimum exposure.
            // This prevents takeoff if there's not enough power for a real sustained flight. 
            //print(min_solar_exposure).
            if min_solar_exposure {
                if ship:sensors:light >= min_solar_exposure {
                    start_up_sequence().
                    BRAKES OFF.
                    set pre_start to False.
                } else {
                    print(ship:sensors:light + " is below the minimum solar exposure threshold of " + min_solar_exposure + ".").
                    print("Waiting for sufficient light.").
                    wait until ship:sensors:light >= min_solar_exposure.
                 }
            } else {
                    print("Starting up Quadcopter").
                    start_up_sequence().
                    BRAKES OFF.
                    set pre_start to False.
            }

        }

        set altitude_target to current_task:Altitude.

        // Once within 1m altitude, complete takeoff task. 
        if abs( altitude_reference - altitude_target ) < 1 {
            set state to "task_complete".
            set gear_bypass to False. 
            //set state to "calibrate".           // Uncomment this to short-circuit the state machine and go into calibration mode. 
        }

    } else if state = "fly" {

        // Setups to take care of only once, at the start of the flight state. 
        if pre_start {

            print("Flying to destination").

            set destination to parse_destination(current_task:Destination).
            set altitude_target to current_task:Altitude.

            set pre_start to False. 
        }


        set heading_target to get_bearing(SHIP:GEOPOSITION, destination).

        // Don't start flying until within certain deadband of heading.
        if abs(heading_pid:error) > 20 {
            set forward_velocity_target to 0.
            set lateral_velocity_target to 0.

        } else { 
            // Need time to decelerate, and this will depend on the body's atmosphere and PID tuning.
            set forward_velocity_target to abs(max(min(get_distance(SHIP:GEOPOSITION, destination)/8, forward_velocity_limit), 0.5)).
        }

        //if get_distance(SHIP:GEOPOSITION, destination:GEOPOSITION) < 500 {
        if get_distance(SHIP:GEOPOSITION, destination) < 10 {
            //print("Range less than 25m. Switching to targeted hover mode").
            //set state to "hover".
            set state to "task_complete".
            
        }

    } else if state = "hover" {

        // Setups to take care of only once, at the start of the flight state. 
        if pre_start {
            print("Mode:          Hover at location").
            if current_task:haskey("Destination") {
                set destination to parse_destination(current_task:Destination).
                if current_task:Destination[0] = "dock" {
                    set heading_target to compass_for(vessel(current_task:Destination[1])).
                }

            } else {
                set destination to ship:geoposition.
            }
            set altitude_target to current_task:Altitude.

            set timer to -1.

            set pre_start to False. 
        }

        keep_position(destination).

        // clearscreen.
        // print("Mode:          Hover at location").
        // print("heading       " + compass_for(ship)).
        // print("heading TGT   " + heading_target).
        // print("prograde vec  " + SHIP:SRFPROGRADE:VECTOR:X).
        // print("groundspeed   " + ship:groundspeed).
        // print("Pitch - Roll  " + pitch_for(ship) + " - " + roll_for(ship)).
        // //print("Setpoint:     " + forward_velocity_pid:setpoint + " - " + lateral_velocity_pid:setpoint).
        // //print("Velocity:     " + round(forward_velocity,3) + " - " + round(lateral_velocity,3)).
        // print("Pos Error:    " + round(x_pid:error,3) + " - " + round(y_pid:error,3)).
        // print("PID P Terms:  " + round(x_pid:pterm,3) + " - " + round(y_pid:pterm,3)).   
        // print("PID I Terms:  " + round(x_pid:iterm,3) + " - " + round(y_pid:iterm,3)).   
        // print("PID D Terms:  " + round(x_pid:dterm,3) + " - " + round(y_pid:dterm,3)).     
        // // print("Emergency:    " + emergency).
        // print(corrections).
        
        //TODO: Make this configurable in the mission control.
        if emergency {
            // If emergency, just get a "good enough" position. Don't waste time getting precise.  
            set location_error to 2.5.
            set speed_error to 1.5.
        } else {
            set location_error to 0.1.
            set speed_error to 0.1.
        }

        // Check if we've got good positional error. if so, start the timer.
        if sqrt( x_pid:error*x_pid:error + y_pid:error*y_pid:error) < location_error and VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:VELOCITY:SURFACE):MAG < speed_error {
           //set state to "task_complete".
           if timer = -1 set timer to TIME:SECONDS.
        } 

        // Check if timer initialized, and if we're past our wait time, finish the task.
        if timer <> -1 and (TIME:SECONDS - timer) > current_task:Time {
            set state to "task_complete".
        }
    
    } else if state = "emergency_land" {

        // Find the flattest spot to land within 500 meters execute an emergency landing. 
        print("Going into Emergency Landing Mode.").
        print("Coarse Searching for suitable landing zone.").
        set coarse_search to search_slope(ship:geoposition, emergency_search_range, 50, 45, 5).
        print("Fine Searching for suitable landing zone.").
        set destination to search_slope(coarse_search[0], emergency_search_range / 5, 20, 30, 2)[0].

        // Overwrite the mission with a new emergency landing mission. 
        set mission to QUEUE(
            lexicon("State", "fly", 
                    "Destination", list("geoposition", destination),
                    "Altitude", 50,
                    "MaxSpeed", 30),
            lexicon("State", "Hover",
                    "Destination", list("geoposition", destination),
                    "Altitude", 50,
                    "Time", 0),
            lexicon("State", "land"),
            lexicon("State", "exit")
            ).

        set state to "task_complete".
    
    } else if state = "land" {

        if pre_start {

            print("Landing now.").

            //laser_on().
            set pre_start to False.
        }

        set altitude_target to 0.

        // Somewhat dangerous? Relies on a destination having been pre-set in a previous task. 
        keep_position(destination).

        if alt:radar < 0.4 {
            set state to "parked".
        }

    } else if state = "dock" {

        if pre_start {

            set target_dock to vessel(current_task:destination[1]):partstagged(current_task:destination[2])[0].
            SET TARGET TO target_dock. //vessel(current_task:destination[1]):partstagged(current_task:destination[2])[0].
            print("Descending to dock.").
            set pre_start to False. 

            set target_dock_altitude to vessel(current_task:destination[1]):geoposition:altitudeposition(vessel(current_task:destination[1]):geoposition:terrainheight):mag + (up:forevector*target_dock:nodeposition).
            print("Dockign port altitude is " + target_dock_altitude + "m above terrain.").
        }

        // Nominally, the port will go through these states:
        // Ready
        // Acquire (dockee)
        // Docked (dockee)
        //print(ship:partstagged("copterDockingPort")[0]:STATE).

        // Check to see if we're docked yet or not. If not, we can proceed with docking, and will set our heading to match the target. 
        //print(ship:partstagged("copterDockingPort")[0]:STATE). // States didn't seem to match documentation. This is for debugging. 
        if ship:partstagged("copterDockingPort")[0]:STATE = "Docked (docker)" or ship:partstagged("copterDockingPort")[0]:STATE = "Docked (dockee)" or ship:partstagged("copterDockingPort")[0]:STATE = "Docked (same vessel)" or ship:partstagged("copterDockingPort")[0]:STATE = "Acquire" {
            set state to "parked".
        } else if not vessel(current_task:Destination[1]):isdead {
            set heading_target to compass_for(vessel(current_task:Destination[1])).
            // To Do: Make the relative heading angle configurable in the mission. 
        }

        set altitude_target to target_dock_altitude.

        keep_position(destination).
    
        // Check if docked, then go to "parked" mode. 

    } else if state = "parked" {

        laser_off().

        BRAKES ON.

        shut_down_sequence().


        wait 1.
        // Probably do a few other things too, to shut down the copter. 

        set state to "task_complete".

    } else if state = "science" {

        if pre_start {

            set timer to time:seconds.

            // Check if any experiments are left in the list. 
            if current_task:Instrument:Length > 0 {
                set next_experiment to current_task:Instrument[0].
                current_task:Instrument:remove(0).
                print("Running Experiment: " + next_experiment).

                set instrument_list to SHIP:PARTSNAMED(next_experiment).

                            // Check if any instruments of that type are returned.
                if instrument_list:length > 0 {
                    set i to 0.

                    set valid to True.
                    until instrument_list[i]:GETMODULE("ModuleScienceExperiment"):INOPERABLE = FALSE and instrument_list[i]:GETMODULE("ModuleScienceExperiment"):HASDATA = False {

                        // If nothing available, task should be completed and move on. 
                        if i >= instrument_list:length -1 {
                            print("No instruments available, everything seems to have data or is otherwise inoperable.").
                            set pre_start to True.
                            set valid to False.
                            break.
                        }
                        set i to i + 1.
                    }
                    if valid { instrument_list[i]:GETMODULE("ModuleScienceExperiment"):DEPLOY(). }

                    set pre_start to False. 

                } else {
                    print("No instruments available -- Does this craft have one? If so, check spelling/name.").
                    // No instruments returned. Move onto the next thing. 
                }
                
            } else {
                print("Science Task Complete").
                set state to "task_complete".
            }
        }

        if instrument_list:length > 0  and valid {
            if instrument_list[i]:GETMODULE("ModuleScienceExperiment"):HASDATA {
                if current_task:Transmit {
                    // TODO: Check if enough battery to transmit science? Otherwise we'll crash. 
                    instrument_list[i]:GETMODULE("ModuleScienceExperiment"):TRANSMIT().
                    
                    if instrument_list[i]:GETMODULE("ModuleScienceExperiment"):RERUNNABLE {
                        instrument_list[i]:GETMODULE("ModuleScienceExperiment"):RESET(). 
                    }
                }

                // Check if we have any other items in the queue. If so, go onto the next item, otherwise move to the next task. 
                if current_task:Instrument:LENGTH > 0 {
                    set pre_start to True.
                } else {
                    set state to "task_complete".
                }
            }
        }

        // Check if experiment is timing out. Can be useful if the experiment is in invalid situation (e.g. Gravioli in flight). 
        // Perhaps there's a better way to check if an experiment is in an invalid situation, but I don't know how. 
        // Is 2 seconds long enough for all experiments? If you modify the craft, this may need to change. 
        if TIME:SECONDS > timer + 2 {
            print("Science Instrument timed out. Is this a valid situation?").
            set pre_start to True.
        }

        // if state is "flying" then keep location. Should be preceded by hover command but not nessecary. (I think).
        // We rely on a destination having been set in the previous command. 
        keep_position(destination).
        // If state is "landed" or "docked", don't keep location, just do science. For science!


    } else if state = "recovery" {

        // Kill ship velocity and get to a stable altitude. 
        // May want to bypass the velocity and force setting attitude instead. 
        
        set altitude_target to 50.

        set forward_velocity_target to 0.
        set lateral_velocity_target to 0.
        //set vertical_velocity_target to 0.

        // Implement some checks to then do an emergency landing. 
        if abs(pitch_for(ship)) < 2 and abs(roll_for(ship)) < 2 {
            if abs(pitch_pid:iterm) < 2 and abs(roll_pid:iterm) < 2 {
                            set state to "emergency_land".
            }
        }

    } else if state = "calibration" {

        if pre_start {
            clearscreen.
            print("Entering Calibration Mode").
            set cycle_count to 0.
            set count_time to TIME:SECONDS.
            set pre_start to False.
            set altitude_setpoint to current_task:Altitude.

            set pitch_target to 0.
            set roll_target to 0.

            set forward_velocity_target to 0.
            set lateral_velocity_target to 0.
        }

        if cycle_count < current_task:Cycles {
            if TIME:SECONDS > (count_time+current_task:Interval) {
                print(current_task:mode).
                if current_task:mode = "pitch" {
                    print(pitch_target).
                    if pitch_target = 0 or pitch_target = -current_task:Delta {
                        print("Setting Pitch to " + current_task:Delta).
                        set pitch_target to current_task:Delta.
                    } else { //if pitch_target = current_task:Delta {
                        print("Setting Pitch to -"+current_task:Delta).
                        set pitch_target to -current_task:Delta.
                    }
 //               set roll_target to 0.

                } else if current_task:mode = "forward_velocity" {
                    if forward_velocity_target = 0 or forward_velocity_target = -current_task:Delta {
                        print("Setting Velocity to " + current_task:Delta).
                        set forward_velocity_target to current_task:Delta.
                    } else { //if pitch_target = current_task:Delta {
                        print("Setting Velocity to -" + current_task:Delta).
                        set forward_velocity_target to -current_task:Delta.
                    }
                }
            set cycle_count to cycle_count + 1.
            set count_time to TIME:SECONDS.
            }
        } else {
            set state to "task_complete".
        }

        if current_task:mode = "pitch" {
            // Todo: Log debug outputs. 
        } else if current_task:mode = "forward_velocity" {
            // Todo: Log debug outputs.
            clearscreen.
            print(forward_velocity_target).
            print(forward_velocity_pid:setpoint).
            print("PID P Terms:  " + round(forward_velocity_pid:pterm,3)).   
            print("PID I Terms:  " + round(forward_velocity_pid:iterm,3)).   
            print("PID D Terms:  " + round(forward_velocity_pid:dterm,3)).   
            velocity_loop(forward_velocity_target, lateral_velocity_target).  
        }
    
    set altitude_setpoint to altitude_target. // altitude_loop(altitude_target).
    //velocity_loop(forward_velocity_target, lateral_velocity_target).
    
    stability_loop(pitch_target,roll_target,heading_target,altitude_setpoint).

    }

    if state <> "calibration" {

        set altitude_setpoint to altitude_target. // altitude_loop(altitude_target).
        velocity_loop(forward_velocity_target, lateral_velocity_target).
        stability_loop(pitch_target,roll_target,heading_target,altitude_setpoint).

    }

    // Check if we're pitched/rolled too much (indicating something has gone wrong.). If so, go into recovery mode. 
    if state <> "calibration" {
        if abs(pitch_for(ship)) > 60 {
           // print("Unstable! Going into recovery").
            set state to "recovery".
        }
        if abs(roll_for(ship)) > 60 {
           // print("Unstable! Going into recovery").
            set state to "recovery".
        }
    }

    // Check for emergency conditions. If any exist, go into recovery.
    
    if not emergency {
        // Don't do these checks if already landing or parked. No need. 
        if state <> "land" and state <> "parked" and state <> "dock" {
            // Check for low battery
            if SHIP:ELECTRICCHARGE < low_battery_threshold {
                print("Low Battery, going into Emergency Landing.").
                set state to "recovery".

                set emergency to True. 
            }
        }

    }

    // Do landing gear automagically! Because we can! But not always.
    if state = "fly" or state = "hover" or state = "land" or state = "science" or state = "takeoff" {
        if not gear_bypass {
            SET GEAR TO ALT:RADAR < 25.          
        }
    }
    

// Some debug arrows.... 

//    SET headingArrow TO VECDRAWARGS(
//    v(0,0,0),
//    VECTOREXCLUDE(SHIP:UP:FOREVECTOR, SHIP:FACING:FOREVECTOR),
//    //SHIP:FACING:FOREVECTOR,
//    RED,
//    "Heading",
//    1,
//    TRUE
//    ).

//    SET facingArrow TO VECDRAWARGS(
//    v(0,0,0),
//    ship:facing:forevector,
//    GREEN,
//    "facing",
//    1,
//    TRUE
//    ).

//     SET progradeArrow TO VECDRAWARGS(
//     v(0,0,0),
//     VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR),
//     GREEN,
//     "Prograde",
//     1,
//     TRUE
//     ).

//     SET zenithArrow TO VECDRAWARGS(
//     v(0,0,0),
//     SHIP:UP:FOREVECTOR,
//     YELLOW,
//     "Zenith",
//     1,
//     TRUE
//     ).

    wait 0.

}

// Cleanup. These cause issues between missions if they persist. 
unset min_solar_exposure.
unset mission.