clearScreen.

run lib_navball.
run lib_comm_manager.

set commsat_list to LIST("CommSat I Block II", "CommSat II Block II", "CommSat III Block II").

set dock to vessel("LandingTarget"):partstagged("targetDock")[0].
set dockLocation to body:geopositionof(dock:nodeposition).
set dockAltitude to body:altitudeof(dock:nodeposition).
print(dockLocation + " - " + dockAltitude).


comms_manager(commsat_list, SHIP:PARTSDUBBED("auto_antenna")).

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

function get_bearing {
    parameter A.
    parameter B.

    local X is cos(B:LAT)*sin(B:LNG - A:LNG).
    local Y is ( cos(A:LAT)*sin(B:LAT) ) - (sin(A:LAT)*cos(B:LAT)*COS(B:LNG -A:LNG)).
    local bearing is arctan2(X, Y).
 //   print bearing.
    return bearing.
}

function get_distance {
    parameter A.
    parameter B.
    local distance is vang(A:position-body:position,B:position-body:position)*Constant:DegToRad*body:radius.

 //   print distance.
    return distance.
}


function get_velocity {

    local sinYaw is sin(ship:up:yaw).
    local cosYaw is cos(ship:up:yaw).
    local sinPitch is sin(ship:up:pitch).
    local cosPitch is cos(ship:up:pitch).

    local unitVectorEast is V(-cosYaw, 0, sinYaw).
    local unitVectorNorth is V(-sinYaw*sinPitch, cosPitch, -cosYaw*sinPitch).
    
    local shipVelocitySurface is ship:velocity:surface.
    local speedEast is vdot(shipVelocitySurface, unitVectorEast).
    local speedNorth is vdot(shipVelocitySurface, unitVectorNorth).

    return list(speedEast, speedNorth).

}

function position_error {
    parameter A.
    parameter B.

    //local x_err is vang(VECTOREXCLUDE(BODY:UP:FOREVECTOR, A:position-body:position),VECTOREXCLUDE(BODY:UP:FOREVECTOR, A:position-body:position))*Constant:DegToRad*body:radius.
    //local y_err is vang(VECTOREXCLUDE(BODY:UP:FOREVECTOR, A:position-body:position),VECTOREXCLUDE(BODY:UP:FOREVECTOR, A:position-body:position))*Constant:DegToRad*body:radius.
    local x_err is (A:LNG - B:LNG)*Constant:DegToRad*body:radius.
    local y_err is (A:LAT - B:LAT)*Constant:DegToRad*body:radius.

    return (list(x_err, y_err)).
}

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


function clamp {
    parameter A.
    parameter MIN.
    parameter MAX.

    return MAX(MIN(A, MAX), MIN).
}

function navigation_loop {

//    local speed_target is 40.
    set forward_velocity_pid:setpoint to speed_target.
    return -forward_velocity_pid:update(TIME:SECONDS, ship:groundspeed).
}

function stability_loop {
    parameter pitch.
    parameter roll.
    parameter heading_1.
    parameter altitude.
    //parameter climb_rate.

    // Avoid rollover errors when pointing north. This probably breaks naviation for a range of angles. In fact I know it does.
    if compass_for(ship) < 45 {
        set heading_1 to heading_1 + 180.
        set current_compass to compass_for(ship) + 180.
    } else if compass_for(ship) > 315 {
        set heading_1 to heading_1 + 180.
        set current_compass to compass_for(ship) - 180.
    } else {
        set current_compass to compass_for(ship).
    }

    // Adjust PID parameters for thrust when going up vs going down. (Gravity compensation!)

    if altitude < alt:radar - 2 {
        set altitude_pid:kp to 5.
        set altitude_pid:ki to 0.01.
        set altitude_pid:kd to 35.
    } else {
        set altitude_pid:kp to 10. // was 20
        set altitude_pid:ki to 0.25.
        set altitude_pid:kd to 15.
    }

    set altitude_pid:setpoint to altitude.
    set heading_pid:setpoint to heading_1.
    set pitch_pid:setpoint to pitch.
    set roll_pid:setpoint to roll.

    set roll to roll_pid:UPDATE(TIME:SECONDS, roll_for(ship)).
    set pitch to pitch_pid:UPDATE(TIME:SECONDS, pitch_for(ship)).
    set yaw to heading_pid:UPDATE(TIME:SECONDS, current_compass).
    //set thrust to altitude_pid:UPDATE(TIME:SECONDS, ALT:RADAR) + 80.

    //print("Pitch " + pitch_for(ship) + " - " + pitch).
    //print("roll " + roll_for(ship) + " - " + roll).

    set_rpm("leftFrontRotor", thrust + yaw + pitch + roll).
    set_rpm("rightFrontRotor", thrust - yaw + pitch - roll).
    set_rpm("rightRearRotor", thrust + yaw - pitch - roll).
    set_rpm("leftRearRotor", thrust - yaw - pitch + roll).

}

function control_loop {

    parameter forward_velocity.
    parameter lateral_velocity.
    parameter altitude.

}




set rotors to LIST("leftFrontRotor", "leftRearRotor", "rightRearRotor", "rightFrontRotor").

SET heading_pid to PIDLOOP(1.0,0.4,0.6, -20,20).
SET altitude_pid to PIDLOOP(20,0.25,15, 60-80, 300-80).
//SET roll_pid to PIDLOOP(1.4,0.2,0.8). //d 1.2
//SET pitch_pid to PIDLOOP(1.4,0.2,0.8).
SET roll_pid to PIDLOOP(1.3,0.5,1.0). //d 1.2
SET pitch_pid to PIDLOOP(1.3,0.5,1.0).


//SET speed_pid to PIDLOOP(0.01,0.001,0.1,-10,10).
//SET forward_velocity_pid to PIDLOOP(5,0,2.5,-20,20).
//SET lateral_velocity_pid to PIDLOOP(5,0,2.5,-20,20).
SET forward_velocity_pid to PIDLOOP(4,2,2.5,-20,20).
SET lateral_velocity_pid to PIDLOOP(4,2,2.5,-20,20).
SET vertical_velocity_pid to PIDLOOP(1,0,0).

SET steering_pid to PIDLOOP(0.5,0.25,1,-30,30).
SET roll_steering_pid to PIDLOOP(2,0.5,2,-5,5).

//SET x_pid to PIDLOOP(4, 3, 2,-1.5,1.5).
//SET y_pid to PIDLOOP(4, 3, 2,-1.5,1.5).

SET x_pid to PIDLOOP(1, 0.5, 2,-1,1).
SET y_pid to PIDLOOP(1, 0.5, 2,-1,1).

set quadcopter to ship.

// Deploy Sequence
    // Set hinge angles
    // When angle acheived lock
    // disable motor
    // unlock rotors


// Start up the rotors
SET rotor TO rotors:ITERATOR.
UNTIL NOT rotor:NEXT {
    print rotor:VALUE.
    set_torque(rotor:VALUE, 100).
    set_rpm(rotor:VALUE, 50).
}


// States: init, rest, takeoff, fly, land
set state to "takeoff".
//set state to "land".

set pitch_target to 0.
set roll_target to 0.
set heading_target to 90.
set altitude_target to 50.

set speed_target to 10.

lock altitude_reference to alt:radar.

//set destination to SHIP:GEOPOSITION.
set destination to dockLocation.

print(SHIP:GEOPOSITION).
//print(destination).

for location in allwaypoints() {
    if location:ISSELECTED{
        print("Waypoint is " + location:GEOPOSITION).
        set destination to waypoint(location:name):GEOPOSITION.
    }
}

set count_time to TIME:SECONDS.



// The Main Loop
until 0 {



    if state = "takeoff" {

        // Undock

        BRAKES OFF.

        // Don't maneuver until 20m above terrain. 
        if ALT:RADAR >20 {
            set state to "fly".
            set state to "calibrate".
            print("Setting state to fly").
        }
        // Todo: Get fixed status r heading and lock it in....
        stability_loop(0,0,90,altitude_target).

    } else if state = "fly" {

        set heading_target to get_bearing(SHIP:GEOPOSITION, destination).

        local v1 is get_velocity.

        set v1 to vectorRotate(v1[1],v1[0],-compass_for(ship)).

        local forward_velocity is v1[0].
        local lateral_velocity is v1[1].
        set lateral_velocity_pid:setpoint to 0.
        set roll_target to lateral_velocity_pid:update(TIME:SECONDS, lateral_velocity).

        if heading_target < 0 {
        set heading_target to 360 + heading_target.
        }

       // print("heading/compass: " + heading_target + " - " + compass_for(ship)).
        //local steering_error is VANG(VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR),SHIP:UP:FOREVECTOR + R(0,-90,0)).
        
        local steering_error is rotateFromTo(VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR), VECTOREXCLUDE(SHIP:UP:FOREVECTOR, SHIP:FACING:FOREVECTOR)):PITCH.
        //print(steering_error).

        if steering_error >=180 {
            set steering_error to steering_error - 360.
        }

        //print(heading_target + " - " + steering_error).

        set roll_steering_pid:setpoint to 0.

        //print(steering_error + " - " + get_distance(SHIP:GEOPOSITION, destination)).

        local heading_error is heading_target - compass_for(ship).

        if heading_error > 180 {
            set heading_error to heading_error - 360.
        } else if heading_error < -180 {
            set heading_error to heading_error + 360.
        }

                //print(heading_error).

        if abs(heading_pid:error) > 10 {
            set forward_velocity_pid:setpoint to 0.
            set pitch_target to 0.
            set roll_target to 0.
        
        } else {
            //set speed_pid:setpoint to get_distance(SHIP:GEOPOSITION, destination:GEOPOSITION).    
            set forward_velocity_pid:setpoint to abs(min(get_distance(SHIP:GEOPOSITION, destination)/10-0.1, 20)).
            //set speed_pid:setpoint to speed_target.

        }
        //set steering_pid:setpoint to 0.
        //set roll_target to stering_pid:update(TIME:SECONDS, SHIP:heading)
      
        //set pitch_target to clamp(get_distance(SHIP:GEOPOSITION, destination:GEOPOSITION)., -30, 30).
        //set pitch_target to get_distance(SHIP:GEOPOSITION, destination:GEOPOSITION).
            set pitch_target to -forward_velocity_pid:update(TIME:SECONDS, ship:groundspeed).
           // set roll_target to roll_steering_pid:update(TIME:SECONDS, steering_error).

        stability_loop(pitch_target,roll_target,heading_target,altitude_target).

        clearscreen.
        print("Mode:          Fly").
        print("heading       " + compass_for(ship)).
        print("Target Hdg    " + heading_target).
        print("prograde vec  " + SHIP:SRFPROGRADE:VECTOR:X).
        print("groundspeed   " + ship:groundspeed).
        print("Distance      " + get_distance(SHIP:GEOPOSITION, destination)).
  

        //if get_distance(SHIP:GEOPOSITION, destination:GEOPOSITION) < 500 {
        if get_distance(SHIP:GEOPOSITION, destination) < 10 {
            //print("Range less than 25m. Switching to targeted hover mode").
            set state to "hover".
            //forward_velocity_pid:reset.
        }

    } else if state = "hover" {

        //set heading_target to 0.

        local steering_error is rotateFromTo(VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR), VECTOREXCLUDE(SHIP:UP:FOREVECTOR, SHIP:FACING:FOREVECTOR)):PITCH.

        //local forward_velocity is ship:groundspeed * cos(compass_for(ship)).
        //local lateral_velocity is ship:groundspeed * sin(compass_for(ship)).

        //local forward_velocity is ship:groundspeed * VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR):X.
        //local lateral_velocity is ship:groundspeed * VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR):Y.

        //local v1 is VXCL(up:forevector,ship:velocity:surface).
        //set v1 to VXCL(SHIP:UP:VECTOR, SHIP:velocity:surface).
        local v1 is get_velocity.

        set v1 to vectorRotate(v1[1],v1[0],-compass_for(ship)).

        local forward_velocity is v1[0].
        local lateral_velocity is v1[1].

        //local forward_velocity is ship:VELOCITY:SURFACE:Y.
        //local lateral_velocity is ship:VELOCITY:SURFACE:X.

        //local x_err is sin(get_bearing(SHIP:GEOPOSITION, destination)) * get_distance(destination, SHIP:GEOPOSITION).
        //local y_err is cos(get_bearing(SHIP:GEOPOSITION, destination)) * get_distance(destination, SHIP:GEOPOSITION).

        local new_err is position_error(destination, SHIP:GEOPOSITION).
              

        local x_err is new_err[0].
        local y_err is new_err[1].

        set x_pid:setpoint to 0.
        set y_pid:setpoint to 0.

        local x_correction is x_pid:update(TIME:SECONDS, x_err).
        local y_correction is y_pid:update(TIME:SECONDS, y_err).


//        set forward_velocity_pid:setpoint to +sin(compass_for(ship))*x_correction + cos(compass_for(ship))*y_correction.
//        set lateral_velocity_pid:setpoint to -sin(compass_for(ship))*y_correction - cos(compass_for(ship))*x_correction.

// x2=cosβx1−sinβy1
// y2=sinβx1+cosβy1

        //set lateral_velocity_pid:setpoint to cos(compass_for(ship))*x_correction - sin(compass_for(ship))*y_correction.
        //set forward_velocity_pid:setpoint to sin(compass_for(ship))*x_correction  cos(compass_for(ship))*y_correction.

        set corrections to vectorRotate(x_correction, y_correction, compass_for(ship)).

        set lateral_velocity_pid:setpoint to -corrections[0].
        set forward_velocity_pid:setpoint to -corrections[1].


        //set lateral_velocity_pid:setpoint to 0.
        //set forward_velocity_pid:setpoint to 0.

        clearscreen.
        print("Mode:          Hover at location").
        print("heading       " + compass_for(ship)).
        print("prograde vec  " + SHIP:SRFPROGRADE:VECTOR:X).
        print("groundspeed   " + ship:groundspeed).
        print("Setpoint:     " + forward_velocity_pid:setpoint + " - " + lateral_velocity_pid:setpoint).
        print("Velocity:     " + round(forward_velocity,3) + " - " + round(lateral_velocity,3)).
        print("Pos Error:    " + round(x_err,3) + " - " + round(y_err,3)).
        print("PID I Terms:  " + round(x_pid:iterm,3) + " - " + round(y_pid:iterm,3)).     

        //set logline TO (TIME:SECONDS)
        //+ " " + forward_velocity_pid:setpoint
        //+ " " + lateral_velocity_pid:setpoint
        //+ " " + forward_velocity
        //+ " " + lateral_velocity
        //+ " " + x_err
        //+ " " + y_err.


        set pitch_target to -forward_velocity_pid:update(TIME:SECONDS, forward_velocity).
        set roll_target to lateral_velocity_pid:update(TIME:SECONDS, lateral_velocity).

        //set roll_target to 0.

        //set pitch_target to -sin(compass_for(ship))*x_correction - cos(compass_for(ship))*y_correction.
        //set roll_target to -sin(compass_for(ship))*y_correction + cos(compass_for(ship))*x_correction.

        //set pitch_target to 0.
        //set roll_target to 0.

        
        stability_loop(pitch_target,roll_target,heading_target,altitude_target).

        if abs(x_err) < 0.02 and abs(y_err) < 0.02 and abs(x_pid:iterm) < 0.02 and abs(y_pid:iterm) < 0.02 {
            set state to "dock".
        }
    
    } else if state = "land" {

    } else if state = "dock" {

        print(ship:partstagged("copterDockingPort")[0]:STATE).
        if ship:partstagged("copterDockingPort")[0]:STATE = "Docked (docker)" or ship:partstagged("copterDockingPort")[0]:STATE = "Docked (dockee)" or ship:partstagged("copterDockingPort")[0]:STATE = "Docked (same vessel)"  or ship:partstagged("copterDockingPort")[0]:STATE = "Acquire (dockee)" {
            set state to "parked".
        } else if not vessel("landingTarget"):isdead {

            set heading_target to compass_for(vessel("landingTarget")).
        }


        set altitude_target to 4.

        local steering_error is rotateFromTo(VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR), VECTOREXCLUDE(SHIP:UP:FOREVECTOR, SHIP:FACING:FOREVECTOR)):PITCH.

        local v1 is get_velocity.

        set v1 to vectorRotate(v1[1],v1[0],-compass_for(ship)).

        local forward_velocity is v1[0].
        local lateral_velocity is v1[1].

        local new_err is position_error(destination, SHIP:GEOPOSITION).
              

        local x_err is new_err[0].
        local y_err is new_err[1].

        set x_pid:setpoint to 0.
        set y_pid:setpoint to 0.

        local x_correction is x_pid:update(TIME:SECONDS, x_err).
        local y_correction is y_pid:update(TIME:SECONDS, y_err).

        set corrections to vectorRotate(x_correction, y_correction, compass_for(ship)).

        set lateral_velocity_pid:setpoint to -corrections[0].
        set forward_velocity_pid:setpoint to -corrections[1].

        clearscreen.
        print("Mode:          Dock").
        print("heading       " + compass_for(ship)).
        print("prograde vec  " + SHIP:SRFPROGRADE:VECTOR:X).
        print("groundspeed   " + ship:groundspeed).
        print("Setpoint:     " + forward_velocity_pid:setpoint + " - " + lateral_velocity_pid:setpoint).
        print("Velocity:     " + round(forward_velocity,3) + " - " + round(lateral_velocity,3)).
        print("Pos Error:    " + round(x_err,3) + " - " + round(y_err,3)).
        print("PID I Terms:  " + round(x_pid:iterm,3) + " - " + round(y_pid:iterm,3)).     

        set pitch_target to -forward_velocity_pid:update(TIME:SECONDS, forward_velocity).
        set roll_target to lateral_velocity_pid:update(TIME:SECONDS, lateral_velocity).
        
        stability_loop(pitch_target,roll_target,heading_target,altitude_target).

    } else if state = "parked" {

        clearscreen.
        print("Park").

        BRAKES ON.


    }
    
    else if state = "recovery" {

        stability_loop(0,0,0,100).
    } else if state = "calibrate" {
        // Loop to calibrate pitch/roll.... 
        // Every 10 seconds bump pitch by 10 or so degrees opposite direction.
        //print("Calibration Mode!").

        if TIME:SECONDS > (count_time+10) {
            if pitch_target= 0 or pitch_target = -20 {
                print("Setting Pitch to 20").
                set pitch_target to 20.
            } else if pitch_target = 20 {
                print("Setting Pitch to -20").
                set pitch_target to -20.
            }

            set count_time to TIME:SECONDS.
        }
        





        stability_loop(pitch_target,roll_target,heading_target,altitude_target).


    }

    //stability_loop(0, 0, 90, 200).

    if state <> "calibration" {
        if abs(pitch_for(ship)) > 60 {
            print("Unstable! Going into recovery").
            set state to "recovery".
        }
        if abs(roll_for(ship)) > 60 {
            print("Unstable! Going into recovery").
            set state to "recovery".
        }
    }

    // Do landing gear automagically!
    if state <> "dock" and state <> "parked" and state <> "takeoff" {
        SET GEAR TO ALT:RADAR<25.
    }
    


//    SET headingArrow TO VECDRAWARGS(
//    v(0,0,0),
//    VECTOREXCLUDE(SHIP:UP:FOREVECTOR, SHIP:FACING:FOREVECTOR),
//    //SHIP:FACING:FOREVECTOR,
//    RED,
//    "Heading",
//    1,
//    TRUE
//    ).

//    SET progradeArrow TO VECDRAWARGS(
//    v(0,0,0),
//    VECTOREXCLUDE(SHIP:UP:FOREVECTOR , SHIP:SRFPROGRADE:VECTOR),
//    GREEN,
//    "Prograde",
//    1,
//    TRUE
//    ).

    wait 0.
}