set quadcopter to ship. 

set hinges to quadcopter:PARTSDUBBED("hinge.01.s").
//set servo to hinges[0]:getmodule("ModuleRoboticServoHinge").

local now is TIME:SECONDS.

SET hinge TO hinges:ITERATOR.
UNTIL NOT hinge:NEXT {
    local m is hinge:value:getmodule("ModuleRoboticServoHinge").
    wait 0.1.
    m:DOACTION("engage motor drive", True).
    wait 0.1.
    m:DOACTION("disengage servo lock", True).
    wait 0.1.
    m:SETFIELD("target angle", 0).

    }

WHEN TIME:SECONDS > now + 10 THEN { 
    print("Locking").
    set hinges to quadcopter:PARTSDUBBED("hinge.01.s").
    SET hinge TO hinges:ITERATOR.
    UNTIL NOT hinge:NEXT {
        local m is hinge:value:getmodule("ModuleRoboticServoHinge").
        m:DOACTION("engage servo lock", True).
        wait 0.
        m:DOACTION("disengage motor drive", True).
    }
}

// until False {
//     print(servo:GETFIELD("current angle")).
//     wait 0.
// }
//until servo:GETFIELD("current angle") = 0 {
//    wait 0.
//}
wait 50.