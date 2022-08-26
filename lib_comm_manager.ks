function comms_manager {
    parameter available_commsats.
    parameter antennas. 

    local bird is available_commsats:ITERATOR.

    local useable_commsats is LIST().
    
    UNTIL NOT bird:NEXT {

        local elevationAngle is 90 - vAng(-SHIP:GEOPOSITION:POSITION, VESSEL(bird:VALUE):DIRECTION:VECTOR).

        // Check if the minimum elevation angle is greater than 10 degrees.
        if elevationAngle > 10 {
            local sat_info is LIST().
            local range is VESSEL(bird:VALUE):distance.
            local sat_info is LIST(vessel(bird:VALUE), elevationAngle, range).
            if useable_commsats:length = 0 {
                useable_commsats:add(sat_info).
            } else  {
                local X is 0.
                UNTIL X >= useable_commsats:length {      // Prints the numbers 1-10
                    
                    if sat_info[2] < useable_commsats[X][2] {
                        useable_commsats:INSERT(X,sat_info).
                        break.
                    } 
                    SET X to X + 1.
                }
                // Catch if it wasn't larger than anything in the list, and if so then add it. 
                if X = useable_commsats:length {
                    useable_commsats:add(sat_info).
                }
                
            }        
    }

    //print(useable_commsats).

    }

    local X is 0.

    for antenna in antennas {
           //print(X).
        // Loop through and assign antennas. 
        SET M to antenna:GETMODULE("ModuleRTAntenna").
        //print(useable_commsats:length).
        if X < useable_commsats:length {

            // Check if activated, if not then activate. 
            if M:GETFIELD("status") = "Off" {

                M:DOEVENT("activate").
            }
            M:SETFIELD("target", useable_commsats[X][0]).

        } else {
            // No remotes left. Check if antenna is activated, and if so de-activate.
            if M:GETFIELD("status") = "Operational" {
                M:DOEVENT("deactivate").
            }

        }

        SET X to X + 1.
    }

}