// Mission Manager

// Fly - Destination, Altitude, Max Speed
// Hover - Destination, Altitude, Hold Time
// Science - Part List
// Takeoff - Altitude 
// Dock - Destination
// Park, Hold Time
// Science

// Destinations: Waypoint (name), dock (vessel_name, port tag), Navigation
// Altitude_Reference: Define if altitue is above terrain, or above sea-level. If not set, will assume above terrain.

set mission to QUEUE(
        lexicon("State", "takeoff",
                "Altitude", 50),
        lexicon("State", "Hover",
 //               "Destination", list("waypoint", "KSC"),
                "Altitude", 50,
                "Error", 0.1,
                "Time", 60),
        lexicon("State", "land"),
        lexicon("State", "exit")
). 

run "Quadcopter.ks".