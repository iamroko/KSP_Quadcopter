// Mission Manager

// Fly - Destination, Altitude, Max Speed
// Hover - Destination, Altitude, Hold Time
// Science - Part List
// Takeoff - Altitude 
// Dock - Destination
// Park, Hold Time

// Destinations: Waypoint (name), dock (vessel_name, port tag), Navigation
// Altitude_Reference: Define if altitue is above terrain, or above sea-level. If not set, will assume above terrain.

set mission to QUEUE(
        lexicon("State", "takeoff",
                "Altitude", 50),
        lexicon("State", "fly", 
                "Destination", list("waypoint", "Site 0-MZR1"),
                "Altitude", 50,
                "MaxSpeed", 30),
        lexicon("State", "fly", 
                "Destination", list("waypoint", "Sector L-8LB"),
                "Altitude", 50,
                "MaxSpeed", 30),
        lexicon("State", "fly", 
                "Destination", list("waypoint", "KSC"),
                "Altitude", 50,
                "MaxSpeed", 30),
        lexicon("State", "Hover",
                "Destination", list("waypoint", "KSC"),
                "Altitude", 50,
                "Time", 0),
        lexicon("State", "land"),
        lexicon("State", "exit")
). 

run "Quadcopter.ks".