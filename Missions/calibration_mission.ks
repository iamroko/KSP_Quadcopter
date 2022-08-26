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
                "Altitude", 100),
        lexicon("State", "Hover",
                "Altitude", 100,
                "Error", 0.1,
                "Time", 0),
        lexicon("State", "calibration",
                "Altitude", 100,
                "Mode", "pitch",
                "Delta", 20,
                "Cycles", 5,
                "Interval", 20),
        lexicon("State", "land"),
        lexicon("State", "exit")
). 

run "Quadcopter.ks".