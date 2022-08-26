// Mission Manager

// Fly - Destination, Altitude, Max Speed
// Hover - Destination, Altitude, Hold Time
// Science - Part List
// Takeoff - Altitude 
// Dock - Destination
// Park, Hold Time

// Destinations: Waypoint (name), dock (vessel_name, port tag), Navigation
// Altitude_Reference: Define if altitue is above terrain, or above sea-level. If not set, will assume above terrain.


// Sensor Types: 
// GooExperiment
// SensorTemperature
// SensorBarometer
// SensorGravimeter
// SensorAccelerometer

set mission to QUEUE(
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
        lexicon("State", "Science",
                "Instrument", list("SensorThermometer"),
                "Transmit", False),   
        lexicon("State", "Hover",
                "Destination", list("navigation_target"),
                "Altitude", 50,
                "Time", 0),
        lexicon("State", "land"),
        lexicon("State", "Science",
                "Instrument", list( "GooExperiment", "SensorThermometer", "SensorBarometer", "SensorGravimeter", "SensorAccelerometer"),
                "Transmit", False),   
        lexicon("State", "takeoff",
                "Altitude", 50),    
        // lexicon("State", "fly", 
        //         "Destination", list("return to start"),
        //         "Altitude", 50,
        //         "MaxSpeed", 30),
        // lexicon("State", "Hover",
        //         "Destination", list("return to start"),
        //         "Altitude", 50,
        //         "Time", 0),
        // lexicon("State", "land"),
        lexicon("State", "fly", 
                "Destination", list("dock", "DSR Lander", "dsrPort"),
                "Altitude", 50,
                "MaxSpeed", 30),
        lexicon("State", "Hover",
                "Destination", list("dock", "DSR Lander", "dsrPort"),
                "Altitude", 50,
                "Time", 0),
        lexicon("State", "Dock",
                "Destination", list("dock", "DSR Lander", "dsrPort")),
        lexicon("State", "exit")
). 

run "Quadcopter.ks".