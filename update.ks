// Script to copy firmware to duna copter. This helps prevent issues with the .git file.
// This can be used as the bootloader script to automatically pull updates. 

copypath("0:/Quadcopter/Quadcopter.ks", "Quadcopter.ks").
copypath("0:/Quadcopter/deploy.ks", "deploy.ks").
copypath("0:/Quadcopter/lib_comm_manager.ks", "lib_comm_manager.ks").
copypath("0:/Quadcopter/lib_geo.ks", "lib_geo.ks").
copypath("0:/Quadcopter/lib_navball.ks", "lib_navball.ks").
copypath("0:/Quadcopter/lib_slope.ks", "lib_slope.ks").
copypath("0:/Quadcopter/Parameters", "/").
copypath("0:/Quadcopter/Missions", "/").
PRINT("Firmware updated.").