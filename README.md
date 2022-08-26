# Kerbal Space Program Quadcopter

## What is it?

This is a KOS script to control a Quadcopter in Kerbal Space Program. It's capable of running simple missions to explore and conduct science on Kerbin and other atmospheric bodies in the game. When paired with other vessels (such as a sample return lander) with a matching docking port, the script is also capable of docking. 

## The Quadcopter 

Included is a simple Quadcopter used to develop this script. So far it has been tested on Kerbin and Duna. It does include parts from mods, including:

- [KOS](https://forum.kerbalspaceprogram.com/index.php?/topic/165628-ksp-1101-and-111-kos-v1310-kos-scriptable-autopilot-system/) - What this script actually runs on.
- [Near Future Solar](https://forum.kerbalspaceprogram.com/index.php?/topic/155465-most-112x-near-future-technologies-august-26/) - This provides the high output solar panel to power it with good sunlight.
- [Near Future eXploration](https://forum.kerbalspaceprogram.com/index.php?/topic/155465-most-112x-near-future-technologies-august-26/) - Nano landing legs, as well as some tiny antennas. 
- [Hullcam VDS](http://forum.kerbalspaceprogram.com/index.php?/topic/145633-113-hullcam-vds-continued/&do=findComment&comment=2710247) - Cameras for porbe views.
- [LaserDist](http://forum.kerbalspaceprogram.com/index.php?/topic/141697-*) - A laser rangefinder. Not currently used in this script (although there is some code written for it), but in the future may be used to help scan for suitable landing spots. 

Appart from KOS, MODs aren't strictly nessecary for this Quadcopter to work. (i.e. the Near Future Solar cells could be replaced with stock solar cells, or RTGs. However, note that the rotors do consume a fair amount of power, and without a good power source the battery will quickly deplete)

## How it works. 

Copy all files (Except the `Ship` folder!) to the Quadcopter. You can use the handy `update.ks` script to easily do this (or even install it as the bootloader).

The simplest way to get up and running is to create a waypoint (e.g. using scansat, or other mods that allow creating waypoints), activate navigation and then run "Quadcopter.ks". This will run a default mission where the Quadcopter will takeoff, fly to the waypoint, and then land. (Note: Make sure the selected landing spot is relatively flat. The autopilot currenlty only searches for flat landing spots when in emergency landing mode, otherwise it relies on the user to select appropriate landing coordinates.)

More complex missions can be created using lists of lexicon commands, wich are stored in the Missions directory. There are several examples of a variety of common missions.

Missions in the mission folder should be run from the base directory (e.g. `run "Missions/science_mission.ks".`)

## The Comms Manager

Not strictly related to the Quadcopter flight, and not required, there is a Comms Manager library included. This controls antennas tagged `auto_antenna` by automatically activating/deactivating and pointing towards the closest comms satellite above the horizon. This can be confiugred in the parameters file. Set `use_comms_manager` to False if you don't want to use this function. 

## Future Development

There is a fair amount of cleanup and documentation still required. The program is relativley complete and functional, however a few features are still planned:
- Use velocity limits in the "fly" mission command
- Better altitude control (i.e. selectable AGL vs ASL altitude. Right now the script terrain-hugs (i.e. AGL)).
- Smart Landing to search for suitable landing spots near waypoints. The code to do this exists for emergecy landing, so it should be relatively simple to include in regular missions. 
- Laser rangefinder integration. This will help when landing near objects not represented by terrain height (i.e. other ships)

There is no guarantee if/when these features will be implemented. 
There is no guarantee for backwards compatibility for mission files as new features are added. 

## Libraries from other contributors.

This script uses [lib_navball.ks](https://github.com/KSP-KOS/KSLib/blob/master/library/lib_navball.ks) from [KSLib](https://github.com/KSP-KOS/KSLib), provided under the MIT license. 

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>. 