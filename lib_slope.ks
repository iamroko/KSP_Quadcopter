


function get_position {
    //R = 6378.1 #Radius of the Earth
    //brng = 1.57 #Bearing is 90 degrees converted to radians.
    //d = 15 #Distance in km

    //#lat2  52.20444 - the lat result I'm hoping for
    //#lon2  0.36056 - the long result I'm hoping for.

    //lat1 = math.radians(52.20472) #Current lat point converted to radians
    //lon1 = math.radians(0.14056) #Current long point converted to radians

    //lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
    //    math.cos(lat1)*math.sin(d/R)*math.cos(brng))

    //lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
     //           math.cos(d/R)-math.sin(lat1)*math.sin(lat2))

    //lat2 = math.degrees(lat2)
    //lon2 = math.degrees(lon2)

        //local distance is vang(A:position-body:position,B:position-body:position)*Constant:DegToRad*body:radius.

    local parameter origin. // A geoposition. 
    local parameter range.
    local parameter bearing.

    local current_lat is origin:lat*Constant:DegtoRad.
    local current_lng is origin:lng*Constant:DegtoRad.

    local lat2 is arcSin(sin(origin:lat)*cos(range/body:radius*Constant:RadtoDeg) + cos(origin:lat)*sin(range/body:radius*Constant:RadtoDeg)*cos(bearing)).
    local lon2 is origin:lng + arctan2(sin(bearing)*sin(range/body:radius*Constant:RadtoDeg)*cos(origin:lat), cos(range/body:radius*Constant:RadtoDeg)-sin(origin:lat)*sin(lat2)).

    return latlng(lat2, lon2).

    }

FUNCTION slope_calculation {//returns the slope of p1 in degrees
	PARAMETER p1.
	LOCAL upVec IS (p1:POSITION - p1:BODY:POSITION):NORMALIZED.
	RETURN VANG(upVec,surface_normal(p1)).
}

FUNCTION surface_normal {
	PARAMETER p1.
	LOCAL localBody IS p1:BODY.
	LOCAL basePos IS p1:POSITION.

	LOCAL upVec IS (basePos - localBody:POSITION):NORMALIZED.
	LOCAL northVec IS VXCL(upVec,LATLNG(90,0):POSITION - basePos):NORMALIZED * 3.
	LOCAL sideVec IS VCRS(upVec,northVec):NORMALIZED * 3.//is east

	LOCAL aPos IS localBody:GEOPOSITIONOF(basePos - northVec + sideVec):POSITION - basePos.
	LOCAL bPos IS localBody:GEOPOSITIONOF(basePos - northVec - sideVec):POSITION - basePos.
	LOCAL cPos IS localBody:GEOPOSITIONOF(basePos + northVec):POSITION - basePos.
	RETURN VCRS((aPos - cPos),(bPos - cPos)):NORMALIZED.
}


// To Do: include a check to see if it's water. 
function search_slope {
  local parameter current_location.
  local parameter max_range is 500. // Only search up to here. 
  local parameter range_step is 25. // range step in meters
  local parameter angle_step is 15.  // radial step size in degrees. 
  local parameter threshold is 0.5. // Minimum value.. If supplied, will return first result below this. 

  local X is 0.
  local best_point is list(0,100).

    until X >= max_range {
    
        local Y is 0.

        until Y >= 360 {
            set sample_position to get_position(current_location, X, Y).
            set sample_point to slope_calculation(sample_position).
            if sample_point < best_point[1] {
                set best_point[0] to sample_position.
                set best_point[1] to sample_point.
                if best_point[1] <= threshold {
                    return best_point.
                }
            }
            //Scan all points. Find minimum :slope 
            set Y to Y + angle_step.
        }

        set X to X + range_step.
    }

    return best_point.
}


function laser_search {
    local parameter x_range is 15.
    local parameter y_range is 15.

    return ship:geoposition.

}

// Random Debugging stuff:
//print(search_slope(latlng(-0.0520300148639656, -75.0091002719728))).
//print("Coarse Searching").
//set coarse_search to search_slope(latlng(-0.0520300148639656, -75.0091002719728), 500, 50, 45, 5).
//print(coarse_search).
//print("Fine Searching").
//print(search_slope(coarse_search[0], 100, 20, 30, 0.5)).
//print(addons).
//print(addons:scansat:slope(ship:body, get_position(ship:geoposition, 1, 00))).
//print(slope_calculation(get_position(ship:geoposition, 2850, 270))).