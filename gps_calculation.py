import math
import initialize_data

'''
Vincenty's formulae for GPS distance and heading calculation - https://en.wikipedia.org/wiki/Vincenty%27s_formulae
#Using Inverse problem to calculate ellipsiodal distance between two points on the earths surface and the heading from the first point to the second point
'''

def alternate_calc_gps_distance(lat_home, lon_home, lat_drone, lon_drone, heading_current, angle_current, drone_alt):
	lat1 = (float(lat_home)*math.pi)/180 #Home latitude
	lat2 = (float(lat_drone)*math.pi)/180 #Drone latitude
	lon1 = (float(lon_home)*math.pi)/180 #Home longitude
	lon2 = (float(lon_drone)*math.pi)/180 #Drone longitude

	#Constants for the formula
	a = 6378137.0 # length of semi-major axis of the ellipsoid (radius at equator)
	f = 1/298.257223563 #  	flattening of the ellipsoid
	b = 6356752.314245 # length of semi-minor axis of the ellipsoid (radius at the poles)
	u1 = math.atan((1-f)*math.tan(lat1)) #reduced latitude of the first point (latitude on the auxiliary sphere)
	u2 = math.atan((1-f)*math.tan(lat2)) #redused latitude of the second point (latitude on the auxiliary sphere)
	deltaLon = lon2-lon1 #difference in longitude between the two points
	λ = deltaLon #Initial value for λ

	#First part of the inverse problem calculation of λ until it converges to 1e-11
	while(True): 
		sinSigma = math.sqrt(((math.cos(u2)*math.sin(λ))**2)+(((math.cos(u1)*math.sin(u2))-(math.sin(u1)*math.cos(u2)*math.cos(λ)))**2))
		cosSigma = math.sin(u1)*math.sin(u2) + math.cos(u1)*math.cos(u2)*math.cos(λ)
		sigma = math.atan2(sinSigma, cosSigma)
		if(sinSigma == 0): #Used to prevent division with 0
			sinSigma = 1e-15
		sinAlfa = (math.cos(u1)*math.cos(u2)*math.sin(λ))/sinSigma
		cosSqrAlfa = (1 - (sinAlfa**2))
		if(cosSqrAlfa == 0): #Used to prevent division with 0
			cosSqrAlfa = 1e-15
		cos2SigmaM = cosSigma - ((2*math.sin(u1)*math.sin(u2))/cosSqrAlfa)
		bigC = (f/16)*cosSqrAlfa * (4+(f*(4-(3*cosSqrAlfa))))
		saveλ = λ
		λ = deltaLon + (((1-bigC)*f*sinAlfa)*(sigma + ((bigC*sinSigma) *(cos2SigmaM + (bigC*cosSigma*(-1+(2*(cos2SigmaM*cos2SigmaM))))))))
		if(abs(λ - saveλ)< 1e-11): #End of loop condition
			break

	#Second part of the inverse problem.
	uSqr = cosSqrAlfa * ((a**2) - (b**2))/(b**2)
	bigA = 1 + ((uSqr/16384)*(4096 + (uSqr *(-768 + (uSqr * (320 - (175*uSqr)))))))
	bigB = (uSqr/1024)*(256+(uSqr*(-128 + uSqr*(74-(47*uSqr)))))
	deltaSigma = bigB * sinSigma*(cos2SigmaM + ((1/4)*bigB))*(cosSigma*(-1 + (2* (cos2SigmaM**2))) - ((bigB/6)*cos2SigmaM)*(-3 + ((4* (sinSigma**2)) * (-3 + (4*(cos2SigmaM**2))))))
	s = b*bigA *(sigma-deltaSigma) #Ellipsoidal distance
	s = float(int(s*10000)/10000) #Rounds the distance to 4 decimal places
	alfa1 = math.atan2(math.cos(u2)*math.sin(λ), math.cos(u1)*math.sin(u2)-math.sin(u1)*math.cos(u2)*math.cos(λ))
	alfa1 = math.degrees(alfa1)
	alfa1 = float(int(alfa1*100)/100) #Rounds the heading to 2 decimal places
	if(alfa1 < 0): #Converts the heading to 0-360 degrees
		alfa1+=360
	
	newheading_from_home = alfa1 #Heading from home to drone
	direct_distance = math.sqrt(drone_alt*drone_alt + s*s) #Direct distance from home to drone using pythagoras theorem with altitude and ellipsoidal distance
	if((s>0 and drone_alt >5) or (s>=10 and drone_alt >=0)): #Close distance between drone and home can create division by 0 or unpredicted behaviour
		new_angle = (math.asin(drone_alt/direct_distance)*180)/math.pi
	else:
		new_angle = 10 #Default angle if distance is too small
	if(new_angle <= 5): #If the calculated angle is below 5 degrees sets it to 5
		new_angle = 5
	new_angle = float(int(new_angle*1000)/1000) #Rounds the angle to 3 decimal places
	direct_distance = float(int(direct_distance*1000)/1000) #Rounds the distance to 3 decimal places
	if(initialize_data.debug): #Debug print if enabled
		print("Direct distance: ", str(direct_distance) + " Newheading from home: " + str(newheading_from_home) + " New angle: " + str(new_angle) + " Ground distance: " + str(s)) 
	return direct_distance, newheading_from_home, new_angle

