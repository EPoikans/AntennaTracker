import math

def calc_gps_distance(lat_home, lon_home, lat_drone, lon_drone, heading_current, angle_current, drone_alt):
	lat1 = float(lat_home)*math.pi/180 #Home latitude
	lat2 = float(lat_drone)*math.pi/180
	lon1 = float(lon_home)*math.pi/180 #Home longitudeb
	lon2 = float(lon_drone)*math.pi/180
	#Haversine Formula - great-circle distance between two points on a sphere from longitudes and latitudes
	a = (math.sin((lat2-lat1)/2) ** 2) + math.cos(lat1) * math.cos(lat2) * (math.sin((lon2 - lon1)/2) ** 2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	d = 6371000 * c #Distance

	y = math.sin(lon2-lat1) * math.cos(lat2)
	x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)
	o = math.atan2(y,x)
	newheading_from_home = (o*180/math.pi + 360)%360
	direct_distance = math.sqrt(drone_alt*drone_alt + d*d)
	if(d>=5 and drone_alt >=3):
		new_angle = math.sin(drone_alt/direct_distance)*180/math.pi
	elif(drone_alt>=10 and d<=5):
		new_angle = math.sin(drone_alt/2)*180/math.pi
	elif(d<=5 and drone_alt<=3):
		newheading_from_home = heading_current
		new_angle = angle_current
	else:
		new_angle = 10
	if(new_angle <= 5):
		new_angle = 5
	newheading_from_home = float(int(newheading_from_home*100)/100)
	new_angle = float(int(new_angle*1000)/1000)
	direct_distance = float(int(direct_distance*1000)/1000)
	return direct_distance, newheading_from_home, new_angle

#Vincenty's formulae for GPS distance and heading calculation

def alternate_calc_gps_distance(lat_home, lon_home, lat_drone, lon_drone, heading_current, angle_current, drone_alt):
	lat1 = (float(lat_home)*math.pi)/180 #Home latitude
	lat2 = (float(lat_drone)*math.pi)/180
	lon1 = (float(lon_home)*math.pi)/180 #Home longitudeb
	lon2 = (float(lon_drone)*math.pi)/180
	a = 6378137.0 # length of semi-major axis of the ellipsoid (radius at equator)
	f = 1/298.257223563 #  	flattening of the ellipsoid
	b = 6356752.314245 # length of semi-minor axis of the ellipsoid (radius at the poles)
	u1 = math.atan((1-f)*math.tan(lat1)) #reduced latitude (latitude on the auxiliary sphere)
	u2 = math.atan((1-f)*math.tan(lat2))
	deltaLon = lon2-lon1
	λ = deltaLon
	while(True):
		sinSigma = math.sqrt(((math.cos(u2)*math.sin(λ))**2)+(((math.cos(u1)*math.sin(u2))-(math.sin(u1)*math.cos(u2)*math.cos(λ)))**2))
		cosSigma = math.sin(u1)*math.sin(u2) + math.cos(u1)*math.cos(u2)*math.cos(λ)
		sigma = math.atan2(sinSigma, cosSigma)
		if(sinSigma == 0):
			sinSigma = 1e-15
		sinAlfa = (math.cos(u1)*math.cos(u2)*math.sin(λ))/sinSigma
		cosSqrAlfa = (1 - (sinAlfa**2))
		cos2SigmaM = cosSigma - ((2*math.sin(u1)*math.sin(u2))/cosSqrAlfa)
		bigC = (f/16)*cosSqrAlfa * (4+(f*(4-(3*cosSqrAlfa))))
		saveλ = λ
		λ = deltaLon + (((1-bigC)*f*sinAlfa)*(sigma + ((bigC*sinSigma) *(cos2SigmaM + (bigC*cosSigma*(-1+(2*(cos2SigmaM*cos2SigmaM))))))))
		if(abs(λ - saveλ)< 1e-11):
			break
	uSqr = cosSqrAlfa * ((a**2) - (b**2))/(b**2)
	bigA = 1 + ((uSqr/16384)*(4096 + (uSqr *(-768 + (uSqr * (320 - (175*uSqr)))))))
	bigB = (uSqr/1024)*(256+(uSqr*(-128 + uSqr*(74-(47*uSqr)))))
	deltaSigma = bigB * sinSigma*(cos2SigmaM + ((1/4)*bigB))*(cosSigma*(-1 + (2* (cos2SigmaM**2))) - ((bigB/6)*cos2SigmaM)*(-3 + ((4* (sinSigma**2)) * (-3 + (4*(cos2SigmaM**2))))))
	s = b*bigA *(sigma-deltaSigma) #Ellipsoidal distance
	s = float(int(s*10000)/10000)
	alfa1 = math.atan2(math.cos(u2)*math.sin(λ), math.cos(u1)*math.sin(u2)-math.sin(u1)*math.cos(u2)*math.cos(λ))
	alfa1 = math.degrees(alfa1)
	alfa1 = float(int(alfa1*100)/100)
	if(alfa1 < 0):
		alfa1+=360
	newheading_from_home = alfa1
	#alfa2 = math.atan2(math.cos(u1)*math.sin(λ), -math.sin(u1)*math.cos(u2)+math.cos(u1)*math.sin(u2)*math.cos(λ))
	#alfa2 = math.degrees(alfa2)
	direct_distance = math.sqrt(drone_alt*drone_alt + s*s)
	if(s>=5 and drone_alt >=3):
		new_angle = math.sin(drone_alt/direct_distance)*180/math.pi
	elif(drone_alt>=10 and s<=5):
		new_angle = math.sin(drone_alt/2)*180/math.pi
	elif(s<=5 and drone_alt<=3):
		newheading_from_home = heading_current
		new_angle = angle_current
	else:
		new_angle = 10
	if(new_angle <= 5):
		new_angle = 5
	new_angle = float(int(new_angle*1000)/1000)
	direct_distance = float(int(direct_distance*1000)/1000)
	return direct_distance, newheading_from_home, new_angle

print(calc_gps_distance(56.916714, 24.317700, 56.916711, 24.317302, 240, 30, 15))
print(alternate_calc_gps_distance(56.916714, 24.317700, 56.916711, 24.317302, 240, 30, 15))
print(calc_gps_distance(56.916714, 24.317700, 56.916726,24.317554, 240, 30, 15))
print(alternate_calc_gps_distance(56.916714, 24.317700, 56.916726,24.317554, 240, 30, 15))
print(calc_gps_distance(56.916714, 24.317700, 56.916694,24.316534, 240, 30, 15))
print(alternate_calc_gps_distance(56.916714, 24.317700, 56.916694,24.316534, 240, 30, 15))
print(calc_gps_distance(56.916714, 24.317700, 56.916730, 24.317675, 240, 30, 15))
print(alternate_calc_gps_distance(56.916714, 24.317700, 56.916730, 24.317675, 240, 30, 15))
print(calc_gps_distance(56.916714, 24.317700, 56.916712, 24.317710, 240, 30, 15))
print(alternate_calc_gps_distance(56.916714, 24.317700, 56.916712, 24.317710, 240, 30, 15))
print(calc_gps_distance(56.916714, 24.317700, 56.917370, 24.318769, 240, 30, 15))
print(alternate_calc_gps_distance(56.916714, 24.317700, 56.917370, 24.318769, 240, 30, 15))