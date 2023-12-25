import math

def calc_gps_distance(lat_home, lon_home, lat_drone, lon_drone, heading_current, angle_current, drone_alt):
	lat1 = lat_home*math.pi/180 #Home latitude
	lat2 = lat_drone*math.pi/180
	lon1 = lon_home*math.pi/180 #Home longitudeb
	lon2 = lon_drone*math.pi/180
	#Haversine Formula - great-circle distance between two points on a sphere from longitudes and latitudes
	a = (math.sin((lat2-lat1)/2) ** 2) + math.cos(lat1) * math.cos(lat2) * (math.sin((lon2 - lon1)/2) ** 2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	d = 6371000 * c

	y = math.sin(lon2-lat1) * math.cos(lat2)
	x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)
	o = math.atan2(y,x)
	newheading_from_home = (o*180/math.pi + 360)%360
	direct_distance = math.sqrt(drone_alt*drone_alt + d*d)
	if(direct_distance>=5 and drone_alt >=10):
		new_angle = math.sin(drone_alt/direct_distance)*180/math.pi
	elif(drone_alt>=10 and direct_distance<=5):
		new_angle = math.sin(drone_alt/2)*180/math.pi
	elif(direct_distance<=5 and drone_alt<=10):
		newheading_from_home = heading_current
		new_angle = angle_current
	else:
		new_angle = 20
	if(new_angle <= 15):
		new_angle = 15
	return direct_distance, newheading_from_home, new_angle