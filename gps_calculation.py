import math
heading_zero = 306
lat1 = 56.9168693*math.pi/180 #Home latitude
lat2 = 56.9167429*math.pi/180
lon1 = 24.3175778*math.pi/180 #Home longitudeb
lon2 = 24.3177747*math.pi/180
relative_alt = 5
#Haversine Formula - great-circle distance between two points on a sphere from longitudes and latitudes
a = (math.sin((lat2-lat1)/2) ** 2) + math.cos(lat1) * math.cos(lat2) * (math.sin((lon2 - lon1)/2) ** 2)
c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
d = 6371000 * c

y = math.sin(lon2-lat1) * math.cos(lat2)
x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)
o = math.atan2(y,x)
heading_from_home = (o*180/math.pi + 360)%360
headingchange = heading_zero - heading_from_home
direct_distance = math.sqrt(relative_alt*relative_alt + d*d)
vert_angle = math.sin(relative_alt/direct_distance)*180/math.pi
print(heading_from_home, 'heading from home')
print(headingchange, 'relative heading change(+ right, - left)')
print(direct_distance, 'Direct distance')
print(vert_angle, 'Vertical angle')
print(d, 'meters from home')
