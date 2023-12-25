def headingchangeFn(heading, endheading_from_home, accelerometer):
	headingchange = endheading_from_home - heading
	#print(headingchange)
	if(accelerometer):
		print("b")
	new_heading = endheading_from_home
	return new_heading

def anglechangeFn(angle, end_angle, accelerometer):
	anglechange = end_angle - angle
	#print(anglechange)
	if(accelerometer):
		print("b")
	new_angle = end_angle
	return new_angle