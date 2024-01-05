import serial_com

def headingchangeFn(heading, endheading_from_home, accelerometer, zeroHeading):
	if(zeroHeading >=260):
		headingOverflow = 360-zeroHeading
		if(endheading_from_home >= 0 and endheading_from_home <= headingOverflow):
			endheading_from_home+=360
	if(zeroHeading <=100):
		headingOverflow = 360+zeroHeading-100
		if(endheading_from_home<=360 and endheading_from_home >= headingOverflow):
			endheading_from_home-=360

	headingdiff = (int(zeroHeading) - int(endheading_from_home))*(-1)
	pwm_headingchange = (int(zeroHeading) - int(heading))*(-1)
	#print(zeroHeading, endheading_from_home, pwm_headingchange, headingOverflow, headingdiff)
	if(abs(headingdiff)>=100 and abs(headingdiff)<=180):
		if(headingdiff<0):
			pwm_current_estimate = 1250
			pwm_freq = 1250
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
		elif(headingdiff>=0):
			pwm_current_estimate = 8750
			pwm_freq = 8750
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
	elif(abs(headingdiff)>=180):
		if(headingdiff>0):
			pwm_current_estimate = 1250
			pwm_freq = 1250
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
		elif(headingdiff<=0):
			pwm_current_estimate = 8750
			pwm_freq = 8750
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
	elif(headingdiff<0):
		pwm_current_estimate = 5000 - int(3750/100*pwm_headingchange)
		pwm_freq = 5000 - int(3750/100*abs(headingdiff))
		serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
	elif(headingdiff>=0):
		pwm_current_estimate = 5000 + int(3750/100*pwm_headingchange)
		pwm_freq = 5000 + int(3750/100*abs(headingdiff))
		serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
	#print(headingchange)
	if(accelerometer):
		print("b")
	new_heading = endheading_from_home
	return new_heading

def anglechangeFn(angle, end_angle, accelerometer):
	if(accelerometer):
		print("b")
	else:
		if(end_angle <= 90):
			pwm_current_estimate = 2000 + int(3400/90*angle)
			pwm_freq = 2000 + int(3400/90*end_angle) #2000 - 0degrees
			serial_com.setVerticalServo(pwm_freq, pwm_current_estimate)
		else:
			pwm_freq = 5400
			serial_com.setVerticalServo(pwm_freq, pwm_freq)
	new_angle = end_angle
	return new_angle

def headingChangeFailsafe(heading, homeheading):
	headingdiff = int(homeheading) - int(heading)
	if(abs(headingdiff)>=100):
		if(headingdiff<0):
			pwm_freq = 1250
			serial_com.setHorizontalServo(pwm_freq, pwm_freq)
		elif(headingdiff>=0):
			pwm_freq = 8750
			serial_com.setHorizontalServo(pwm_freq, pwm_freq)
	elif(headingdiff<0):
		pwm_freq = 5000 - int((3750/100)*abs(headingdiff))
		serial_com.setHorizontalServo(pwm_freq, pwm_freq)
	elif(headingdiff>=0):
		pwm_freq = 5000 + int((3750/100)*abs(headingdiff))
		serial_com.setHorizontalServo(pwm_freq, pwm_freq)
	return

def angleChangeFailsafe(angle):
	if(angle <= 90):
			pwm_freq = 2000 + int(3400/90*angle) #2000 - 0degrees
			serial_com.setVerticalServo(pwm_freq, pwm_freq)
	else:
		pwm_freq = 5400
		serial_com.setVerticalServo(pwm_freq, pwm_freq)
	return