import serial_com

def headingchangeFn(heading, endheading_from_home, accelerometer, zeroHeading):
	headingdiff = int(zeroHeading) - int(endheading_from_home)
	pwm_headingchange = int(zeroHeading) - int(heading)
	if(abs(headingdiff)>=100):
		if(headingdiff<0):
			pwm_current_estimate = 1250
			pwm_freq = 1250
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
		elif(headingdiff>=0):
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