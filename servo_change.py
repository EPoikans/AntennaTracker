import serial_com
import initialize_data

global pwm_current_estimate
pwm_current_estimate = 4900

def headingchangeFn(heading, endheading_from_home, accelerometer, zeroHeading): #4900 - mid, 1200 -115deg, 8600 - +115deg
	global pwm_current_estimate 
	
	if(zeroHeading >=245):
		headingOverflow = 360-zeroHeading
		if(endheading_from_home >= 0 and endheading_from_home <= headingOverflow):
			endheading_from_home+=360
	if(zeroHeading <=115):
		headingOverflow = 360+zeroHeading-100
		if(endheading_from_home<=360 and endheading_from_home >= headingOverflow):
			endheading_from_home-=360
	headingdiff = (int(zeroHeading) - int(endheading_from_home))*(-1)
	if(initialize_data.debug):
		headingOverflow = locals().get('headingOverflow', 0)
		print(str(headingdiff) + '- headingdiff ' + str(endheading_from_home) + '- endheading_from_home ' + str(zeroHeading) + ' - zeroHeading ' + str(headingOverflow) + ' - headingOverflow')
	if(abs(headingdiff)>=115 and abs(headingdiff)<=190):
		if(headingdiff>0):
			pwm_freq = 1200
			if(initialize_data.debug):
				print(str(pwm_current_estimate) + ' current ' + str(pwm_freq) + ' pwm 1')
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
			pwm_current_estimate = pwm_freq
		elif(headingdiff<=0):
			pwm_freq = 8600
			if(initialize_data.debug):
				print(str(pwm_current_estimate) + ' current ' + str(pwm_freq) + ' pwm 2')
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
			pwm_current_estimate = pwm_freq
	elif(abs(headingdiff)>=190):
		if(headingdiff<0):
			pwm_freq = 1200
			if(initialize_data.debug):
				print(str(pwm_current_estimate) + ' current ' + str(pwm_freq) + ' pwm 3')
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
			pwm_current_estimate = pwm_freq
		elif(headingdiff>=0):
			pwm_freq = 8600
			if(initialize_data.debug):
				print(str(pwm_current_estimate) + ' current ' + str(pwm_freq) + ' pwm 4')
			serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
			pwm_current_estimate = pwm_freq
	elif(headingdiff>0):
		pwm_freq = 4900 - int(3700/115*abs(headingdiff))
		if(initialize_data.debug):
			print(str(pwm_current_estimate) + ' current ' + str(pwm_freq) + ' pwm 5')
		serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
		pwm_current_estimate = pwm_freq
	elif(headingdiff<=0):
		pwm_freq = 4900 + int(3700/115*abs(headingdiff))
		if(initialize_data.debug):
			print(str(pwm_current_estimate) + ' current ' + str(pwm_freq) + ' pwm 6')
		serial_com.setHorizontalServo(pwm_freq, pwm_current_estimate)
		pwm_current_estimate = pwm_freq
	new_heading = endheading_from_home
	return new_heading

def anglechangeFn(angle, end_angle, accelerometer):
	if(end_angle <= 90):
		pwm_current_estimate = 2000 + int(3400/90*angle)
		pwm_freq = 2000 + int(3400/90*end_angle) #2000 - 0degrees
		if(initialize_data.debug):
			print(str(pwm_current_estimate) + ' - current ' + str(pwm_freq) + ' - pwm')
		serial_com.setVerticalServo(pwm_freq, pwm_freq)
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