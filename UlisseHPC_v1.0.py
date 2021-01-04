###########################################
#                             	          #
#  B U L U N     S K U N K    W O R K S   #
#                                         #
#            welcome you to               #
#                                         #
#                 ULISSE                  #
#           Universal Lander              #
#                   for                   #
#    InterStellar Space Exploration       #
#                                         #
#	UlisseHPCv1.0: MPI - Bocking	  #
#					  #
#	"The Lord of the Processes"	  #
#       "One Process to rule them all"	  #
#       "and in the darkness bind them"	  #
#                                         #
###########################################

#
# Import MPI library 
#

from mpi4py import MPI
import sys

#
# Import the necessary packages
#


from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import datetime
import curses
import PiMotor
import RPi.GPIO as GPIO
import Adafruit_DHT
import lcddriver
import ConfigParser
import serial
import string
import pynmea2
import smbus
import py_qmc5883l

GPIO.setmode(GPIO.BOARD)

#
# Initialize Motors
#

#Name of Individual MOTORS
m2 = PiMotor.Motor("MOTOR2",1)
m1 = PiMotor.Motor("MOTOR1",1)

#To drive all motors together
motorAll = PiMotor.LinkedMotors(m1,m2)

#########################################
#                                       #
#	DEFINE DISTANCE FUNCTION	#
#					#
#########################################

def distance( TRIG, ECHO ):
  pulse_start = 0
  pulse_end = 0
  start_while = 0

  GPIO.output(TRIG, False)
  GPIO.output(TRIG, True)
  time.sleep(0.00001)
  GPIO.output(TRIG, False)

  start_while = time.time()

  while ( GPIO.input(ECHO)==0 and ( time.time() - start_while ) < 0.5):
    pulse_start = time.time()

  while GPIO.input(ECHO)==1:
    pulse_end = time.time()

  pulse_duration = pulse_end - pulse_start

  cm = round( pulse_duration * 17150, 1 )

  return cm
 

#########################################
#                                       #
#       DEFINE MOVE FUNCTION            #
#                                       #
#########################################

def move( arrow_flash, direction, throttle, rotate_throttle, fwd_time,  rotate_time, dbg_cfg ):
  loop_time = 0
  start_time = time.time()

#  print( "Direction = ", direction )

  if( dbg_cfg == "full" ):
    print( "Direction = ", direction )
    print( "Throttle = ", throttle, "    - (F) Faster +5% - (S) Slower -5%" )
    print( "Rotate_throttle = ", rotate_throttle, " - (L) Faster +5% - (R) Slower -5%" )
    print( "FWD time = ", fwd_time )
    print( "Rotate time = ", rotate_time )
    print( "\n" )

  m2.stop()
  m1.stop()

  if direction == "RIGHT":
    throttle = rotate_throttle 

    while loop_time < rotate_time:
      loop_time = time.time() - start_time
 
      m2.forward(throttle)
      m1.reverse(20)

#      time.sleep( arrow_flash )

  elif direction == "LEFT":
    throttle = rotate_throttle 

    while loop_time < rotate_time:
      loop_time = time.time() - start_time 

      m2.reverse(30)
      m1.forward(throttle)

#      time.sleep(arrow_flash)

  elif direction == "FRONT":
    while loop_time < fwd_time:
      loop_time = time.time() - start_time 

      motorAll.reverse(throttle)

#      time.sleep(arrow_flash)

  elif direction == "BACK":
    while loop_time < fwd_time:
      loop_time = time.time() - start_time 

      motorAll.forward(throttle)

#      time.sleep(arrow_flash)

  elif direction == "HALT":
    m2.stop()
    m1.stop()

    motorAll.stop()

#########################################
#					#
#	DEFINE FRAME_CAPTURE FUNCTION 	#
#					#
#########################################

def frame_capture( args, vs ):

#
# grab the current frame
#

  frame = vs.read()

#
# handle the frame from VideoCapture or VideoStream
#

  frame = frame[1] if args.get("video", False) else frame

#
# if we are viewing a video and we did not grab a frame, then we have reached the end of the video
#

  return frame

#################################
#				#
#    DEFINE WRITE FUNCTION	#
#				#
#################################

def write( frame, xcoord, ycoord, text):
  cv2.putText(frame, text, (xcoord, ycoord ), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)


##################################
#				 #
#	DEFINE SETANGLE FUNCTION #
#				 #
##################################

def SetAngle(pwm_servo, servo_pin, angle):
  duty = angle / 18 + 2

  GPIO.output(servo_pin, True)
  pwm_servo.ChangeDutyCycle(duty)

  time.sleep(0.5)

  GPIO.output(servo_pin, False)

  pwm_servo.ChangeDutyCycle(0)


#################################
#				#
#	DEFINE LCD FUNCTION	#
#				#
#################################

def lcd( display, string, line ):

  display.lcd_display_string(string, line)

#################################
#				#
# 	DEFINE BEARING FUNCTION #
#				#
#################################

def bearing( sensor, dbg_cfg ):
  sensor = py_qmc5883l.QMC5883L()
  sensor.declination = 10.02
  s = sensor.get_bearing()
  m = sensor.get_magnet()

  if( dbg_cfg == "full" ):
    print( "\n Bearing = ", s, " Magnet = ", m )

  return s

#################################
#				#
#	DEFINE XLR8 Function	#
#				#
#################################

def xlr8( dbg_cfg ):

  accl = [0,0,0,0,0,0]
  xconv = 3.5e-3
  yconv = 3.9e-3
  zconv = 4.3e-3

# Get I2C bus
  bus = smbus.SMBus(1)

# ADXL345 address, 0x53(83)
# Select bandwidth rate register, 0x2C(44)
#               0x0A(10)        Normal mode, Output data rate = 100 Hz

  try:
    bus.write_byte_data(0x53, 0x2C, 0x0A)
  except:
    print("Read Accelerator data failed!!! Try again")
    return

# ADXL345 address, 0x53(83)
# Select power control register, 0x2D(45)
#               0x08(08)        Auto Sleep disable

  try:
    bus.write_byte_data(0x53, 0x2D, 0x08)
  except:
    print("Read Accelerator data failed!!! Try again")
    return

# ADXL345 address, 0x53(83)
# Select data format register, 0x31(49)
#               0x08(08)        Self test disabled, 4-wire interface
#                                       Full resolution, Range = +/-2g

  bus.write_byte_data(0x53, 0x31, 0x08)

  time.sleep(0.5)

# ADXL345 address, 0x53(83)
# Read data back from 0x32(50), 2 bytes
# X-Axis LSB, X-Axis MSB

  try:
    data0 = bus.read_byte_data(0x53, 0x32)
    data1 = bus.read_byte_data(0x53, 0x33)
  except:
    print("Read Accelerator data failed!!! Try again")
    return accl

# Convert the data to 10-bits
  xAccl = ((data1 & 0x03) * 256) + data0

  if xAccl > 511 :
        xAccl -= 1024

# ADXL345 address, 0x53(83)
# Read data back from 0x34(52), 2 bytes
# Y-Axis LSB, Y-Axis MSB

  data0 = bus.read_byte_data(0x53, 0x34)
  data1 = bus.read_byte_data(0x53, 0x35)

# Convert the data to 10-bits

  yAccl = ((data1 & 0x03) * 256) + data0

  if yAccl > 511 :
        yAccl -= 1024

# ADXL345 address, 0x53(83)
# Read data back from 0x36(54), 2 bytes
# Z-Axis LSB, Z-Axis MSB

  data0 = bus.read_byte_data(0x53, 0x36)
  data1 = bus.read_byte_data(0x53, 0x37)

# Convert the data to 10-bits

  zAccl = ((data1 & 0x03) * 256) + data0

  if zAccl > 511 :
        zAccl -= 1024

  accl[0] = xAccl * xconv
  accl[1] = yAccl * yconv
  accl[2] = zAccl * zconv
  accl[3] = xAccl
  accl[4] = yAccl
  accl[5] = zAccl

  if(dbg_cfg == "full" ):
    print ("xAccl = ", xAccl, "   xG = ", round(accl[0],2) )
    print ("yAccl = ", yAccl, "   yG = ", round(accl[1],2) )
    print ("zAccl = ", zAccl, "   zG = ", round(accl[2],2) )

  return accl

#########################################
#					#
#	DEFINE SHIFT_REG FUNCTION	#
#					#
#########################################

def shift_reg( dbg_cfg, SER, SCK, RCK ):
  sleep_time = 0.1

  for y in range(8):  # Schleife wird 8Mal (8bit) ausgefuehrt
          if( dbg_cfg == "full" ):
            outputfile.write("Worker ID: " + str( mpi_rank ) + " - First loop through the pins: " + str(y) +"\n" )

          GPIO.output(SER,1)  
          time.sleep( sleep_time )
          GPIO.output(SCK,1)  
          time.sleep( sleep_time )
          GPIO.output(SCK,0)  
          GPIO.output(SER,0)
          GPIO.output(RCK,1)
          time.sleep( sleep_time ) 
          GPIO.output(RCK,0) 

  for y in range(8):  
          if( dbg_cfg == "full" ):
            outputfile.write("Worker ID: " + str( mpi_rank ) + " - Second loop through the pins: " + str(y) + "\n"  )

          GPIO.output(SER,0)  
          time.sleep( sleep_time )
          GPIO.output(SCK,1)
          time.sleep( sleep_time )
          GPIO.output(SCK,0) 
          GPIO.output(SER,0)
          GPIO.output(RCK,1)
          time.sleep( sleep_time ) 
          GPIO.output(RCK,0)



#################################
#				#
#	DEFINE WELCOME FUNCTION #
#				#
#################################

def welcome( display ):
  display.lcd_clear()
  lcd( display, "Bulun SkunkWorks", 1)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, " welcome you to", 1)
  time.sleep(2)
  lcd(display, "U.L.I.S.S.E. HPC", 2)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, "Universal", 1)
  lcd(display, "Lander", 2)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, "for", 1)
  lcd(display, "Interstellar", 2)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, "Space", 1)
  lcd(display, "Exploration", 2)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, "MU/TH/UR 6000", 1)
  lcd(display, "Powered", 2)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, "The Lord of the ", 1)
  lcd(display, "Processes", 2)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, "One Process to", 1)
  lcd(display, "rule them all,", 2)
  time.sleep(2)
  display.lcd_clear()

  lcd(display, "and in darkness", 1)
  lcd(display, "bind them.", 2)
  time.sleep(2)




#################################
#				#
#	DEFINE MAIN FUNCTION	#
#				#
#################################

def main():

#
# Defaults Values 
#
  frame_size = 400
  fps = 100
  radius = 0
  throttle_def = "50"				# Default throttle
  throttle = float( throttle_def)  		# Default throttle
  rotate_throttle_def = "40"
  rotate_throttle = float( rotate_throttle_def ) 	# Default rotation throttle
  fwd_time_def = 0.5				# Default forward time
  fwd_time = float( fwd_time_def )		# Default forward time
  rotate_time_def = 0.75			# Default rotation time
  rotate_time = float( rotate_time_def) 	# Default rotation time
  arrow_flash = 0
  direction = "FRONT"
  direction_txt = "0 - Direction: " + direction
  NrIterations = 2 
  DefaultRadius = 80
  EstDistance = 0
  gpio_out = 12
  mindX = 30
  mindY = 20
  NavMod = ord("o") 		# o: Object Tracking; m: Manual Steering
  NavMode_txt = "Nav Mod [o/p/m]: "
  manual_override = 0
  manual_override_def = "0"
  Temperature_txt = "T(C): DEFAULT"
  Humidity_txt = "H(%): DEFAULT"
  temperature = 0
  humidity = 0
  Bearing_txt = "Bearing (deg): N/A"
  throttle_txt = "Throttle [f/s]: 0%"
  rotate_throttle_txt = "Rotational Throttle [l/r]: 0%"
  mission_start = 0
  mission_time_txt = "0:00:00.0"
  mission_txt = "Mission Time: " + mission_time_txt
  start_objtrk_time = 0
  port="/dev/ttyAMA0"
  logfile="ulisse.log"
  distance_txt = "!@: N/A!"
  lat_txt = "Latitude: N/A"
  lng_txt = "Longitude: N/A"
  alt_txt = "Altitude: N/A"
  xaccl_txt = "X-Axis accl. (G): N/A"
  yaccl_txt = "Y-Axis accl. (G): N/A"
  zaccl_txt = "Z-Axis accl. (G): N/A"
  target_lock_txt = "Target not locked"
  cm = 0
  old_temp = 0
  old_hum = 0

#
# Default configuration parameters, otherwise read from ulisse.cfg
#

  dbg_cfg = "none" 	# none, basic, full
  motor_cfg = 0		# 0, 1 - Enable/Disable Motors
  lcd_cfg = 0       	# 0, 1 - Write/Dont write on LCD
  gps_cfg = 0	    	# 0, 1 - Enable/Disable GPS navigation
  dht_cfg = 0 	    	# 0, 1 - Enable/Disable DHT readings
  servo_cfg = 0		# 0, 1 - Enable/Disable Servo motor 
  compass_cfg = 0       # 0, 1 - Enable/Disable Compass
  accl_cfg = 0       	# 0, 1 - Enable/Disable Accelerometer
  shift_cfg = 0		# 0, 1 - Enable/Disable Shift Register
  dht_time = 60    	# Time interval among DHT readings, in seconds 
  gps_time = 60    	# Time interval among GPS readings, in seconds 
  compass_time = 60    	# Time interval among Compass readings, in seconds 
  accl_time = 60    	# Time interval among Accelerometer readings, in seconds 
  objtrk_time = 0	# Time interval for Object Traking, in seconds - 0 = Real Time
  old_lat = lat_txt 
  old_lng = lng_txt 
  old_alt = alt_txt
  size = 0
  ran = 0
  leftright = 0

#
# KICK OFF THE PARALLEL COMPUTING ENVIRONMENT
#
  print("****************************************" )
  print("*                                      *" )
  print("*  B U L U N   S K U N K    W O R K S  *" )
  print("*                                      *" )
  print("*            welcome you to            *" )
  print("*                                      *" )
  print("*          U.L.I.S.S.E. HPC            *" ) 
  print("*                                      *" )
  print("*     The Lord of the Processes        *" ) 
  print("*                                      *" )
  print("*  'One Process to rule them all,'     *" ) 
  print("*  'and in darkness bind them.'        *" ) 
  print("*                                      *")
  print("****************************************")
  print( "\nStarting up Message Passing Interface..." )

  mpi_rank = 0
  mpi_size = 0

  mpi_comm = MPI.COMM_WORLD
  mpi_size = MPI.COMM_WORLD.Get_size()
  mpi_rank = MPI.COMM_WORLD.Get_rank()
  mpi_name = MPI.Get_processor_name()

  mpi_dest_rank = 0
  mpi_source_proc = 0
  tmp_data = [0,0,0,0,0,0]
  mpi_data = [0,				# 0/-1: continue/terminate
              0,				# leftright
              0,				# distance
              "Bearing (deg): N/A",	 	# Bearing 
              0,				# Temperature
              0,				# Humidity
              "Latitude: N/A",			# Latitude
              "Longitude: N/A",			# Longitude
              "Altitude: N/A",			# Altitude
              "X-Axis G: N/A",			# x-Axis Acceleration (in G) 
              "Y-Axis G: N/A",			# y-Axis Acceleration (in G) 
              "Z-Axis G: N/A",			# z-Axis Acceleration (in G) 
              0,                                # arrow_flash
              "FRONT",				# direction
	      0,				# throttle
              "!@ N/A",				# Distance txt
	      0,				# rotational throttle
              0 ]				# Manual Override

  print ("Message Passing Interface Started up." )

#
# ALL PROCESSES TO READ ulisse.cfg 
#
  print( "\nFirst things first: try to read ulisse.cfg" )

  configfile = ConfigParser.ConfigParser()

  try:
    configfile.readfp( open( r'ulisse.cfg' ) )

    logfile = configfile.get( 'Ulissecfg', 'logfile' )

    logfile = logfile + "." + str( mpi_rank )
    outputfile = open( logfile, "w+")

    dbg_cfg = configfile.get( 'Ulissecfg', 'dbg_cfg' )
    motor_cfg = configfile.get( 'Ulissecfg', 'motor_cfg' )
    lcd_cfg = configfile.get( 'Ulissecfg', 'lcd_cfg' )
    gps_cfg = configfile.get( 'Ulissecfg', 'gps_cfg' )
    dht_cfg = configfile.get( 'Ulissecfg', 'dht_cfg' )
    servo_cfg = configfile.get( 'Ulissecfg', 'servo_cfg' )
    compass_cfg = configfile.get( 'Ulissecfg', 'compass_cfg' )
    accl_cfg = configfile.get( 'Ulissecfg', 'accl_cfg' )
    shift_cfg = configfile.get( 'Ulissecfg', 'shift_cfg' )

    dht_time = configfile.get( 'Ulissetiming', 'dht_time' )
    gps_time = configfile.get( 'Ulissetiming', 'gps_time' )
    objtrk_time = configfile.get( 'Ulissetiming', 'objtrk_time' )
    compass_time = configfile.get( 'Ulissetiming', 'compass_time' )
    accl_time = configfile.get( 'Ulissetiming', 'accl_time' )

    manual_override_def = configfile.get ( 'Defaults', 'manual_override_def' )
    throttle_def = configfile.get ( 'Defaults', 'throttle_def' )
    rotate_throttle_def = configfile.get ( 'Defaults', 'rotate_throttle_def' )
    fwd_time_def = configfile.get ( 'Defaults', 'fwd_time_def' )
    rotate_time_def = configfile.get ( 'Defaults', 'rotate_time_def' )

  except IOError:
    logfile = logfile + "." + str( mpi_rank )
    outputfile = open( logfile, "w+")
    outputfile.write( "--> WARNING!!!\n" )
    outputfile.write( "--> File ulisse.cfg not found or not readable.\n" )
    outputfile.write( "--> Using default configuration parameters:\n," )

  outputfile.write("*************************************\n" )
  outputfile.write("*                                   *\n" )
  outputfile.write("*          U.L.I.S.S.E. HPC         *\n" ) 
  outputfile.write("*                                   *\n" )
  outputfile.write("*     The Lord of the Processes     *\n" ) 
  outputfile.write("*                                   *\n" )
  outputfile.write("*  'One Process to rule them all,'  *\n" ) 
  outputfile.write("*  'and in darkness bind them.'     *\n" ) 
  outputfile.write("*                                   *\n")
  outputfile.write("*************************************")
  outputfile.write( "\n\nStarting up Message Passing Interface..." )
  outputfile.write( "\n\nMission Start Time: \n\n" )
  outputfile.write( "Message Passing Interface\n" )
  outputfile.write( "-------------------------\n" )
  outputfile.write( "Nr of Processes executed: " + str( mpi_size ) + "\n" ) 
  outputfile.write( "Process ID: " + str( mpi_rank ) + "\n" )
  outputfile.write( "Hostname: " + str( mpi_name ) + "\n" )
  outputfile.write( "\nUlisse Configuration Parameters" )
  outputfile.write( "\n-------------------------------\n" )
  outputfile.write( "dbg_cfg = " + dbg_cfg + "\n" )
  outputfile.write( "motor_cfg = " + motor_cfg + "\n" )
  outputfile.write( "lcd_cfg = " + lcd_cfg + "\n" )
  outputfile.write( "gps_cfg = " + gps_cfg + "\n" )
  outputfile.write( "dht_cfg = " + dht_cfg + "\n" )
  outputfile.write( "servo_cfg = " + servo_cfg + "\n" )
  outputfile.write( "compass_cfg = " + compass_cfg + "\n" )
  outputfile.write( "accl_cfg = " + accl_cfg + "\n" )
  outputfile.write( "shift_cfg = " + shift_cfg + "\n" )
  outputfile.write( "dht_time = " + dht_time + "\n" )
  outputfile.write( "gps_time = " + gps_time + "\n" )
  outputfile.write( "objtrk_time = " + objtrk_time + "\n" )
  outputfile.write( "compass_time = " + compass_time + "\n" )
  outputfile.write( "accl_time = " + accl_time + "\n" )

  manual_override = int( manual_override_def )
  throttle = float( throttle_def )
  rotate_throttle = float( rotate_throttle_def )
  fwd_time = float( fwd_time_def )
  rotate_time = float( rotate_time_def )

  outputfile.write( "manual_override_def = " + manual_override_def + "\n" )
  outputfile.write( "throttle_def = " + throttle_def + "\n" )
  outputfile.write( "rotate_throttle = " + rotate_throttle_def + "\n" )
  outputfile.write( "fwd_time = " + fwd_time_def + "\n" )
  outputfile.write( "rotate_time = " + rotate_time_def + "\n\n" )

#
# construct the argument parse and parse the arguments
#

  ap = argparse.ArgumentParser()
  ap.add_argument("-f", "--fine-debug", dest='fine', action='store_true',
        help="debug mode")

  ap.add_argument("-d", "--debug", dest='debug', action='store_true',
        help="debug mode")
  ap.add_argument("-v", "--video",
        help="path to the (optional) video file")
  ap.add_argument("-b", "--buffer", type=int, default=32,
        help="max buffer size")
  args = vars(ap.parse_args())

  if( mpi_rank == 0):
#
# START MPI MASTER PROCESS
#

    start_time = time.time()

    mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
    mission_txt = "Mission Time: " + mission_time_txt 

#
# START THE WORKERS PROCESS UP
#
#    mpi_data[0] = 0 
    
    outputfile.write( "\n" + mission_txt + " Start up Workers\n" )
    print( "\nStart up Workers" )

    for dest_rank in range (1, mpi_size ):
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      outputfile.write( "\n" + mission_txt + " Sending startup signal to Worker ID: " + str(dest_rank ) )

      mpi_comm.send( mpi_data, dest = dest_rank, tag=1 )
    
    mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
    mission_txt = "Mission Time: " + mission_time_txt 

    outputfile.write( "\n\n" + mission_txt + " Getting MPI.Status\n" )
    print( "Getting MPI.Status" )

    mpi_status = MPI.Status()

    if( dbg_cfg == "basic" ):
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      outputfile.write( mission_txt + " MPI.Status " + str(mpi_status) +"\n")

      print
      print( "U.L.I.S.S.E. Navigation System started up. Let's rock!" )

#
# START NAVIGATION
#
    mission_start = time.time()

    while True:
# receive "OK" signal from workers

        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt 
#        outputfile.write( "\n" + mission_txt + " -  LOOPING...\n" )
#	print("0744 - ", mission_time_txt, " - LOOPING...")

        mpi_data = mpi_comm.recv( source=MPI.ANY_SOURCE, tag = 1, status=mpi_status )
        mpi_source_proc = mpi_status.Get_source()

        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt 

#	print( "0751 - ", mission_time_txt, " - RANK 0 - RCVD MSG from ", mpi_source_proc )

        if( mpi_source_proc == 1 ):
# Received Video Stream response
#	  print("0716 - MSG Received from Video Stream") 

          manual_override = mpi_data[ 17 ]

          if( dbg_cfg == "full" ):
            outputfile.write( "\n" + mission_txt + " -  MSG Received from Video Stream\n" )

          if( mpi_data[0] == -1 ):
# Received Termination signal
            if( dbg_cfg == "full" ):
              outputfile.write( "\n" + mission_txt + " -  From Worked ID: " + str(mpi_source_proc ) + " received Video Stream: " + str( leftright ) + "\n" )

            break
        elif( mpi_source_proc == 2 ):
# Received OK from Motors
          if( dbg_cfg == "full" ):
            outputfile.write( "\n" + mission_txt + " -  From Worked ID: " + str(mpi_source_proc ) + " received OK from MOTORS " + "\n" )

        elif( mpi_source_proc == 4 ):
# Received OK from Shift Register

          if( dbg_cfg == "basic" ):
            outputfile.write( "\n" + mission_txt + " -  From Worked ID: " + str(mpi_source_proc ) + " received OK from Shift register\n" )

        elif( mpi_source_proc == 6 ):
          leftright= mpi_data[1]

        elif( mpi_source_proc == 5 ):
# Received Distance from Obstacle
          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt

#          leftright= mpi_data[1]
          cm = mpi_data[2]
          
          if( dbg_cfg == "full" ):
            outputfile.write( "\n" + mission_txt + " -  From Worked ID: 5 received Distance = " + str(cm) + " - leftright=" + str(leftright)+"\n")

          #
          #  CHECK IF OBSTACLES ARE IN RANGE
          #

          if( cm < 0 ):
            distance_txt = "!@ N/A!"

          elif( cm > 300 ):
            distance_txt = "!@ >300 cm"

          else:
            distance_txt = "!@: " + str( cm ) + " cm"

          mpi_data[ 15 ] = distance_txt

          if( cm > 0 and cm < 15 ):
            if ( leftright == 0 ):
              direction = "RIGHT"

            else:
              direction = "LEFT"

          else:
             direction = "FRONT"

          outputfile.write( mission_time_txt + " - 796 - Distance: " + str(cm) + " - leftright: " + str(leftright ) + " - Direction: " + direction +"\n")

          direction_txt = "Direction: " + direction

          if( motor_cfg == '1' ):
            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt

            outputfile.write("\n" + mission_txt + " Obstacle - move to " + direction + "\n" )

          mpi_data[ 12 ] = arrow_flash
          mpi_data[ 13 ] = direction
          mpi_data[ 14 ] = throttle
          mpi_data[ 16 ] = rotate_throttle

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt
          outputfile.write(mission_txt + " - 812 - DISPLAYING on LCD - distance: " + mpi_data[ 15] + "\n" )

          # Display messages on LCD
          mpi_comm.send( mpi_data, dest = 4, tag = 1 )
          mpi_data = mpi_comm.recv( source= 4, tag = 1, status=mpi_status )

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt
          outputfile.write(mission_txt + " - 820 - MOVING THE ROBOT - Direction: " + mpi_data[13] + " throttle = " + str( mpi_data[14]) + "\n" )

#	  print("0830 - ", mission_time_txt, " - WRITTEN ON LCD")

          # Move the Robot to avoid obstacle

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt
          outputfile.write(mission_txt + " RETURN move 815\n" )
#	  print("0837 - ", mission_time_txt, " - CHECK IF OBSTACLE")

          if( motor_cfg == '1' and mpi_size > 2 and direction != "FRONT" and manual_override == 0 ):
            mpi_comm.send( mpi_data, dest = 2, tag = 1 )
#            mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )
#	    print("0842 - ", mission_time_txt, " - AVOIDED OBSTACLE")
            outputfile.write(mission_txt + " AVOIDED Obstacle 0842\n" )
	  else:
#	    print("0838 - ", mission_time_txt, " - NO OBSTACLE TO AVOID")
            outputfile.write(mission_txt + " NO Obstacle to avoid.\n" )

        elif( mpi_source_proc == 3 ):
# Received DHT, GPS, Accelerometer, Compass

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt

          if( dbg_cfg == "full" ):
            outputfile.write("0848 - " + mission_time_txt + "Received MSG from Worked ID 3" )

          outputfile.write("0848 - " + mission_time_txt + "Received MSG from Worked ID 3 - T: "+str(mpi_data[4])+" - H: "+ str(mpi_data[5]) )

          old_temp = mpi_data[4]  
          old_hum = mpi_data[5]  

#          print("0848 - ", mission_time_txt, "Received MSG from Worked ID 3 - T: ", mpi_data[ 4 ], " - H: ", mpi_data[ 5 ] )

        else:
          if( dbg_cfg == "basic") :
            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt 
            outputfile.write( mission_txt + " - Received OK signal from Worked ID: " + str(mpi_source_proc ) + "\n" )

        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt 
#        outputfile.write( mission_txt + " - Send OK signal to Worked ID: " + str(mpi_source_proc ) + "\n" )

        mpi_data[0] = 0

        if(mpi_source_proc != 2):
          mpi_data[4] = old_temp
          mpi_data[5] = old_hum

          outputfile.write( mission_txt + " - Send OK signal to Worked ID: " + str(mpi_source_proc ) + " - T:"+ str(mpi_data[4]) + " - H:" + str(mpi_data[5]) +"\n" )

#	  print( "872 - ", mission_time_txt, " - Send Continue Message to rank = ", mpi_source_proc )

          mpi_comm.send( mpi_data, dest=mpi_source_proc, tag=1 )

        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt 
#        print( "0880 - ", mission_time_txt, " - END LOOPING" )
	
#
# MASTER sends termination signal to all Working processes
#

    mpi_data[0] = -1

    for mpi_dest_rank in range (3, mpi_size ):
      outputfile.write( "MPI MASTER PROCESS, sending termination signal to Worker: " + str( mpi_dest_rank ) +"\n")

      mpi_comm.send( mpi_data, dest=mpi_dest_rank, tag=1 )

    print("\nMASTER PROCESS - Game over, man. Game over!" )

  elif ( mpi_rank == 3 ):
#
# STEER THE DHT, GPS, Accelerometer, Compass 
#

#    print( "Distance Sensor Initialized.")
#    outputfile.write( "Distance Sensor Initialized.\n")

# Initialize GPS

    startGPS_time = 0

    if( gps_cfg == '1' ):
      print( '\nInitialization of GPS sensor...' )
      outputfile.write( "\nInitialization of GPS sensor...\n" )

      ser=serial.Serial(port, baudrate=9600, timeout=0.5)

      print("GPS Sensor Initialized." )
      outputfile.write("GPS Sensor Initialized." )

# Initialize Accelerometer
    if( accl_cfg == '1' ):
      startAccl_time = 0
      old_acclx = "X-Axis accl. (G): N/A"
      old_accly = "Y-Axis accl. (G): N/A"
      old_acclz = "Z-Axis accl. (G): N/A"

      print("\nInitialization of Accelerometer Sensor..." )
      outputfile.write("\nInitialization of Accelerometer Sensor..." )

      print("Accelerometer Sensor Initialized." )
      outputfile.write("\nAccelerometer Sensor Initialized." )

# Initialize Compass
    startCompass_time = 0
    old_mpidata = "Bearing (deg):" 

    if( compass_cfg == '1' ):
      print( "\nInitialization of Compass Sensor..." )
      outputfile.write( "\nInitialization of Compass Sensor..." )

      sensor = 0

      print( "Compass Sensor Initialized." )
      outputfile.write( "\nCompass Sensor Initialized." )

# Initialize DHT

    if( dht_cfg == '1' ):
      print( "\nInitialization of DHT Sensor..." )
      outputfile.write( "\nInitialization of DHT Sensor..." )

# DHT PIN in GPIO.BOARD mode
#      DHT11_PIN = 26		# 
      DHT11_PIN = 13		# <--- WARNING!!!! PIN IN BCM MODE!!!!!!!

      humidity = 0
      temperature = 0
      startDHT_measure = 0
      sensorDHT = Adafruit_DHT.DHT11

      print ("DHT Sensor Initialized.")
      outputfile.write("\nDHT Sensor Initialized.")

# Perform first DHT measure

      print("\nPerform first Temperature and Humidity measurement." )
      outputfile.write("\nPerform first Temperature and Humidity measurement." )

      humidity, temperature = Adafruit_DHT.read_retry(sensorDHT, DHT11_PIN)

      if humidity is not None and temperature is not None:
        if( dbg_cfg == "basic" or dbg_cfg == "full" ):
          print('Temperature={0:0.1f}*C  Humidity={1:0.1f}%'.format(temperature, humidity))

        Temperature_txt = "Temperature (C): " + str( temperature )
        Humidity_txt = "Humidity (%): " + str( humidity )

      else:
        print
        print('--> WARNING!!!')
        print('--> Failed to get reading from the DHT sensor. Try again!')

      outputfile.write( "\nFIRST READINGS: " + Temperature_txt + " " + Humidity_txt + "\n")

# Loop over DHT, GPS until termination signal is received.

    while True:
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      if( dbg_cfg == "full" ):
        outputfile.write("\n" + mission_txt + "- DHT, GPS Worker ID: " + str( mpi_rank ) + " - Waiting for MSG"  )

      mpi_data = mpi_comm.recv( source=0, tag=1 )

#      print( "1568 - ", mission_time_txt, " - RANK 3 - MSG RECVD" )

      mpi_data[4] = temperature
      mpi_data[5] = humidity

      if( mpi_data[0] == -1 ):
# received exit signal, leave While loop
        outputfile.write( "\n" + mission_txt + " - DHT, GPS Worker ID: " + str( mpi_rank ) + " - Received exit signal.\n" )
        print("\nWorker ID: ", mpi_rank, " - Game over, man. Game over!" )

        break

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      if( dbg_cfg == "full" ):
        outputfile.write( "\n" + mission_txt + " DHT, GPS Worker ID: " + str( mpi_rank ) + " - I'm alive and kicking\n"  )

# DETERMINE BEARING
      if( compass_cfg == '1' ):
        if( startCompass_time == 0 ):
          startCompass_time = time.time()

        if( time.time() - startCompass_time > float( compass_time ) ): 
          startCompass_time = 0

          reading = bearing( sensor, dbg_cfg )

	  if( reading >= 0 and reading <=22.5 ):
	    dir = "N"
          elif( reading > 22.5 and reading <= 67.5 ):
	    dir = "NE"
	  elif( reading > 67.5 and reading <= 112.5 ):
	    dir = "E"
          elif( reading > 112.5 and reading <= 157.5 ):
	    dir = "SE"
	  elif( reading > 157.5 and reading <= 202.5 ):
	    dir = "S"
	  elif( reading > 202.5 and reading <= 247.5 ):
	    dir = "SO"
	  elif( reading > 247.5 and reading <= 292.5 ):
	    dir = "O"
	  elif( reading > 292.5 and reading <= 337.5 ):
	    dir ="NO" 
 	  elif( reading > 337.5 and reading <= 360 ):
	    dir = "N"

          mpi_data[ 3 ] = "Bearing (deg): " + str(round( reading, 1 ) ) + " (" + dir + ")"
          old_mpidata = mpi_data[ 3 ]
        else:
          mpi_data[ 3 ] = old_mpidata

        if(dbg_cfg == "full" ):
          print("\nBearing", mpi_data[ 3 ] )

        outputfile.write("\n\n" + mpi_data[ 3 ] + "\n")

# DETERMINE ACCELERATION
      if( accl_cfg == '1' ):
        if( startAccl_time == 0 ):
          startAccl_time = time.time()

        if( time.time() - startAccl_time > float( accl_time ) ):
          startAccl_time = 0

          if( dbg_cfg == "full" ):
            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt 
            outputfile.write("\n" + mission_txt + " - Worker ID 3 - Reading Acceleration information" )

          tmp_data = xlr8( dbg_cfg )

          if( dbg_cfg == "full" ):
            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt 
            outputfile.write("\n" + mission_txt + " - Worker ID 3 - Acceleration information\n" )

          mpi_data[ 9 ] = "X-Axis accl. (G): " + str( round( float(tmp_data[ 0 ]), 4  ) )
          mpi_data[ 10 ] = "Y-Axis accl. (G): " + str( round( float(tmp_data[ 1 ]), 4 ) )
          mpi_data[ 11 ] = "Z-Axis accl. (G): " + str( round( float(tmp_data[ 2 ]), 4 ) )
          old_acclx = mpi_data[ 9 ]
          old_accly = mpi_data[ 10 ]
          old_acclz = mpi_data[ 11 ]
        else:
          if( dbg_cfg == "full" ):
            print("old_acclx = ", old_acclx )
            print("old_accly = ", old_accly )
            print("old_acclz = ", old_acclz )

          mpi_data[ 9 ] = old_acclx
          mpi_data[ 10 ] = old_accly
          mpi_data[ 11 ] = old_acclz

        if( dbg_cfg == "full" ):
          outputfile.write("X-Accl = " + mpi_data[ 9 ] + " X-Reading: " + str( tmp_data[ 3 ] ) + "\n")
          outputfile.write("Y-Accl = " + mpi_data[ 10 ] + " Y-Reading: " + str( tmp_data[ 4 ] ) + "\n" )
          outputfile.write("Z-Accl = " + mpi_data[ 11 ] + " Z-Reading: " + str( tmp_data[ 5] ) + "\n")
    

# DETERMINE GPS POSITION

      if( gps_cfg == '1' ):
          if( startGPS_time == 0 ):
	    startGPS_time = time.time() 

          if( time.time() - startGPS_time > float( gps_time ) ):
            startGPS_time = 0

            dataout = pynmea2.NMEAStreamReader()
            newdata=ser.readline().decode('UTF-8')

            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt 
            outputfile.write("\n" + mission_txt + " Worker ID 3 - GPS Reading: "+ newdata + "\n" )
#            print("Worker ID 3 - GPS information: ", newdata )

            if newdata[0:6] == "$GPGGA":
                newmsg=pynmea2.parse(newdata)
                lat=newmsg.latitude
                lat_min=newmsg.latitude_minutes
                lat_sec=newmsg.latitude_seconds
                lat_dir=newmsg.lat_dir

                lng=newmsg.longitude
                lng_min=newmsg.longitude_minutes
                lng_sec=newmsg.longitude_seconds
                lng_dir=newmsg.lon_dir

                num1 ='{:.5f}'.format(lat)
                num2 ='{:.5f}'.format(lat_min)
                num3 ='{:.5f}'.format(lat_sec)

                gps = "Latitude= " + str(num1) + " deg " + \
                          str( num2 ) + string.printable[68] + " " + \
                          str( num3) + string.printable[63] + " " + lat_dir

                old_lat = gps
                mpi_data[6] = gps
                outputfile.write( gps + '\n' )

                num1 ='{:.5f}'.format(lng)
                num2 ='{:.5f}'.format(lng_min)
                num3 ='{:.5f}'.format(lng_sec)

                gps = "Longitude= " + str(num1) + " deg " + \
                          str( num2 ) + string.printable[68] + " " + \
                          str( num3) + string.printable[63] + " " + lng_dir

                mpi_data[7] = gps
                old_lng = gps

 
                outputfile.write( gps + '\n' )

                alt = newmsg.altitude
                alt_units = newmsg.altitude_units
                gps = "Altitude = " + str( alt ) + " " + alt_units
                mpi_data[8] = gps 
                old_alt = gps

                outputfile.write( gps + '\n' )

            else:
                mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
                mission_txt = "Mission Time: " + mission_time_txt 
                outputfile.write( "\nNo GPS Readings\n" )
                mpi_data[6] = old_lat
                mpi_data[7] = old_lng
                mpi_data[8] = old_alt
          else:
            mpi_data[ 6 ] = old_lat
            mpi_data[ 7 ] = old_lng
            mpi_data[ 8 ] = old_alt
        
                 
# MEASURE TEMPERATURE AND HUMIDITY - BUT ONLY every dht_time seconds 

      if( dht_cfg == '1' ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt 

        if( startDHT_measure == 0 ):
          startDHT_measure= time.time()

        if( dbg_cfg == "full" ):
          outputfile.write( "\n\n" + mission_txt + " Worker ID: " + str( mpi_rank ) + " - DHT Measure\n" )
          outputfile.write( "\nTemperature in memory: " + Temperature_txt + " Humitidy in memory: " + Humidity_txt + "\n" )

        if ( time.time() - startDHT_measure > float( dht_time ) ):

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt 
          outputfile.write( mission_txt + " Worker ID: " + str( mpi_rank ) + " Resetting StartDHT_measure.")

          startDHT_measure = 0

          outputfile.write("\nMeasuring DHT\n" )
          humidity, temperature = Adafruit_DHT.read_retry(sensorDHT, DHT11_PIN)

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt 

          if( dbg_cfg == "full" ):
            outputfile.write( mission_txt + " Worker ID: " + str( mpi_rank ) + " - READ DHT\n" )

          if humidity is not None and temperature is not None:
            mpi_data[4] = temperature
            mpi_data[5] = humidity

          else:
            print
            print('WARNING!!!')
            print('-->Failed to get reading from the DHT sensor. Try again!')

            mpi_data[4] = 0
            mpi_data[5] = 0

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt 

        outputfile.write( mission_txt + " Worker ID 3 - DHT RETURNED TO MASTER:" + str( mpi_data[4]) + " " + str( mpi_data[5]) + "\n" )

# sending back to Master "OK" signal

#      print(" 1251 - ", mission_time_txt, " - send sensors data to Video Stream process" ) 
      mpi_comm.send( mpi_data, dest = 1, tag = 1 )

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
#      print(" 1255 - ", mission_time_txt, " - send OK signal to Master" ) 
      mpi_comm.send( mpi_data, dest = 0, tag = 1 )

  elif( mpi_rank == 4 ):
# Display on LCD and steer the Shift Register

#
# INITIALIZE LCD DRIVER
#
    if( lcd_cfg == '1' ):
      print ("\nInitialization of LCD driver...")
      outputfile.write ("\nInitialization of LCD driver...")

      display = lcddriver.lcd()

      print( "LCD Driver initialized.\n\nDisplay Welcome Message.\n")
      outputfile.write( "\nLCD Driver initialized.\nDisplay Welcome Message.")

      welcome( display )

#
# Steer the Shift Register
#

    if( shift_cfg == '1' ):
      print("\nInitialization of the Shift Register..."  )
      outputfile.write("Initialization of the Shift Register...\n"  )

# PINS FOR BOARD MODE
      SER_PIN = 29 
      SCK_PIN = 31 
      RCK_PIN = 7 

      GPIO.setup(SER_PIN,GPIO.OUT)
      GPIO.setup(SCK_PIN,GPIO.OUT)
      GPIO.setup(RCK_PIN,GPIO.OUT)

      print("Shift Register initialized.\n"  )
      outputfile.write("Shift Register initialized.\n\n"  )

    while True:
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      if( dbg_cfg == "full" ):
        outputfile.write("\n" + mission_txt + " - Worker ID: " + str( mpi_rank ) + " - Waiting for MSG\n"  )

      mpi_data = mpi_comm.recv( source=0, tag=1 )
#      print( "1793 - ", mission_time_txt, " - RANK 4 - MSG RECVD - T: ", mpi_data[4], " - H: ", mpi_data[5] )

      if( mpi_data[0] == -1 ):
# received exit signal, leave While loop
        print("\nWorker ID: ", mpi_rank, " - Game over, man. Game over!" )
        if( lcd_cfg == '1' ):
          display.lcd_clear()
          lcd( display, "Game over, man.", 1 )
          lcd( display, "GAME OVER!", 2  )
 
        outputfile.write( "Worker ID: " + str( mpi_rank ) + " - Received exit signal.\n" )

        break

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      if( dbg_cfg == "basic" ):
        outputfile.write("\n" + mission_txt +" - Worker ID 4: Temperature = " + str( mpi_data[ 4 ] )  + " Humidity = " + str( mpi_data [5 ] ) )

      Temperature_txt = "T: " + str( mpi_data[ 4 ] )
      Humidity_txt = "H: " + str( mpi_data [5 ] )

      if( lcd_cfg == '1' ):
        display.lcd_clear()
        lcd( display, Temperature_txt +" "+ Humidity_txt, 1 )
#        lcd( display, Humidity_txt, 2 )
        lcd( display, mpi_data[ 15 ], 2 )

# Steer the Shift Register and switch the LEDs on
      if( shift_cfg == '1' ):
        shift_reg( dbg_cfg, SER_PIN, SCK_PIN, RCK_PIN )

        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt 

        outputfile.write("\n" + mission_txt + " Worker ID: " + str( mpi_rank ) + " - RETURNING from shift_reg\n\n"  )

# sending back to Master "OK" signal

      mpi_data[0] = 0
      mpi_comm.send( mpi_data, dest = 0, tag = 1 )

  elif( mpi_rank == 2 ):
#
# ACTIVATE MOTORS
#

    while True:
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      outputfile.write("\n\n" + mission_txt + " Worker ID: " + str( mpi_rank ) + " - Waiting for MSG\n"  )

      mpi_status = MPI.Status()
      mpi_data = mpi_comm.recv( source=MPI.ANY_SOURCE, tag=1, status = mpi_status )
      mpi_source_proc = mpi_status.Get_source()
     
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt
     
#      print( "1357 - ", mission_time_txt, " - RANK 2 - RCVD MSG from ", mpi_source_proc )

      if( mpi_data[0] == -1 ):
# received exit signal, leave While loop
        outputfile.write( "Worker ID: " + str( mpi_rank ) + " - Received exit signal.\n" )
        print("\nWorker ID: ", mpi_rank, " - Game over, man. Game over!" )

        break

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 
      outputfile.write(mission_txt + " Worker ID: " + str( mpi_rank ) + " - I'm alive and kicking"  )

      if( dbg_cfg == "full" ):
        outputfile.write("\nArrow_flash: " + str( mpi_data[ 12 ]  ) )
        outputfile.write("\nDirection: " + mpi_data[ 13 ] )
        outputfile.write("\nThrottle: " + str( mpi_data[ 14 ] ) )

      move( mpi_data[ 12 ], mpi_data[ 13 ], mpi_data[ 14 ], mpi_data[ 16 ], fwd_time, rotate_time, dbg_cfg )

#      time.sleep(1)

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt 

      if( dbg_cfg == "full" ):
        outputfile.write("\n" + mission_txt + " Worker ID: " + str( mpi_rank ) + " - RETURNED MOVE 1857"  )

# sending back to Master "OK" signal

      mpi_data[0] = 0
      mpi_comm.send( mpi_data, dest = 1, tag = 1 )

  elif( mpi_rank == 5 ):
#
# STEER THE DISTANCE SENSOR and SERVO MOTOR
#

# Initialize Distance Sensor

    print( "Initialization of Distance Sensor...")
    outputfile.write( "Initialization of Distance Sensor...\n")    

#
# Distance Sensors in GPIO.BOARD mode
#
    TRIG_PIN = 38
    ECHO_PIN = 40

    GPIO.setup(TRIG_PIN,GPIO.OUT)
    GPIO.setup(ECHO_PIN,GPIO.IN)

    GPIO.output(TRIG_PIN, False)
    time.sleep(2)

    print( "Distance Sensor Initialized.")
    outputfile.write( "Distance Sensor Initialized.\n")

#
# Initialize the Servo Motor
#
    mpi_recv_status = 0
    angle = 0
    leftright = 0 # 0 = left, 1 = right
    mission_start = time.time()

    if( servo_cfg == '1' ):

#
# Servo PIN in GPIO.BOARD mode
#
      SERVO_PIN = 12

      GPIO.setup(SERVO_PIN, GPIO.OUT)

      pwm_servo = GPIO.PWM(SERVO_PIN, 50)

      print( "Servo Motor initialized.")
      outputfile.write( "Servo Motor initialized.\n")

      pwm_servo.start(0)


    while True:
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt

      outputfile.write("\nWorker ID: " + str( mpi_rank ) + " " + mission_txt + " - Waiting for MSG\n"  )

      mpi_data = mpi_comm.recv( source=0, tag=1 )
#      print( "1906 - ", mission_time_txt, " - RANK 5 - MSG RECVD" )

      if( mpi_data[0] == -1 ):
# received exit signal, leave While loop
        outputfile.write( "Worker ID: " + str( mpi_rank ) + " - Received exit signal.\n" )

        if( servo_cfg == '1' ):
          SetAngle( pwm_servo, SERVO_PIN, 50 )
          pwm_servo.stop()

        break

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt

      if( dbg_cfg == "full" ):
        outputfile.write("Worker ID: " + str( mpi_rank ) + " " + mission_txt + " - I'm alive and kicking\n"  )

      if( servo_cfg == '1' ):
#
# ROTATE DISTANCE SENSOR
#

        if ( angle == 0 ):
          leftright = 0
        elif (angle == 100 ):
          leftright = 1

        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt

        outputfile.write( mission_txt + " Servo Motor: leftright = " + str( leftright ) + ", angle = " + str( angle ) + "\n")

        SetAngle( pwm_servo, SERVO_PIN, angle )

        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        mission_txt = "Mission Time: " + mission_time_txt

        outputfile.write( mission_txt + " - Returned SetAngle\n")

        if ( leftright == 0 ):
          angle = angle + 50
        elif ( leftright == 1 ):
           angle = angle - 50

        mpi_data[1] = leftright


# Check if obstacle are in range

      cm = -1 

      while( cm < 0 ):
        cm = distance( TRIG_PIN, ECHO_PIN )

        if( cm < 0 ):
          distance_txt = "Obstacle at: NOT DETECTED!"

        elif( cm > 300 ):
          distance_txt = "Obstacle at: >300 cm"

        else:
          distance_txt = "Obstacle at: " + str( cm ) + " cm"

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt

      outputfile.write("Worker ID: " + str( mpi_rank ) + " " + mission_txt + " " + distance_txt + "\n")

# sending back to Master "OK" signal

#      print("Worker ID 5 - Send OK signal to Master - cm = ", cm )

      mpi_data[2] = cm
      mpi_comm.send( mpi_data, dest = 1, tag = 1 )
      mpi_comm.send( mpi_data, dest = 0, tag = 1 )

  elif ( mpi_rank == 1 ):

    mission_start = time.time()
#
# VIDEO STREAM
#
 
    temperature_old = 0
    humidity_old = 0
    Bearing_txt_old = Bearing_txt
    lat_txt_old = lat_txt
    lng_txt_old = lng_txt
    alt_txt_old = alt_txt
    xaccl_txt_old = xaccl_txt
    yaccl_txt_old = yaccl_txt
    zaccl_txt_old = zaccl_txt
    Temperature_txt_old = Temperature_txt
    Humidity_txt_old = Humidity_txt
    distance_txt_old = distance_txt
 
#
# Define the lower and upper boundaries of the "green" ball
# in the HSV color space
#

    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)
 
#
# Initialize the list of tracked points,
# the frame counter, and the coordinate deltas
#

    pts = deque(maxlen=args["buffer"])
    counter = 0
    (dX, dY) = (0, 0)

#
# Grab the reference to the webcam and start the Video stream,
# warm the camera up and open the output video stream
#
    vs = VideoStream(src=0).start()
    time.sleep(2.0)
#    out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc(*'XVID'), fps, (300,210))

    while True:
      outputfile.write("VIDEO STREAM - Worker ID: " + str( mpi_rank ) + " - Waiting for MSG\n"  )

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
#      print("1483 - ", mission_time_txt, " - Rank ID = 1 waiting for MSG from master" )

      mpi_status = MPI.Status()
      mpi_data = mpi_comm.recv( source=MPI.ANY_SOURCE, tag=1, status = mpi_status )
      mpi_source_proc = mpi_status.Get_source()

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
#      print("1489 - ", mission_time_txt, " - Rank ID = 1 MSG received from Process: ", mpi_source_proc )

      if( mpi_data[0] == -1 ):
# received exit signal, leave While loop
        outputfile.write( "OTHERS - Worker ID: " + str( mpi_rank ) + " - Received exit signal.\n" )
        print("\nWorker ID: ", mpi_rank, " - Game over, man. Game over!" )

        break

      outputfile.write("VIDEO STREAM - Worker ID: " + str( mpi_rank ) + " - I'm alive and kicking\n"  )

#
# OPEN FRAME OUTPUT
#
      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt
      outputfile.write( "\n" + mission_txt + " - OPEN FRAME OUTPUT" )
      outputfile.write( "\n" + mission_txt + " - SETTING DHT, T: " + str( mpi_data[4] ) + " H: " + str(mpi_data[5]) )

#      print( "0907 - ", mission_time_txt, " - OPEN FRAME OUTPUT" )

      frame = frame_capture( args, vs )
      frame = vs.read()

      if frame is None:
        break
#      else:
#        out.write(frame)

#
# Resize the frame, blur it, and convert it to the HSV color space
#

      frame = imutils.resize(frame, width=frame_size)
      blurred = cv2.GaussianBlur(frame, (11, 11), 0)
      hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

#
# Construct a mask for the color "green", then perform a series
# of dilations and erosions to remove any small blobs left in the mask
#

      mask = cv2.inRange(hsv, greenLower, greenUpper)
      mask = cv2.erode(mask, None, iterations=NrIterations )
      mask = cv2.dilate(mask, None, iterations=NrIterations )


#
# find contours in the mask and initialize the
# current (x,y) center of the ball
#

      cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
      cnts = imutils.grab_contours(cnts)

#      print( "1652 - ", mission_time_txt, " - FIRST PRINT, cnts  = ", cnts )

      center = None

#
# Draw the center lines
#

      cv2.line( frame, (frame.shape[1]/3,0), (frame.shape[1]/3,frame.shape[0] ), (0,0,255), 1)
      cv2.line( frame, (frame.shape[1]*2/3,0), (frame.shape[1]*2/3,frame.shape[0] ), (0,0,255), 1)

      cv2.line( frame, (0,frame.shape[0]/2), (frame.shape[1],frame.shape[0]/2), (0,0,255), 1)
      cv2.line( frame, (0,frame.shape[0]*7/8), (frame.shape[1],frame.shape[0]*7/8), (0,0,255), 1)
      cv2.line( frame, (0,frame.shape[0]*6/8), (frame.shape[1],frame.shape[0]*6/8), (0,0,255), 1)

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt

      mission_time_txt = str( datetime.timedelta( seconds = ( time.time() - mission_start ) ) )
      mission_txt = "Mission Time: " + mission_time_txt

#      print( "1653 - time: ", time.time(), " - start: ", mission_start , " Mission Time: ", mission_time_txt) 

#
# Write on Frame
#
      if( mpi_source_proc == 3 ):
#        print("1563 ", mission_time_txt, " - Worker ID 1 received sensor Data from Worked ID 3: T = ", mpi_data[4], " - H = ", mpi_data[5])

        temperature = mpi_data[ 4 ]
        humidity = mpi_data[ 5 ]
        Bearing_txt = mpi_data[3]
        lat_txt = mpi_data[6]
        lng_txt = mpi_data[7]
        alt_txt = mpi_data[8]
        xaccl_txt = mpi_data[ 9 ]
        yaccl_txt = mpi_data[ 10 ]
        zaccl_txt = mpi_data[ 11 ]
        Temperature_txt = "Temperature (C): " + str( temperature )
        Humidity_txt = "Humidity (%): " + str( humidity )
        distance_txt = distance_txt_old

        temperature_old = temperature 
        humidity_old = humidity 
        Bearing_txt_old = Bearing_txt 
        lat_txt_old = lat_txt 
        lng_txt_old = lng_txt 
        alt_txt_old = alt_txt 
        xaccl_txt_old = xaccl_txt 
        yaccl_txt_old = yaccl_txt 
        zaccl_txt_old = zaccl_txt 
        Temperature_txt_old = Temperature_txt
        Humidity_txt_old = Humidity_txt

      elif( mpi_source_proc == 5 ):
#        print("1590 ", mission_time_txt, " - Worker ID 1 received sensor Data from Worked ID 5: ", )

        temperature = temperature_old 
        humidity = humidity_old 
        Bearing_txt = Bearing_txt_old
        lat_txt = lat_txt_old 
        lng_txt = lng_txt_old 
        alt_txt = alt_txt_old 
        xaccl_txt = xaccl_txt_old 
        yaccl_txt = yaccl_txt_old 
        zaccl_txt = zaccl_txt_old 
        Temperature_txt = Temperature_txt_old
        Humidity_txt = Humidity_txt_old 

        cm = mpi_data[ 2 ]  

        if( cm < 0 ):
          distance_txt = "Obstacle at: NOT DETECTED!"

        elif( cm > 300 ):
          distance_txt = "Obstacle at: >300 cm"

        else:   
          distance_txt = "Obstacle at: " + str( cm ) + " cm" 

        distance_txt_old = distance_txt 
      else:
        temperature = temperature_old
        humidity = humidity_old
        Bearing_txt = Bearing_txt_old
        lat_txt = lat_txt_old
        lng_txt = lng_txt_old
        alt_txt = alt_txt_old
        xaccl_txt = xaccl_txt_old
        yaccl_txt = yaccl_txt_old
        zaccl_txt = zaccl_txt_old
        Temperature_txt = Temperature_txt_old
        Humidity_txt = Humidity_txt_old
        distance_txt = distance_txt_old 

      write( frame, 0, 15, mission_txt )
      write( frame, 0, 30, target_lock_txt )
      write( frame, 0, 135, xaccl_txt)
      write( frame, 0, 150, yaccl_txt)
      write( frame, 0, 165, zaccl_txt)
      write( frame, 0, 180, direction_txt )
      write( frame, 0, 195, Temperature_txt )
      write( frame, 0, 210, Humidity_txt )
      write( frame, 0, 225, Bearing_txt )
      write( frame, 0, 240, lat_txt )
      write( frame, 0, 255, lng_txt )
      write( frame, 0, 270, alt_txt )
      write( frame, 0, 285, distance_txt )

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt
      outputfile.write( "\n" + mission_txt + " - END OPEN FRAME\n" )
      outputfile.write( "Xaccl_txt = " + xaccl_txt + "\n" )
      outputfile.write( "Yaccl_txt = " + yaccl_txt + "\n" )
      outputfile.write( "Zaccl_txt = " + zaccl_txt + "\n" )
      outputfile.write( "Direction_txt = " + direction_txt + "\n")
      outputfile.write( "Temperature_txt = " + Temperature_txt + "\n" )
      outputfile.write( "Humidity_txt = " + Humidity_txt +"\n")
      outputfile.write( "Bearing_txt = " + Bearing_txt + "\n")
      outputfile.write( "Latitude_txt = " + lat_txt +"\n")
      outputfile.write( "Longitude_txt = " + lng_txt + "\n")
      outputfile.write( "Altitude_txt= " + alt_txt + "\n")
      outputfile.write( "Distance_txt = " + distance_txt + "\n\n")

#      print("1707 - ", mission_time_txt, " - END FRAME REFRESH" )

#
# INVOKE ARTIFICIAL INTELLIGENCE ONLY AT DISCRETE TIME INTERVALS
#
      if( dbg_cfg == "basic" ):
        outputfile.write( "Checking AI Time...\n" )

      if( start_objtrk_time == 0 ):
        start_objtrk_time = time.time()

        if( dbg_cfg == "basic" ):
          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt
          outputfile.write( mission_txt + " - Checking if Objtrk_time of "+ objtrk_time + " s is already gone..." )

      if( time.time() - start_objtrk_time > float( objtrk_time ) ):
        if( dbg_cfg == "basic" ):
          outputfile.write( "\n" + mission_txt + " - Starting new AI Processing for Object Detection and Tracking...\n" )

        start_objtrk_time = 0

#
# Only proceed if at least one contour was found
#

        if len(cnts) > 0:
          target_lock_txt = "WE ARE ON THE PIPE. FIVE-BY-FIVE."

#          print("1032 - ", mission_time_txt, " - ", target_lock_txt )

#
# Find the largest contour in the mask,
# then use it to compute the minimumn enclosing circle and centroid
#

          c = max(cnts, key=cv2.contourArea)
          ((x, y), radius) = cv2.minEnclosingCircle(c)
          M = cv2.moments(c)
          center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

#
# Only proceed if the radius meets a minimum size
# and draw the vector to the center of the circle
#

          cv2.line( frame, ( int(x), int(y) ), (frame.shape[1]/2, frame.shape[0]*3/4 ), (0, 0, 255), 1 )

          if( dbg_cfg == "full" ):
            print( "radius: ", radius)

          if( radius > 5 ):

#
# Draw the circle and centroid on the frame,
# then update the list of tracked points
#

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
#            pts.appendleft(center)

#
# Calculate dX, dY from center frame
#

            dX = int( x ) - int( frame.shape[ 1 ]/2 )
            dY = int( y ) - int( frame.shape[ 0 ]/2 )

#
#  CHECK IN WHICH X-QUADRANT THE BALL IS
#

            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt

            if( ( int( x ) - int( frame.shape[ 1 ] / 3 ) < 0 ) ):
              if( dbg_cfg == "basic") :
                outputfile.write( "\nMaster Process " + mission_txt + " - H-Quadrant: 1 -- LEFT\n" )

              direction = "LEFT"

            elif( ( int( x ) - int( frame.shape[ 1 ]*2/3 ) > 0 ) ):
              if ( dbg_cfg == "basic" ):
                outputfile.write( "\nMaster Process " + mission_txt + " - H-Quadrant: 3 -- RIGHT\n" )

              direction = "RIGHT"

            else:
#
#  CHECK IN WHICH Y-QUADRANT THE BALL IS

              mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
              mission_txt = "Mission Time: " + mission_time_txt

              if( ( int( y ) - int( frame.shape[ 0 ]*6/8 ) < 0 ) ):
                if ( dbg_cfg == "basic" ):
                  outputfile.write("\n" + mission_txt + " Master Process - V-Quadrant: 1 -- FRONT\n" )

                direction = "FRONT"

              elif( ( int( y ) - int( frame.shape[ 0 ]*7/8 ) > 0 ) ):
                if ( dbg_cfg == "basic" ):
                  outputfile.write("\n993 Master Process " + mission_txt + " - V-Quadrant: 3  -- BACK\n")

                direction = "BACK"

              else:
                if ( dbg_cfg == "basic" ):
                  outputfile.write("\nV-Quadrant: 2 -- HALT\n")

                direction = "HALT"
                target_lock_txt = "TARGET LOCKED - ACTIVATING ROBOTIC ARM."

#
# MOVE THE BALL TO THE CENTER QUADRANT
#
          if ( dbg_cfg == "basic" ):
            outputfile.write( "AI Direction: "+ direction +"\n")

          direction_txt = "3 - Direction: " + direction
          outputfile.write( direction_txt + "\n")

          if( motor_cfg == '1' and mpi_size > 2 and manual_override == 0 ):
            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt

            outputfile.write("\n" + mission_txt + " - ROBOT move to " + direction + "\n" )
            mpi_data[ 12 ] = arrow_flash
            mpi_data[ 13 ] = direction
            mpi_data[ 14 ] = throttle
            mpi_data[ 16 ] = rotate_throttle

            mpi_comm.send( mpi_data, dest = 2, tag = 1 )
            mpi_status = MPI.Status()
            mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt
            outputfile.write(mission_txt + " - RETURNED move 1066\n" )
#            print("1184 - ", mission_time_txt, " - RETURNED MOVE")

#
# Show the movement deltas and the direction of movement on the frame
#

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt
          outputfile.write(mission_txt + " - Write on Frame\n" )

          if ( dbg_cfg == "full" ):
            print("dX:", dX)
            print("dY:", dY)

          radius_txt = "Radius: " + str( int( radius ) )

          if ( dbg_cfg == "full" ):
            print("Radius_txt: ", radius_txt )

          throttle_txt = "Throttle [f/s]: " + str( throttle ) + "%"
          rotate_throttle_txt = "Rotational Throttle [l/r]: " + str( rotate_throttle ) + "%"

          if ( dbg_cfg == "full" ):
            print("throttle_txt: ", throttle_txt )

          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
          mission_txt = "Mission Time: " + mission_time_txt
          outputfile.write(mission_txt + " - Having written on Frame\n" )

#
# end if len (cnts > 0 )
#

        else:
          radius_txt = "Radius: N/A"
          x = 0
	  y = 0
          dX = 0 
          dY = 0

#          write( frame, frame.shape[1] - 120, 15, "TARGET not locked" )
          target_lock_txt = "Target not locked."
          direction = "FRONT"

          if( dbg_cfg == "basic" ):
            outputfile.write( "1066 Master Process " + mission_time_txt + " - Contour not found, proceeding direction: " + direction + "\n")

            write( frame, 0, 180, "             " )
            direction_txt = "5 - Direction: " + direction
#            write( frame, 0, 180, direction_txt )
            outputfile.write( direction_txt + "\n")

            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt

            if( motor_cfg == '1' and mpi_size > 2 and manual_override == 0):
              outputfile.write( "\n1077 - " + mission_txt + " NO LOCK - move to " + direction + "\n" )
              mpi_data[ 12 ] = arrow_flash
              mpi_data[ 13 ] = direction
              mpi_data[ 14 ] = throttle
              mpi_data[ 16 ] = rotate_throttle

              mpi_status = MPI.Status()
              mpi_comm.send( mpi_data, dest = 2, tag = 1 )
              mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
            mission_txt = "Mission Time: " + mission_time_txt
            outputfile.write( "\n" + mission_txt + " RETURNED MOVE NO LOCK 1130\n" )

        write( frame, 0 , 60 , throttle_txt )
        write( frame, 0 , 75 , rotate_throttle_txt )
        write( frame, 0 , 105 , "x: {}, y: {}".format( int( x ) , int( y ) ) )
        write( frame, 0 , 120 , "dx: {}, dy: {}".format( dX, dY ) )
        write( frame, 0 , 90 , radius_txt )


#
# END INVOKE ARTIFICIAL INTELLIGENCE AT DISCRETE INTERVALS
#

#
# Show the frame to our screen and increment the frame counter
#

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt
      outputfile.write( "\n" + mission_txt + " Show the frame" )

      cv2.imshow("Frame", frame)

      NavMod = cv2.waitKey(1) & 0xFF

      if( NavMod == ord("q") ):
#
# End the Video Stream and Send termination signal
#
#        vs.release()
#        out.release()
#        cv2.destroyAllWindors()

	print( "Worker ID = 1 - Game over, man. Game over! ")
 
        mpi_data[0] = -1
        print("Worker ID 1 - Send Termination Signal To Master" )
        mpi_comm.send( mpi_data, dest = 0, tag = 1 )
 
        if( mpi_size > 2 ):
          print("Worker ID 1 - Send Termination Signal To Worker ID 2" )
          mpi_comm.send( mpi_data, dest = 2, tag = 1 )

        print("Worker ID 1 - Terminated Process" )

        break

      elif( NavMod == ord("f") ):
          if ( dbg_cfg == "full" ):
            print("I'm here.... - NavMode: f" )

          if throttle < 100:
            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	    outputfile.write( "\n" + mission_txt + " - PRESSED F HOTKEY: speed increased by 5%" )
            print( mission_time_txt, " - PRESSED F HOTKEY: speed increased by 5%")

            throttle = throttle + 5 

      elif( NavMod == ord("s") ):
          if ( dbg_cfg == "full" ): 
            print("I'm here.... - NavMode: s" )

          if throttle > 10:
            mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	    outputfile.write( "\n" + mission_txt + " - PRESSED S HOTKEY: speed decreased by 5%" )
            print( mission_time_txt, " - PRESSED S HOTKEY: speed decreased by 5%")

            throttle = throttle - 5 

      elif( NavMod == ord("r") ):
        if rotate_throttle > 10:
          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	  outputfile.write( "\n" + mission_txt + " - PRESSED R HOTKEY: rotational speed decreased by 5%" )
          print( mission_time_txt, " - PRESSED R HOTKEY: rotational speed decreased by 5%")

	  rotate_throttle = rotate_throttle - 5

      elif( NavMod == ord("l") ):
        if rotate_throttle < 100:
          mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	  outputfile.write( "\n" + mission_txt + " - PRESSED L HOTKEY: rotational speed increased by 5%" )
          print( mission_time_txt, " - PRESSED L HOTKEY: rotational speed increased by 5%")
          direction = "LEFT"

	  rotate_throttle = rotate_throttle + 5

      elif( NavMod == ord("z") ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - PRESSED Z HOTKEY" )
        print( mission_time_txt, " - PRESSED Z HOTKEY")
        direction = "LEFT"
        mpi_data[ 12 ] = arrow_flash
        mpi_data[ 13 ] = direction
        mpi_data[ 14 ] = throttle
        mpi_data[ 16 ] = 50 

        mpi_status = MPI.Status()
        mpi_comm.send( mpi_data, dest = 2, tag = 1 )
        mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

      elif( NavMod == ord("x") ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
        print( mission_time_txt, " - PRESSED X HOTKEY")
	outputfile.write( "\n" + mission_txt + " - PRESSED X HOTKEY" )
        direction = "RIGHT"
        mpi_data[ 12 ] = arrow_flash
        mpi_data[ 13 ] = direction
        mpi_data[ 14 ] = throttle
        mpi_data[ 16 ] = 50 

        mpi_status = MPI.Status()
        mpi_comm.send( mpi_data, dest = 2, tag = 1 )
        mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

      elif( NavMod == ord("g") ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - PRESSED G HOTKEY" )
        print( mission_time_txt, " - PRESSED G HOTKEY")
        direction = "FRONT"
        mpi_data[ 12 ] = arrow_flash
        mpi_data[ 13 ] = direction
        mpi_data[ 14 ] = 75
        mpi_data[ 16 ] = rotate_throttle

        mpi_status = MPI.Status()
        mpi_comm.send( mpi_data, dest = 2, tag = 1 )
        mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

      elif( NavMod == ord("m" ) ):
        manual_override = 1
        mpi_data[ 17 ] = manual_override
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - MANUAL OVERRIDE: ON" )
        print( mission_time_txt, " - MANUAL OVERRIDE: ON")

      elif( NavMod == ord("a") ):
        manual_override = 0
        mpi_data[ 17 ] = manual_override
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - MANUAL OVERRIDE: OFF" )
        print( mission_time_txt, " - MANUAL OVERRIDE: OFF")
  
      elif( NavMod == ord("h" ) ):
        manual_override = -1 
        mpi_data[ 17 ] = manual_override
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - PRESSED H HOTKEY" )
        print( mission_time_txt, " - ROBOT HALTED")

        direction = "HALT"

      elif( NavMod == ord("1" ) and manual_override == 1 ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - MANUAL OVERRIDE: LEFT" )
        print( mission_time_txt, " - MANUAL OVERRIDE: LEFT")

        direction = "LEFT"
        mpi_data[ 12 ] = arrow_flash
        mpi_data[ 13 ] = direction
        mpi_data[ 14 ] = throttle
        mpi_data[ 16 ] = rotate_throttle

        mpi_status = MPI.Status()
        mpi_comm.send( mpi_data, dest = 2, tag = 1 )
        mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

        direction = "HALT"

      elif( NavMod == ord("2" ) and manual_override == 1 ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - MANUAL OVERRIDE: RIGHT" )
        print( mission_time_txt, " - MANUAL OVERRIDE: RIGHT")

        direction = "RIGHT"
        mpi_data[ 12 ] = arrow_flash
        mpi_data[ 13 ] = direction
        mpi_data[ 14 ] = throttle
        mpi_data[ 16 ] = rotate_throttle

        mpi_status = MPI.Status()
        mpi_comm.send( mpi_data, dest = 2, tag = 1 )
        mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

        direction = "HALT"

      elif( NavMod == ord("9" ) and manual_override == 1 ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - MANUAL OVERRIDE: FRONT" )
        print( mission_time_txt, " - MANUAL OVERRIDE: FRONT")

        direction = "FRONT"
        mpi_data[ 12 ] = arrow_flash
        mpi_data[ 13 ] = direction
        mpi_data[ 14 ] = throttle
        mpi_data[ 16 ] = rotate_throttle

        mpi_status = MPI.Status()
        mpi_comm.send( mpi_data, dest = 2, tag = 1 )
        mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

        direction = "HALT"

      elif( NavMod == ord("0" ) and manual_override == 1 ):
        mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
	outputfile.write( "\n" + mission_txt + " - MANUAL OVERRIDE: BACK" )
        print( mission_time_txt, " - MANUAL OVERRIDE: BACK")

        direction = "BACK"
        mpi_data[ 12 ] = arrow_flash
        mpi_data[ 13 ] = direction
        mpi_data[ 14 ] = throttle
        mpi_data[ 16 ] = rotate_throttle

        mpi_status = MPI.Status()
        mpi_comm.send( mpi_data, dest = 2, tag = 1 )
        mpi_data = mpi_comm.recv( source= 2, tag = 1, status=mpi_status )

        direction = "HALT"

#
# Continue the video streaming
#

      write( frame, 0, 200, "NavMod: " + str(NavMod) )
      counter += 1

      mission_time_txt = str( datetime.timedelta( seconds = time.time() ) )
      mission_txt = "Mission Time: " + mission_time_txt
      outputfile.write( "\n" + mission_txt + " END SHOW the frame\n" )

# sending back to Master "OK" signal
   
#      print( "1895 - ", mission_time_txt, " - Worker ID 1 sending back OK signal to Master")
      mpi_data[0] = 0
      mpi_comm.send( mpi_data, dest = 0, tag = 1 )

  else:
#
# ALL OTHER MPI PROCESSES
#
    while True:
      outputfile.write("OTHERS - Worker ID: " + str( mpi_rank ) + " - Waiting for MSG\n"  )

      mpi_data = mpi_comm.recv( source=0, tag=1 )

      if( mpi_data[0] == -1 ):
# received exit signal, leave While loop
        outputfile.write( "OTHERS - Worker ID: " + str( mpi_rank ) + " - Received exit signal.\n" )
        print("\nWorker ID: ", mpi_rank, " - Game over, man. Game over!" )

        break

      outputfile.write("OTHERS - Worker ID: " + str( mpi_rank ) + " - I'm alive and kicking\n"  )

# sending back to Master "OK" signal

      mpi_data[0] = 0
      mpi_comm.send( mpi_data, dest = 0, tag = 1 )

#
# ALL MPI PROCESSES TERMINATE NORMALLY
#

  outputfile.close()
  GPIO.cleanup()

#
# END OF MAIN FUNCTION 
#

#########################################
#					#
#	INVOKE MAIN FUNCTION		#
#					#
#########################################

if __name__ == "__main__":
  main()
