_VERSION_ = "0.77"

#Seems like one of the libraries is slowing down startup so for now I'm just printing so I know whether its the
#program or the pi having issues. Probably the pyorbital library.
print("Loading libraries...")
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
from time import sleep, time
from datetime import datetime, timedelta
from urllib.request import urlretrieve
from pyorbital import orbital
import threading, re, os, os.path, pytz
from difflib import SequenceMatcher
from json import load as json_load
from json.decoder import JSONDecodeError
from math import cos, sin, radians

#NEEDED FOR SATELLITE TRACKING!



#Defaults. Actual coords read in from file gpscoords.txt in the same dir as this file.
ANTENNA_GPS_LAT = 34
ANTENNA_GPS_LONG = -117
ANTENNA_GPS_ALT = 0 # in kilometers


#Minimum elevation before we start and stop our tracking. 0 is perfectly at the horizon.
#Most location have obstructions around the horizon so that part of the pass is wasted.
#If you know what elevation you start getting a clear line-of-sight then enter that below.
TRACKING_START_ELEVATION = 0

# Filter out satellite passes with max elevations below this number (in degrees) from the passlist
PASSLIST_FILTER_ELEVATION = 15


##GPIO PIN MAPPINGS:

#Motor Control
AZ_AIN1 = 9
AZ_AIN2 = 11
AZ_PWMA = 12 # PWM0
EL_BIN1 = 16
EL_BIN2 = 20
EL_PWMB = 13


#End stops
EL_ENDSTOP_BLUE = 3
EL_ENDSTOP_YELLOW = 4

AZ_MAG_SENSOR = 2

#Rotary Sensor (positioning)
#Azimuth
AZ_POS_OUTA = 5
AZ_POS_OUTB = 6
#Elevation
EL_POS_OUTA = 0
EL_POS_OUTB = 1

# Max allowed input angle. Ideally we would use hard endstops wired to the motor controler to save us from overrun
# Without that, maybe this will save us.
MAX_AZ_INPUT_ANGLE = 360
MAX_EL_INPUT_ANGLE = 180 #Elevation axis can actually go slightly below 0 or 180 but its not needed
PWM_FREQUENCY = 2000
SLOWDOWN_DEGREES = 3 #Slow down when within this many degrees of target

#Roughly how many degrees rotation for each tick.
# Azimuth axis
AZ_ENCODER_DRIVE_GEAR_T = 10
AZ_DRIVEN_GEAR_T = 58
AZ_ENCODER_POINTS = 400 # How many pulses per full rotation of the encoder
AZ_GEAR_RATIO = AZ_DRIVEN_GEAR_T/AZ_ENCODER_DRIVE_GEAR_T
AZ_POINTS_PER_MAST_REV = AZ_GEAR_RATIO * AZ_ENCODER_POINTS
AZ_ANGLE_TICK = 360/AZ_POINTS_PER_MAST_REV
#AZ_ANGLE_TICK = 0.15517
# Elevation axis
EL_ENCODER_DRIVE_GEAR_T = 12
EL_DRIVEN_GEAR_T = 60
EL_ENCODER_POINTS = 400
EL_GEAR_RATIO = EL_DRIVEN_GEAR_T/EL_ENCODER_DRIVE_GEAR_T
EL_POINTS_PER_MAST_REV = EL_GEAR_RATIO * EL_ENCODER_POINTS
EL_ANGLE_TICK = 360/EL_POINTS_PER_MAST_REV
#EL_ANGLE_TICK = 0.18
#For 0.1 degree accuracy on the Elevation axis we need what tooth count on the encoder wheel?
# We would need 360/0.1 = 3600 ticks per full revolution
# For a 400 tick per revolution encoder we need a gear ratio of 3600/400 = 9:1
# With a 60-tooth main gear, the encoder gear needs to be 60/9 = 6.66666 teeth
#Probably need a finer precision rotary encoder or larger main gear.
#Where do we park the elevation and azimuth axes after calibrating?
EL_PARK_ANGLE = 90 # Straight up and down
AZ_PARK_ANGLE = 0 # Park azimuth at 0 (which should be due north)


#Trying to find a ratio so I don't need to remember the exact way to specify meteor-m2 3
SATNAME_MATCH_RATIO = 0.89

#How many days between updating TLE data?
TLE_UPDATE_FREQ_DAYS = 3


RUNFOLDER = "/mnt/SatStationData/SatAntennaControl/src"
#Path doesnt work because its using the symlink's folder instead of the actual folder
TLEFILEPATH = os.path.join(RUNFOLDER, "weather.txt")
GPSFILEPATH = os.path.join(RUNFOLDER, "gpscoords.txt")
#Name of the file to be used for position recovery data. Prefix it with a period to hide it in linux.
POSITION_RECOVERY_FILE_PATH = os.path.join(RUNFOLDER, ".active_position")


class RotaryEncoder:
    CLOCKWISE=1
    ANTICLOCKWISE=2
    rotary_a = 0
    rotary_b = 0
    rotary_c = 0
    last_state = 0
    direction = 0
    # Initialise rotary encoder object
    # ANGLE_TICK is the defined amount of angular movement per tick of the encoder
    # This is derived through the gear ratios of driven gear and encoder drive gear as
    # well as ticks per revolution of the encoder.
    def __init__(self, pinA, pinB, ANGLE_TICK):
        self.ANGLE_TICK = ANGLE_TICK
        self.curangle = 0.0
        self.curtick = 0
        self.pinA = pinA
        self.pinB = pinB
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pinA, GPIO.BOTH, callback=self.switch_event)
        GPIO.add_event_detect(self.pinB, GPIO.BOTH, callback=self.switch_event)

    def reset_position(self):
        self.curangle = 0.0
        self.curtick = 0

    def set_encoder_angle(self, angle):
        self.curangle = angle
        self.curtick = round(angle / self.ANGLE_TICK)

    def switch_event(self, _):
        if GPIO.input(self.pinA):
            self.rotary_a = 1
        else:
            self.rotary_a = 0
        if GPIO.input(self.pinB):
            self.rotary_b = 1
        else:
            self.rotary_b = 0
        self.rotary_c = self.rotary_a ^ self.rotary_b
        new_state = self.rotary_a * 4 + self.rotary_b * 2 + self.rotary_c * 1
        delta = (new_state - self.last_state) % 4
        self.last_state = new_state
        if delta == 1:
            if self.direction == self.CLOCKWISE:
                self.curangle += self.ANGLE_TICK
                self.curtick += 1
                #print("Clockwise: %s" % self.curangle)
            else:
                self.direction = self.CLOCKWISE
        elif delta == 3:
            if self.direction == self.ANTICLOCKWISE:
                self.curangle -= self.ANGLE_TICK
                self.curtick -= 1
                #print("Anticlockwise: %s" % self.curangle)
            else:
                self.direction = self.ANTICLOCKWISE
    def getCurrentTick(self): return self.curtick
    def getCurrentAngle(self): return round(self.curangle, 3) #Round curangle to 3 digits to prevent weird scientific notation output for small values near 0

class ElMotorControl(threading.Thread):
    def __init__(self, BIN1=EL_BIN1, BIN2=EL_BIN2, PWMB=EL_PWMB, YELLOW_ENDSTOP=EL_ENDSTOP_YELLOW, BLUE_ENDSTOP=EL_ENDSTOP_BLUE, recovery_data=None):
        super(ElMotorControl, self).__init__()
        self.name = "ElMotorControlThread"
        self.encoder = RotaryEncoder(EL_POS_OUTA, EL_POS_OUTB, EL_ANGLE_TICK)
        self.bin1 = BIN1
        self.bin2 = BIN2
        self.pwmb = PWMB
        self.yellow_endstop = YELLOW_ENDSTOP
        self.blue_endstop = BLUE_ENDSTOP
        self.recovery_data = recovery_data
        #Setup pins and PWM channel
        GPIO.setup(self.bin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.bin2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pwmb, GPIO.OUT, initial=GPIO.LOW)
        self.EL_motor_PWM_out = GPIO.PWM(self.pwmb, PWM_FREQUENCY)       #create PWM instance with frequency of PWM_FREQUENCY hz
        self.EL_motor_PWM_out.start(0) #Start PWM0 output at 0 duty cycle
        #Elevation axis is using a pair of physical endstop switches that are pulled high and go low when activated
        GPIO.setup(self.yellow_endstop, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.blue_endstop, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Feel like I need to explain the below angle_fuzz. In an ideal world, the end stops for the elevation axis would be
        # perfectly symmetrical in their positioning, and going from one to the other and splitting the difference should be
        # exactly 90 degrees. Well on the first test it wasn't 90 degrees, it was slightly off. So I just kept trying slightly
        # different values of angle_fuzz until it came out level, and thats the number below. Its something to do with the
        # screw and nut positions on the mechanical endstops. I think a magnetic endstop would be more precise between the two.
        # Theoretically a nut moves a specific amount per turn, as defined by the thread pitch. So screwing the nuts the
        # same number of times should line them up perfectly, but it doesn't. Manufacturing tolerances arent good enough maybe.
        self.angle_fuzz = 89.5 #What angle to set to after we go to what we "think" is 90 degrees.

        # New method of moving, instead of being prompted a single angle and moving towards it we will constantly evaluate
        # what the commanded angle is vs our current angle. Speed and direction to be calculated every loop iteration.
        self.commanded_angle = EL_PARK_ANGLE #Starting at park since we go there after calibrating.
        self.quitting = False
        self.homed = False # So we can check when the homing processes ends
        self.stop_movement = False # For estop
        self.speed = 100 # So we can modulate move speed from the command line
        self.is_moving = False


    #Start of threaded operation. We'll home the axis and then begin the move_to_commanded_angle() loop
    def run(self):
        if self.recovery_data is None:
            self.calhome()
            print("\n\n")
        else:
            print("Bypassing homing and using recovery data on the elevation axis. Pos: %s [%s]" % (self.recovery_data["ElevationDeg"], self.recovery_data["ElevationTick"]))
            self.encoder.curangle = self.recovery_data["ElevationDeg"]
            self.commanded_angle = self.encoder.curangle
            self.encoder.curtick = self.recovery_data["ElevationTick"]
            self.homed = True
        #We should be parked at 90 degrees at this point or at a recovered position. Now we start our loop.
        #How many operations per second should the loop run at? Default value is 100 times per second. Could probably get away with less.
        ops = 100
        while self.quitting is False:
            if self.homed is True and self.stop_movement is False:
                self.move_to_commanded_angle()
            #print(" "*100, end="\r")
            #print("Current angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
            sleep(1.0/ops)

        print("ElMotorControl thread received quit signal. Exiting thread...")
        self.quitthread()

    def quitthread(self):
        #If we were called externally then set the internal quitting flag true so the loop can stop
        self.quitting = True
        #Stop the movement follower function and go to park angle
        self.stop_movement = True
        self.move_to_angle(EL_PARK_ANGLE)
        #Stop the PWM channel or it causes problems when quitting python
        #Final app exit will handle the GPIO.cleanup()
        self.EL_motor_PWM_out.stop()

    def move_to_commanded_angle(self):
        # This function is constantly called as a background thread, constantly evaluating and commanding movements.
        #Determine which way to turn the motor
        if self.commanded_angle < self.encoder.curangle:
            movefun = self.move_motor_dir1
            endstop = self.blue_endstop
            #Needed sign information for old function but new function probably could get away with abs() instead. This works though so meh.
            anglesign = -1
        elif self.commanded_angle > self.encoder.curangle:
            movefun = self.move_motor_dir2
            endstop = self.yellow_endstop
            anglesign = 1
        else: #Commanded angle and current angle are the same, don't do anything. Maybe just make sure motor is stopped...
            self.stop_motor()
            return

        #Now that we know what endstop to check, lets make sure we havent crashed into it already
        if GPIO.input(endstop) == 0:
            self.stop_motor()
            return

        #With our anglesign figured out we can do a finer check. Maybe we are less than or greater than our target
        #but its possible we are within the accuracy of our encoder as well. Current tick is too low but one tick
        #more is over the target. Here we test that by comparing the difference in angle to the smallest increment
        #of our rotary encoder (self.encoder.ANGLE_TICK)
        anglediff = anglesign * (self.commanded_angle - self.encoder.curangle)
        if abs(anglediff) < self.encoder.ANGLE_TICK:
            self.stop_motor()
            return # Can't reach exact commanded value because its between the smallest increment we can detect.

        #TODO Because our encoder is incremental in nature, there is a granularity to the measurements we can take.
        #This means sometimes we are a small bit under an ANGLE_TICK away from the target.
        #There could be a situation where stopping before the target, even if less than an ANGLE_TICK away, could
        #be worse than slightly overshooting. Say we command to move from 0 degrees up to 45. We landed slightly
        #early at 44.86 degrees (tick 249). Thats a 0.14 degree diff but our Elevation angle tick is 0.18.
        #If we overshoot to tick 250 we land at 45.04, slightly over but much closer to our target.
        # It should be 250 * 0.18 = exactly 45 but because theres a slight difference in apparently angle our ticks
        # arent exactly aligned anymore so the math gets weird.
        #Basically, we need to determine what side of our target is closest, and then move to that TICK.

        #We know what direction we should move and how far we are, now lets figure out what speed we should move at.
        #The old move_to_angle() function has some good constants that we can follow for now.
        #Ideally we would use a single function or proportionality constant to set the speed based on distance to target.
        #That would require tuning and I know those duty cycle constants more or less work. Only minor adjustments needed.
        movespeed = self.speed # PWM duty cycle is between 0 and 100

        #After testing it seems the minimum PWM value needed to raise the boom from horizontal is 20.
        #At el45 the minimum PWM val needed to move is 15
        #At el70 the minimum PWM val needed to move is only 5
        if anglediff > 5:
            pass # no slowdown, 100 duty cycle
        elif anglediff > 2: # Between 5 and 2 degrees, first slowdown
            movespeed = 20
        elif anglediff > 1: #Between 1 and 2 degrees, second slowdown
            movespeed = 15
        elif anglediff >= self.encoder.ANGLE_TICK*3: #between a degree difference and 3 ANGLE_TICKs
            movespeed = 10
        else: # Between 3 ticks and our target we really crawl slowly
            movespeed = 5 #Slowest move speed possible cant move the elevation axis from 0 or 180

        #PWM values above are what are needed for smooth movement at nearly horizontal.
        #The maths below is to tune that value down as the elevation goes up so we end up
        #around 5 PWM val at the lowest movement speed at vertical elevation and around 20 at
        #perfectly horizontal.
        #Tried both, sin changes too much at low elevation and cos changes too much at high elevation
        angle_compensation = cos(radians(self.encoder.curangle)) * 15
        movespeed += angle_compensation
        #After about 50 degrees the compensation values aren't enough and the PWM vals get bigger than they need to be
        #At 70 degrees we should be at 5 but we aren't
        #So thats what this -2 is for, its tuned to be about perfect over 60 degrees with this
        #Between 30 degrees before vertical and vertical, knock it down a couple numbers to match the required PWM constants found with the tgo command.
        if 60 < self.encoder.curangle < 120:
            movespeed -= 2


        #angle_compensation = sin(radians(self.encoder.curangle)) * 15
        #movespeed -= angle_compensation


        """
        #Small movements at low elevations are hard to do with normal PWM constants, so we buff them up a bit to help.
        if self.encoder.curangle < ELEVATION_CRIT_ANGLE and anglediff < 5:
            movespeed = round(movespeed * ELEVATION_CRIT_MULTI)
        elif self.encoder.curangle > 80 and anglediff < 5:
            movespeed = round(movespeed * 0.5)
        """

        #Ok now we know what motor to activate and how fast it should move.
        movefun(movespeed)

    def stop_motor(self):
        self.EL_motor_PWM_out.ChangeDutyCycle(0)
        GPIO.output(self.bin1, GPIO.LOW)
        GPIO.output(self.bin2, GPIO.LOW)
        self.is_moving = False

    #This moves the elevation axis towards the blue endstop
    def move_motor_dir1(self, speed=100):
        #Clamp PWM vals to be safe
        speed = max(min(100, speed), 0)
        self.EL_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.bin1, GPIO.HIGH)
        GPIO.output(self.bin2, GPIO.LOW)
        self.is_moving = True

    #This moves the elevation axis towards the yellow endstop
    def move_motor_dir2(self, speed=100):
        speed = max(min(100, speed), 0)
        self.EL_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.bin1, GPIO.LOW)
        GPIO.output(self.bin2, GPIO.HIGH)
        self.is_moving = True

    def move_until_stop(self, direction, speed=100):
        if direction == 1:
            #direction 1 moves towards the blue endstop
            self.move_motor_dir1(speed)
            es = self.blue_endstop
            nicename = "blue"
        elif direction == 2:
            self.move_motor_dir2(speed)
            #direction 2 moves towards the yellow endstop
            es = self.yellow_endstop
            nicename = "yellow"
        else:
            raise(Exception("Syntax error with move_until_stop call. Check line 165."))
        stopstate = GPIO.input(es)
        angle_every_seconds = 0.1
        curtime = time()
        lasttime = curtime
        while stopstate > 0:
            sleep(0.01)
            stopstate = GPIO.input(es)
            curtime = time()
            if curtime - lasttime > angle_every_seconds:
                lasttime = curtime
                print(" "*50, end="\r")
                print("Current elevation angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
        print("\nHit the %s endstop on the elevation axis, stopping motor!" % nicename)
        self.stop_motor()

    def move_to_angle(self, angle, speed=100, bypspeedctrl=False):
        #This function simply runs the motor in the correct direction until the curangle of the encoder matches the supplied angle
        #Determine which way to turn the motor
        if angle < self.encoder.curangle:
            movefun = self.move_motor_dir1
            endstop = self.blue_endstop
            nicename = "blue"
            #Could save a few lines of code ignoring the sign information, but we need it to be sure we didnt go past the target.
            anglesign = -1
        else:
            movefun = self.move_motor_dir2
            endstop = self.yellow_endstop
            nicename = "yellow"
            anglesign = 1


        #For nicer output of current angle
        angle_every_seconds = 0.1
        curtime = time()
        lasttime = curtime

        movefun(speed) #Start moving at speed
        try:
            while anglesign * (angle - self.encoder.curangle) > self.encoder.ANGLE_TICK: #Keeps stopping 1 tick too late, so using AZ_ANGLE_TICK instead of 0.
                #How a dummy implements the P in PID
                #Slow down when we are close. Say 25% speed when within 5 degrees?
                if anglesign * (angle - self.encoder.curangle) < SLOWDOWN_DEGREES:
                    if not bypspeedctrl: self.EL_motor_PWM_out.ChangeDutyCycle(50)
                #Slowdown again when we are SLOWDOWN_DEGREES/2 degrees away
                if anglesign * (angle - self.encoder.curangle) < SLOWDOWN_DEGREES/2:
                    if not bypspeedctrl: self.EL_motor_PWM_out.ChangeDutyCycle(30)
                #Last slowdown for super fine movements
                if anglesign * (angle - self.encoder.curangle) < 1:
                    if not bypspeedctrl: self.EL_motor_PWM_out.ChangeDutyCycle(25)

                curtime = time()
                if curtime - lasttime > angle_every_seconds:
                    lasttime = curtime
                    print(" "*100, end="\r") # Blank out the line before reprinting, otherwise old crap can be left behind
                    print("Current angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
                #Check endstops too
                if GPIO.input(endstop) == 0 :
                    print(" "*100, end="\r")
                    print("Hit %s end-stop, stopping movement." % nicename)
                    self.stop_motor()
                    break
                sleep(0.01)
        except KeyboardInterrupt: #Stop movement. Other options to stop mid-move required threads and crazy keyboard interrupt stuff.
            print("\nReceived emergency stop command. Ending movement and allowing motors to wind-down...")
            self.stop_motor()
            sleep(1)
        self.stop_motor()
        print(" "*100, end="\r")
        print("Final angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()))

    #Calibrate home will find the blue endstop, then back up and slowly find the precise location of the blue endstop.
    #It will then set its angle var to 0 and move to the other endstop. When it finds the other endstop it again backs
    #up to slowly find the precise location of the yellow endstop. When it finds it, it takes the total number of ticks
    # and divides it by 2 to find the middle. If the endstops are symmetrical in their positions, that middle should be 90 degrees vertical.
    def calhome(self):
        print("Finding elevation axis blue endstop...")
        self.homed = False
        self.move_until_stop(direction=1)
        self.move_motor_dir2()
        sleep(1)
        self.stop_motor()
        self.move_until_stop(direction=1, speed=5)
        print("Found precise location of blue endstop. Now finding yellow endstop...")
        self.encoder.reset_position()
        self.move_until_stop(direction=2)
        self.move_motor_dir1()
        sleep(1)
        self.stop_motor()
        self.move_until_stop(direction=2, speed=5)
        print("Found precise location of yellow endstop at %s ticks from blue endstop. Now parking at 90-degrees (vertical)..." % self.encoder.curtick)
        #Ok we know exactly where each endstop is. If they are symmetrically placed then theoretically the middle
        #position between the two should be straight vertical, 90 degrees. The problem is, thats nice 90 degrees from
        #our current position. We know the tick number we need to hit, but our function needs an angle. We can
        #back-compute the apparent angle by simple math using the tick we KNOW is 90 degrees.
        #That math is simply: angle = tick_number * angle_tick
        halfway_tick = self.encoder.curtick/2
        self.move_to_angle(halfway_tick * self.encoder.ANGLE_TICK)
        #NOW that we are pretty sure we're at 90 degrees we can reset the encoder object so it also thinks its at 90 degrees.
        #Turns out with my specific system theres some slop SOMEWHERE and its actually about 89 degrees. Some minute different in the endstops. Gotta find a better way.
        #self.encoder.set_encoder_angle(89)
        self.encoder.set_encoder_angle(self.angle_fuzz)
        self.commanded_angle = self.encoder.curangle
        self.homed = True



class AzMotorControl(threading.Thread):
    def __init__(self, AIN1=AZ_AIN1, AIN2=AZ_AIN2, PWMA=AZ_PWMA, he_sensor=AZ_MAG_SENSOR, recovery_data=None):
        super(AzMotorControl, self).__init__()
        self.name = "AzMotorControlThread"
        self.encoder = RotaryEncoder(AZ_POS_OUTA, AZ_POS_OUTB, AZ_ANGLE_TICK)
        self.ain1 = AIN1
        self.ain2 = AIN2
        self.pwma = PWMA
        self.he_sensor = he_sensor
        self.recovery_data = recovery_data
        #Setup needed pins and start PWM channel
        GPIO.setup(self.ain1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ain2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pwma, GPIO.OUT, initial=GPIO.LOW)
        self.AZ_motor_PWM_out = GPIO.PWM(self.pwma, PWM_FREQUENCY)       #create PWM instance with frequency of PWM_FREQUENCY hz
        self.AZ_motor_PWM_out.start(0) #Start PWM0 output at 0 duty cycle
        #Azimuth is using a single magnetic hall-effect sensor for homing
        #The sensor is high when not near a magnet and goes low when near
        GPIO.setup(self.he_sensor, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.commanded_angle = AZ_PARK_ANGLE #Starting at park since we go there after calibrating.
        self.quitting = False
        self.homed = False
        self.stop_movement = False
        self.speed = 100
        self.is_moving = False
        self.manual_offset_diff = 0 # Track our manual azimuth offset difference so we can park properly

    #Start of threaded operation. We'll home the axis and then begin the move_to_command_angle() loop
    def run(self):
        #Allow bypass of homing if we have recovery data
        if self.recovery_data is None:
            self.find_home()
            print("\n\n")
        else:
            print("Bypassing homing and using recovery data on the azimuth axis. Pos: %s [%s]" % (self.recovery_data["AzimuthDeg"], self.recovery_data["AzimuthTick"]))
            self.encoder.curangle = self.recovery_data["AzimuthDeg"]
            self.commanded_angle = self.encoder.curangle
            self.encoder.curtick = self.recovery_data["AzimuthTick"]
            self.homed = True
        #Would 10 loop iterations a second really be too slow? Maybe 50?
        #100 per second uses about 10% CPU on the pi4 which is kinda crazy.
        #That goes down to around 4% with spikes to 8% if you estop
        ops = 100
        while self.quitting is False:
            #If we decided to re-home the axis, we don't want our loop overriding its commands. We also don't really want
            #to have to destroy and re-create the thread so this just stops our commands while being homed.
            if self.homed is True and self.stop_movement is False:
                self.move_to_commanded_angle()
            #print(" "*100, end="\r")
            #print("Current angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
            sleep(1.0/ops)

        print("AzMotorControl thread received quit signal. Exiting thread...")
        self.quitthread() #Is this call necessary? Can we get here without someone calling quitthread() first?

    def quitthread(self):
        #If we were called externally then set the internal quitting flag true so the loop can stop
        self.quitting = True
        #Its super important to properly unwind the mast before stopping until we hav absolute positioning data.
        #Without a proper unwind we could destroy the entire wiring harness.
        self.unwind_mast()
        #Stop the PWM channel or it causes problems when quitting python
        #Final app exit will handle the GPIO.cleanup()
        self.AZ_motor_PWM_out.stop()

    def move_to_commanded_angle(self):
        #Keep the angles between 360
        self.encoder.curangle %= 360
        self.commanded_angle %= 360
        #We don't actually use the ticks for tracking so we will keep that the same for now.
        #Maybe we could even use that tick number to track the true position and unwind later.
        # This function is constantly called as a background thread, constantly evaluating and commanding movements.
        actualdistance = self.commanded_angle - self.encoder.curangle
        #TODO Would a subtick stop check here help the CPU usage much?
        #Looks like we do a LOT of math before we figure out we arent moving.
        #No wonder it uses 10% just sitting there.
        if abs(actualdistance) < 180: # shorter movements are normal movements
            #Determine which way to turn the motor
            if self.commanded_angle > self.encoder.curangle:
                movefun = self.move_motor_dir1
                #Could save a few lines of code ignoring the sign information, but we need it to be sure we didnt go past the target.
                anglesign = 1
            elif self.commanded_angle < self.encoder.curangle:
                movefun = self.move_motor_dir2
                anglesign = -1
            else: #Commanded angle and current angle are the same, don't do anything. Maybe just make sure motor is stopped...
                self.stop_motor()
                return

            anglediff = anglesign * (self.commanded_angle - self.encoder.curangle)
        else: # moving the opposite way and crossing the 0/359 line is shortest movement
            #Determine which way to go. If actualdistance is positive we move counterclockwise, negative we go clockwise
            if actualdistance > 0: # counterclockwise
                movefun = self.move_motor_dir2
            else:
                movefun = self.move_motor_dir1
            # Positive or negative doesn't matter, we can't overrun the target anyway since the next loop iteration would
            # use the new numbers and determine to move the opposite way. And it slows down before target anyway so it should be fine.
            anglediff = 360 - abs(actualdistance)

        if abs(anglediff) < self.encoder.ANGLE_TICK:
            self.stop_motor()
            return # Can't reach exact commanded value because its between the smallest increment we can detect.

        movespeed = self.speed # PWM duty cycle is between 0 and 100
        if anglediff > 5:
            pass # no slowdown, 100 duty cycle
        elif anglediff > 2: # Between 5 and 2 degrees, first slowdown
            movespeed = 30 # Should be 25% here
        elif anglediff > 1: #Between 1 and 2 degrees, second slowdown
            movespeed = 15 #Should be 6.25%
        elif anglediff >= self.encoder.ANGLE_TICK*3: #between a degree difference and 3 ANGLE_TICKs
            movespeed = 8
        else: # Between 3 ticks and our target we really crawl slowly
            movespeed = 5 #Slowest move speed possible

        #Ok now we know what motor to activate and how fast it should move.
        movefun(movespeed)

    def unwind_mast(self):
        #Do nothing if we're already at zero
        if self.encoder.curtick == 0:
            return
        print("Unwinding antenna mast back to home position...")

        #Turn off the movement following function for now
        self.stop_movement = True

        #Instead of doing all the slowdown stuff here, we'll just shoot for a positive value and then
        #do a move_to_angle() to finish it off.
        target_angle = 5 # 5 is a safe spot, even if we stop at full speed we wont overrun enough to matter.
        target_tick = round(target_angle / AZ_ANGLE_TICK)

        #determine direction.
        #Positive ticks were clockwise movements, so we need to go counterclockwise to count back.
        if self.encoder.curtick > target_tick:
            movefun = self.move_motor_dir2
            anglesign = 1
        else: #elif self.encoder.curtick < target_tick:
            movefun = self.move_motor_dir1
            anglesign = -1

        movefun()

        tickdiff = anglesign * (self.encoder.curtick - target_tick)
        #Get close enough, within 20 ticks should do it
        while tickdiff > 0:
            if self.encoder.curangle > 360: self.encoder.curangle %= 360
            tickdiff = anglesign * (self.encoder.curtick - target_tick)
            addtext = " - TDIFF: %s" % tickdiff
            if GPIO.input(self.he_sensor) == 0: addtext = " - HE SEN ACTIVE" + addtext
            print(" "*100, end="\r") # Blank out the line before reprinting, otherwise old crap can be left behind
            print("Current angle: %s degrees [%s] %s" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick(), addtext), end="\r")
            sleep(0.01)

        #Now move to home the proper way and we're done
        print("\nUnwound mast, now parking at home position...")
        self.stop_motor()
        self.move_to_angle(AZ_PARK_ANGLE + self.manual_offset_diff)
        #Reset commanded angle to our park angle and resume follower function
        self.commanded_angle = AZ_PARK_ANGLE + self.manual_offset_diff
        self.stop_movement = False

    def stop_motor(self):
        self.AZ_motor_PWM_out.ChangeDutyCycle(0)
        GPIO.output(self.ain1, GPIO.LOW)
        GPIO.output(self.ain2, GPIO.LOW)
        self.is_moving = False

    #Turn clockwise
    def move_motor_dir1(self, speed=100):
        self.AZ_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.ain1, GPIO.LOW)
        GPIO.output(self.ain2, GPIO.HIGH)
        self.is_moving = True

    #Turn counterclockwise
    def move_motor_dir2(self, speed=100):
        self.AZ_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.ain1, GPIO.HIGH)
        GPIO.output(self.ain2, GPIO.LOW)
        self.is_moving = True

    def move_until_stop(self, speed=100):
        angle_every_seconds = 0.1
        curtime = time()
        lasttime = curtime
        self.move_motor_dir2(speed=speed)
        stopstate = GPIO.input(self.he_sensor)
        while stopstate > 0:
            sleep(0.01)
            stopstate = GPIO.input(self.he_sensor)
            curtime = time()
            if curtime - lasttime > angle_every_seconds:
                lasttime = curtime
                print(" "*50, end="\r")
                print("Current azimuth angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
        print("\nHit azimuth endstop, stopping motor!")
        self.stop_motor()

    def move_to_angle(self, angle, speed=100, bypspeedctrl=False):
        #This function simply runs the motor in the correct direction until the curangle of the encoder matches the supplied angle
        #Determine which way to turn the motor
        if angle > self.encoder.curangle:
            movefun = self.move_motor_dir1
            #Could save a few lines of code ignoring the sign information, but we need it to be sure we didnt go past the target.
            anglesign = 1
        else:
            movefun = self.move_motor_dir2
            anglesign = -1

        #For nicer output of current angle
        angle_every_seconds = 0.1
        curtime = time()
        lasttime = curtime

        movefun(speed) #Start moving at speed
        try:
            while anglesign * (angle - self.encoder.curangle) > AZ_ANGLE_TICK: #Keeps stopping 1 tick too late, so using AZ_ANGLE_TICK instead of 0.
                #How a dummy implements the P in PID
                #Slow down when we are close. Say 25% speed when within 5 degrees?
                if anglesign * (angle - self.encoder.curangle) < SLOWDOWN_DEGREES:
                    if not bypspeedctrl: self.AZ_motor_PWM_out.ChangeDutyCycle(30)
                #Slowdown again when we are SLOWDOWN_DEGREES/2 degrees away
                if anglesign * (angle - self.encoder.curangle) < SLOWDOWN_DEGREES/2:
                    if not bypspeedctrl: self.AZ_motor_PWM_out.ChangeDutyCycle(15)
                #Last slowdown for super fine movements
                if anglesign * (angle - self.encoder.curangle) < 1:
                    if not bypspeedctrl: self.AZ_motor_PWM_out.ChangeDutyCycle(8)

                curtime = time()
                if curtime - lasttime > angle_every_seconds:
                    lasttime = curtime
                    #Check endstop but only report when passing, don't stop movement.
                    #Our new antenna design can rotate 1.5 turns from center position before tangling itself, so we will track that differently.
                    #We only use the endstop to home and thats it.
                    if GPIO.input(self.he_sensor) == 0:
                        addtext = " - HE SEN ACTIVE"
                    else:
                        addtext = ""
                    print(" "*100, end="\r") # Blank out the line before reprinting, otherwise old crap can be left behind
                    print("Current angle: %s degrees [%s] %s" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick(), addtext), end="\r")

                sleep(0.01)
        except KeyboardInterrupt: #Stop movement. Other options to stop mid-move required threads and crazy keyboard interrupt stuff.
            print("\nReceived emergency stop command. Ending movement and allowing motors to wind-down...")
            self.stop_motor()
            sleep(1)
        self.stop_motor()
        print(" "*100, end="\r")
        print("Final angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()))

    def find_home(self):
        print("Finding home for the azimuth axis. Be aware this will reset any axis offset you have set.")
        self.homed = False
        #Rough home at high speed
        self.move_until_stop() # Under the hood this always moves motor direction 2, so to go backwards we go dir1
        #Not sure this helps accuracy much but I think for consistency its a good idea
        #After finding home first time we back up a few degrees and find it again at a slower speed.
        print("Found rough home, backing up for slow approach to find precise home.")
        #Reset encoder class's current position to re-zero to give an indication as to how far we were off after we zero
        self.encoder.reset_position()
        self.move_motor_dir1()
        sleep(1)
        print("\nFinding precise home position...")
        self.move_until_stop(speed=5)
        print("\nFound final home position! Resetting counters. Be warned that endstops are not used to prevent over-travel on the azimuth axis.")
        #Reset encoder position for the final time, should be an accurate and repeatable zero.
        self.encoder.reset_position()
        self.commanded_angle = self.encoder.curangle
        self.manual_offset_diff = 0
        self.homed = True

def loadgpscoords():
    global ANTENNA_GPS_LAT, ANTENNA_GPS_LONG, ANTENNA_GPS_ALT

    matchreg = re.compile("ANTENNA_GPS_(LAT|LONG|ALT)\s*=\s*(-?[0-9]{1,3}(?:\.[0-9]{1,6})?)")
    #Group 1 = LAT, LONG, or ALT
    #Group 2 = float

    if os.access(GPSFILEPATH, os.F_OK):
        print("Found gpscoords.txt file, loading...")
        with open(GPSFILEPATH, "r") as coordsfile:
            line = '.'
            while line != "":
                line = coordsfile.readline()
                m = matchreg.match(line)
                if m is not None:
                    match m.group(1):
                        case "LAT": ANTENNA_GPS_LAT = float(m.group(2))
                        case "LONG": ANTENNA_GPS_LONG = float(m.group(2))
                        case "ALT": ANTENNA_GPS_ALT = float(m.group(2))
                    print("Set GPS coords: %s to %s" % (m.group(1), m.group(2)))
    else:
        print("No gpscoords.txt file to load from, using defaults.")


#TODO Turn this into a simple class and have a main loop function that leads to functions for each command
#Its so annoying to find the correct command to edit.
def terminal_interface(azmc, elmc):
    satfind = SatFinder()
    print("Homing each axis separately, starting with the azimuth.")
    #Starting the thread initiates the homing processes and then waits for changes to its commanded angle
    azmc.start()
    #Wait for home to finish
    while azmc.homed is False:
        sleep(0.1)
    elmc.start()
    while elmc.homed is False:
        sleep(0.1)
    print("Finished homing axes, starting user input loop...\n")

    command = ""
    args = ""
    inp = ""
    while inp.lower() != "quit" and inp.lower() != "q":
        try:
            #Allow internal loop bypass
            if inp == "BYPASS": #Bypassing user input and directly activating another command.
                inp = ""
            else:
                inp = input("CMD: ").strip().lower()
                inpsplit = re.split(" ", inp)
                command = inpsplit[0]
                args = " ".join(inpsplit[1:])

            if command == "p" or command == "pos" or command == "position":
                #Determine what axis we asked about
                if args == "az":
                    print("Current mast position is: %s degrees [%s]\n" % (azmc.encoder.getCurrentAngle(), azmc.encoder.getCurrentTick()))
                elif args == "el":
                    print("Current antenna elevation is: %s degrees [%s]\n" % (elmc.encoder.getCurrentAngle(), elmc.encoder.getCurrentTick()))
                elif args == "":
                    print("Current position of azimuth and elevation is: %s [%s]  -  %s [%s]" % (azmc.encoder.getCurrentAngle(), azmc.encoder.getCurrentTick(), elmc.encoder.getCurrentAngle(), elmc.encoder.getCurrentTick()))
                else:
                    print("Invalid argument, either 'az' or 'el' should be passed to the position command or nothing at all.")

            elif command == "u":
                azmc.unwind_mast()

            elif command == "h" or command == "home":
                if args == "az":
                    print("Finding mast home position, current position is: %s degrees [%s]\n" % (azmc.encoder.getCurrentAngle(), azmc.encoder.getCurrentTick()))
                    azmc.find_home()
                elif args == "el":
                    print("Finding antenna elevation home position, current position is: %s degrees [%s]\n" % (elmc.encoder.getCurrentAngle(), elmc.encoder.getCurrentTick()))
                    elmc.calhome()
                else:
                    print("Invalid argument, either 'az' or 'el' should be passed to this command.")

            #I just realized this is a dumb command name. It doesn't pause anything, it permanently stops movement. There is no continue.
            #Whereas the estop is actually a pause which can be continued at will. But epause just sounds dumb.
            #Pause movement, soft-stop
            #Currently we just set the commanded angle to whatever the angle currently is
            elif command == "estop":
                if args == "az":
                    print("Pausing movement on the azimuth axis at %s degrees" % azmc.encoder.curangle)
                    azmc.commanded_angle = azmc.encoder.curangle
                elif args == "el":
                    print("Pausing movement on the elevation axis at %s degrees" % elmc.encoder.curangle)
                    elmc.commanded_angle = elmc.encoder.curangle
                elif args == "": #Pause both
                    print("Pausing azimuth and elevation axis at AZ%s EL%s" % (azmc.encoder.curangle, elmc.encoder.curangle))
                    azmc.commanded_angle = azmc.encoder.curangle
                    elmc.commanded_angle = elmc.encoder.curangle
                else:
                    print("Invalid argument, either 'az' or 'el' should be passed to this command.")

            #More of a hard-stop, just interrupt the command function loop
            elif command == "pause":
                print("Commanded both axes to stop movement now...")
                azmc.stop_movement = True
                elmc.stop_movement = True

            #ONLY WAY TO RESUME FROM AN ESTOP BESIDES RESTARTING IS THIS COMMAND!
            #NO OTHER COMMANDS WILL WORK AFTER AN ESTOP UNTIL YOU DO THIS
            elif command == "resume":
                print("Restarting movement for both axes...")
                azmc.stop_movement = False
                elmc.stop_movement = False


            #Test go command, allows for testing different PWM constants live
            #Was going to use the normal go command and add this into it but its different enough to needs its own thing
            #It will only support one axis at a time
            #Move command format:
            #tgo az101.64 50
            #That will move the azimuth to angle 101.64 and use the PWM constant 50 as its move speed
            #Should allow me to dial in the exact constants needed to move the antenna at different angles
            elif command == "tgo":
                test_command = re.search("(az|el) ?([0-9]{1,3}(?:\.[0-9]{1,3})?) ([0-9]{1,2})", args)
                if test_command is None:
                    print("Incorrect command structure. Command format is tgo <az|el> <angle> <PWM_val>")
                    continue
                axis, angle, pwmval = test_command.groups()
                #Can't compare strings to ints so correct the types
                try:
                    angle = float(angle)
                    pwmval = int(pwmval)
                except:
                    print("Error processing arguments to test go command. Probably incorrect arg. Command format is tgo <az|el> <angle> <PWM_val>")
                    continue

                #Now that we know what axis and angle, we can make sure those values are sane
                if axis == "az":
                    if abs(angle) > MAX_AZ_INPUT_ANGLE:
                        print("Given azimuth angle is outside the allowable range (MAX +-%s). Not moving." % MAX_AZ_INPUT_ANGLE)
                        continue
                elif axis == "el":
                    if not (0 <= angle <= MAX_EL_INPUT_ANGLE): # Dont allow movements below 0
                        print("Given elevation angle is outside the allowble range (0-%s). Not moving." % MAX_EL_INPUT_ANGLE)
                        continue
                if not (0 <= pwmval <= 100):
                    print("Given PWM value is outside the allowble range (0-100). Not moving.")
                    continue

                #At this point we should have a set of valid, sane values that we can use.
                #I was going to check if we were wound up but eh, just be careful.
                #This isn't a command I will use much, probably disable it after I know the correct vals.
                #So I dont plan on doing anything dumb like moving over 100 degrees while the mast was wound up.
                #Thats all to say, dont be dumb and it should be fine.

                #To have full control over movement speed we have to disable the automatic angle tracking loop
                #The interfaces are the same so we can cut some code out by using one access variable
                if axis == "az":
                    axis_obj = azmc
                elif axis == "el":
                    axis_obj = elmc

                axis_obj.stop_movement = True
                #Now with the follower loop disabled we just wait for move_to_angle() to return
                axis_obj.move_to_angle(angle, pwmval, bypspeedctrl=True)
                #Before we reenable we have to sync the commanded_angle and the current angle, otherwise
                #it will start moving back to its old position when we enable movement again.
                axis_obj.commanded_angle = axis_obj.encoder.curangle
                axis_obj.stop_movement = False


            #Move command format:
            #go az101.64 el95
            #Can commit az or el and move just one axis
            #should be tolerant of spaces between az and the numbers
            #should be tolerant of both ints and floats, possibly mixed
            #should be tolerant of leading zeros.
            #should be tolerant of transposing commands (el before az)
            elif command == "go" or command == "sgo":
                azangle = None
                elangle = None
                azcommand = re.search("az ?([0-9]{1,3}(?:\.[0-9]{1,3})?)", args)
                if azcommand is not None:
                    azangle = float(azcommand.group(1))
                    if abs(azangle) <= MAX_AZ_INPUT_ANGLE:
                        aztick = round(azangle / azmc.encoder.ANGLE_TICK)
                        aztext = "azimuth axis to %s degrees [%s]" % (azangle, aztick)
                    else:
                        print("Invalid azimuth angle of %s degrees (Current limit is %s), not commanding any movements" % (azangle, MAX_AZ_INPUT_ANGLE))
                        continue
                else:
                    aztext = ""
                elcommand = re.search("el ?([0-9]{1,3}(?:\.[0-9]{1,3})?)", args)
                if elcommand is not None:
                    elangle = float(elcommand.group(1))
                    if 0 <= elangle <= MAX_EL_INPUT_ANGLE: # Dont allow movements below 0
                        eltick = round(elangle / elmc.encoder.ANGLE_TICK)
                        eltext = "elevation axis to %s degrees [%s]" % (elangle, eltick)
                    else:
                        print("Invalid elevation angle of %s degrees (Current limit is %s), not commanding any movements" % (elangle, MAX_EL_INPUT_ANGLE))
                        continue
                    if len(aztext) > 0:
                        eltext = " and " + eltext
                else:
                    eltext = ""

                #Dont do anything if we dont have anything
                if elcommand is None and azcommand is None:
                    continue

                if command == "go":
                    movespeed = 100
                    cmdtext = "Moving the "
                else: #slow go command
                    movespeed = 25
                    cmdtext = "Slowly moving the "

                curpostext = " (current pos: AZ%s - EL%s)" % (azmc.encoder.getCurrentAngle(), elmc.encoder.getCurrentAngle())
                outputtext = cmdtext + aztext + eltext + curpostext
                print(outputtext)
                #Our new movement system is closed loop without any feedback. We now have a separate command to watch
                #the movement progress called 'watch'.
                #After setting the movement position we will directly go to watch mode.
                if azangle is not None:
                    azmc.speed = movespeed
                    azmc.commanded_angle = azangle
                if elangle is not None:
                    elmc.speed = movespeed
                    elmc.commanded_angle = elangle
                #Go straight to watch mode
                inp = "BYPASS"
                command = "watch"

            elif command == "w" or command == "watch":
                try:
                    timercount = 0
                    pausedelay = 1 #How long in seconds do we wait after is_moving is false on both axis to stop tracking movement?
                    while timercount < pausedelay/0.1:
                    #while azmc.is_moving is True or elmc.is_moving is True:
                        print(" "*100, end="\r") #Blanking line 100 chars long
                        print("Current antenna position: AZ %s degrees [%s] --\tEL %s degrees [%s]" % (round(azmc.encoder.curangle, 3), azmc.encoder.curtick, round(elmc.encoder.curangle, 3), elmc.encoder.curtick), end="\r")
                        #If we're at position we delay a little so the user can see movement has stopped
                        if azmc.is_moving is False and elmc.is_moving is False:
                            timercount += 1
                        else:
                            #Reset counter if we start moving again on any axis.
                            timercount = 0
                        sleep(0.1)
                    print("\n\nAt target position: AZ %s degrees [%s]--\tEL %s degrees [%s]\n" % (round(azmc.encoder.curangle, 3), azmc.encoder.curtick, round(elmc.encoder.curangle, 3), elmc.encoder.curtick))
                except KeyboardInterrupt:
                    print("\n")
                    continue

            elif command == "updatetle":
                satfind.updatetle()

            elif command == "gpscoords":
                print("Current GPS Coords: %s, %s (Alt: %s)" % (ANTENNA_GPS_LAT, ANTENNA_GPS_LONG, ANTENNA_GPS_ALT))

            #Here we can manually change what our azimuth or elevation currently is
            #Say you point at GOES but find the strongest signal is actually .2 degrees to the left.
            #You point the antenna at the strongest spot and then use this command to tell the program its current
            #azimuth is the exact azimuth of GOES. Same as manually calibrating azimuth but in software. We could even
            #save the offset to file and load it again.
            elif command == "setazoffset":
                if args.count(".") > 1 or args.replace(".", "").isnumeric() is False:
                    print("Invalid argument, must be an int or float. E.g. setazoffset 211.8")
                    continue
                angle = float(args)
                if abs(angle - azmc.encoder.curangle) > 5:
                    print("For safety, azimuth adjustments of more than 5 degrees are not allowed. If your mast is off by that much you need to manually realign it. This function is for fine adjustment only.")
                    continue

                if abs(angle) > MAX_AZ_INPUT_ANGLE:
                    print("Input angle is outside of allowable range for the azimuth axis. Entered value: %s (MAX %s)" % (str(angle), str(MAX_AZ_INPUT_ANGLE)))
                    continue
                #Should be valid angle adjustment now (hopefully)
                yn = '.'
                while yn not in "yn":
                    #Not changing the tick number here since we need that to unwind without crashing.
                    #Tick is only used to unwind and angle is only reset on home, so just an angle offset should work for everything.
                    offset_diff = angle - azmc.encoder.curangle
                    yn = input("Changing azimuth angle from %s to %s (Diff: %s), are you sure you want to do this? (Y/N): " % (round(azmc.encoder.curangle, 3), angle, offset_diff)).lower()
                    if yn == "y":
                        print("Ok updating azimuth angle to %s degrees..." % angle)
                        azmc.manual_offset_diff += offset_diff # Should track multiple different offsets properly
                        azmc.encoder.curangle = angle
                        azmc.commanded_angle = angle
                    else:
                        print("Canceling offset update...")
                        continue

            elif command == "d":
                print("Current commanded angles (AZ, EL): (%s, %s)" % (azmc.commanded_angle, elmc.commanded_angle))
                print("Motor movement status (AZ, EL): (%s, %s)" % (str(azmc.is_moving), str(elmc.is_moving)))
                print("E-Stop Status: AZ:%s  - EL:%s" % (str(azmc.stop_movement), str(elmc.stop_movement)))
                print("Manual azimuth offset: %s degrees" % azmc.manual_offset_diff)

            elif command == "v" or command == "version":
                print("Satellite Antenna Control version %s" % _VERSION_)

            elif command == "satlist":
                satfind.satlist()

            #Support input of timeframe directly after command and before sat name
            # e.g. passlist 48 meteor-m2 2
            # would give passes for the next 48 hours for meteor-m2 2
            # No number there would default to 24h  
            elif command == "passlist": # Generate a 24 hour passlist for a specific satellite
                #Check for time limit. None of the satellites start with a number so this should be reliable.
                argsplit = re.split(" ", args)
                if argsplit[0].isdigit():
                    tlimit = int(argsplit[0])
                    args = " ".join(argsplit[1:])
                else:
                    tlimit = 24 # Default limit of 24 hours
                if len(args) == 0:
                    print("You must include a satellite name to list passes for!")
                    continue
                passes = satfind.passlist(args, tlimit)
                if passes is not None:
                    satparams = satfind.getsatparams(args)
                    satfind.printpasses(satparams, passes)

            #Starting to feel like this should be separated. So much code here and theres more coming.
            elif command == "track":
                try: #Covering this one specifically in try/except so we can always unwind if anything goes wrong
                    satname = args #so the code looks nicer
                    if len(satname) == 0:
                        print("You must include a satellite name to track!")
                        continue
                    satparams = satfind.getsatparams(satname)
                    passes = satfind.passlist(satname, 24) # A full day is a long time to wait parked for a track
                    if passes is None:
                        continue
                    satfind.printpasses(satparams, passes)
                    print("%s) Cancel Track" % len(passes))
                    selectedpass = input("Select a pass number to track: ")
                    if selectedpass.isdigit() is False:
                        print("Selection must be a digit")
                        continue
                    if int(selectedpass) > len(passes)-1:
                        print("Canceling track selection.")
                        continue
                    passdata = passes[int(selectedpass)]
                    passduration = (passdata[1] - passdata[0]).total_seconds()
                    starttimelocaltz = passdata[0].astimezone()
                    nicestarttime = starttimelocaltz.strftime("%Y-%m-%d %H:%M:%S")
                    starttime = starttimelocaltz - datetime.now().astimezone()
                    startimetext = create_time_string(starttime.total_seconds())
                    max_location = satparams.get_observer_look(passdata[2], ANTENNA_GPS_LONG, ANTENNA_GPS_LAT, ANTENNA_GPS_ALT)
                    eastwest = "E" if max_location[0] < abs(ANTENNA_GPS_LONG) else "W"
                    print("Selected pass #%s, %s%s MEL, starting in %s." % (selectedpass, round(max_location[1]), eastwest, startimetext))
                    #Find the start location for this pass
                    (startaz, startel) = satparams.get_observer_look(passdata[0], ANTENNA_GPS_LONG, ANTENNA_GPS_LAT, ANTENNA_GPS_ALT)
                    startmove = input("Starting position for this pass is at Az%s El%s. Move to start position and wait to track (Y/N)? " % (round(startaz, 3), round(startel, 3))).lower()
                    if startmove == "n":
                        print("Canceling tracking...")
                        continue
                    elif startmove == "y":
                        #print("Unwinding mast to home position before moving to start position...") # unwind_mast() prints basically the same thing
                        azmc.unwind_mast()
                        print("Moving to starting position...")
                        azmc.commanded_angle = startaz
                        elmc.commanded_angle = startel
                        #Track movement until we're about 1 degree away then go into wait mode
                        while abs(azmc.commanded_angle - azmc.encoder.curangle) > 1 or abs(elmc.commanded_angle - elmc.encoder.curangle) > 1:
                            print(" "*100, end="\r") #Blanking line 100 chars long
                            print("Current antenna position: AZ %s degrees [%s] --\tEL %s degrees [%s]" % (round(azmc.encoder.curangle, 3), azmc.encoder.curtick, round(elmc.encoder.curangle, 3), elmc.encoder.curtick), end="\r")
                            sleep(0.1)
                        print("")
                        startimetext = create_time_string((starttimelocaltz - datetime.now().astimezone()).total_seconds())
                        print("Parked at starting position, now waiting for the pass to start. Start time is %s (%s)." % (nicestarttime, startimetext))
                        seconds_until_start = 2
                        while seconds_until_start > 1:
                            seconds_until_start = (starttimelocaltz - datetime.now().astimezone()).total_seconds()
                            timetostarttext = create_time_string(seconds_until_start)
                            print(" "*100, end="\r")
                            print("Waiting for %s, starting at %s (ETA %s)..." % (satname, nicestarttime, timetostarttext), end="\r")
                            sleep(1)
                        print("")
                        print("Starting track on %s!" % satname)
                        #TODO Start satdump command line for automatically reception of images! Think I have it figured out, should be easy.
                        #Example command lines for NOAA 18 and Meteor-M2 HRPT
                        # satdump live noaa_hrpt OUTPUT_FOLDER --source rtlsdr --samplerate 2.4e6 --frequency 1707.0e6 --gain 49 --http_server 0.0.0.0:8080
                        # satdump live meteor_hrpt OUTPUT_FOLDER --source rtlsdr --samplerate 2.4e6 --frequency 1700.0e6 --gain 49 --http_server 0.0.0.0:8080
                        #The http server allows realtime feedback of progress and signal strength, we could incorporate that data into our output.
                        stoptimelocaltz = passdata[1].astimezone()
                        seconds_until_stop = 2
                        while seconds_until_stop > 1:
                            seconds_until_stop = (stoptimelocaltz - datetime.now().astimezone()).total_seconds()
                            #Figure out sat position right now
                            (sataz, satel) = satparams.get_observer_look(datetime.now(pytz.utc), ANTENNA_GPS_LONG, ANTENNA_GPS_LAT, ANTENNA_GPS_ALT)
                            #Command movements, then update screen
                            azmc.commanded_angle = sataz
                            elmc.commanded_angle = satel
                            timeleftstr = create_time_string(seconds_until_stop)
                            invtimeleft = passduration - seconds_until_stop
                            timeleftpct = round((invtimeleft/passduration)*100)
                            #Clearing more than the current width of the screen causes a \n to be printed
                            #But we dont want to underclear either. Just take the length of the current message and add 10.
                            #It probably hasnt changed THAT much from one second ago.
                            printmsg = "Tracking %s at Az%s El%s - %s%% %s -\tCurrent antenna position: Az%s El%s" % (satname, round(sataz, 3), round(satel, 3), timeleftpct, timeleftstr, round(azmc.encoder.curangle, 3), round(elmc.encoder.curangle, 3))
                            print(" "*(len(printmsg)+10), end="\r")
                            print(printmsg, end="\r")
                            sleep(0.05) #Update 50 times a second. Might need more.
                        print("")
                        print("Finished tracking %s! Parking antenna at home position..." % satname)
                        #azmc.unwind_mast() call is how we park home again in the try/except's finally statement below
                except KeyboardInterrupt:
                    print("Canceling track command. Unwinding mast to home position.")
                    continue
                finally:
                    azmc.unwind_mast()
                    #AAAAnd now park
                    azmc.commanded_angle = AZ_PARK_ANGLE + azmc.manual_offset_diff #Account for any offset we have
                    elmc.commanded_angle = EL_PARK_ANGLE
                    inp = "BYPASS"
                    command = "watch"

            #End of while loop
            sleep(0.5)
        except KeyboardInterrupt:
            print("\n")
            continue
        except Exception as e:
            azmc.stop_movement = True
            elmc.stop_movement = True
            print("An unexpected error occurred, stopping all motors for safety. You must type 'resume' to allow movement again. Double check your commanded angles first with the 'd' command! The error: ")
            print(e)
            continue

class SatFinder:
    def __init__(self):
        self.satnames = []
        #Updatetle only calls updatesatnames() when it grabs a new TLE file. No reason to always update the satnamelist
        #even if we don't download a new TLE file. It could possibly double call if we have to download a new TLE but
        #im ok with that, it only happens once at start anyway.
        self.updatesatnames()
        self.updatetle()

    def satlist(self):
        print("Satellites in TLE file:")
        for sat in self.satnames:
            print(sat)

    #Takes in a passlist, filters out passes with max elevations under the elevation_limit, and returns the list
    def filterpasses(self, satparams, passlist, elevation_limit):
        filteredpasses = []
        for passdata in passlist:
            _, maxelevation = satparams.get_observer_look(passdata[2], ANTENNA_GPS_LONG, ANTENNA_GPS_LAT, ANTENNA_GPS_ALT)
            if float(maxelevation) < elevation_limit:
                continue
            filteredpasses.append(passdata)
        return filteredpasses

    def printpasses(self, satparams, passlist):
        #Go over the list and pull out some details, then print each one
        for i, passdata in enumerate(passlist):
            #passdata[0] = AOS time
            #passdata[1] = LOS time
            #passdata[2] = max elevation time
            localtz = passdata[0].astimezone()
            nicestarttime = localtz.strftime("%Y-%m-%d %H:%M:%S")
            starttime = localtz - datetime.now().astimezone()
            durationtext = create_time_string((passdata[1] - passdata[0]).total_seconds())
            startimetext = create_time_string(starttime.total_seconds())
            max_location = satparams.get_observer_look(passdata[2], ANTENNA_GPS_LONG, ANTENNA_GPS_LAT, ANTENNA_GPS_ALT)
            eastwest = "E" if max_location[0] < abs(ANTENNA_GPS_LONG) else "W"
            longitude = round(satparams.get_lonlatalt(passdata[2])[0]) #Longitude at max elevation
            longtext = "%s%s" % (abs(longitude), "E" if longitude > 0 else "W")
            #Find the direction. Only way I could think of is to look at the change in azimuth angle and if its going down or up
            #One minute ahead in time should be enough to tell for sure what direction we are going, using max elevation as reference
            p2time = passdata[2] + timedelta(minutes=1)
            p2location = satparams.get_observer_look(p2time, ANTENNA_GPS_LONG, ANTENNA_GPS_LAT, ANTENNA_GPS_ALT)
            #More complicated than I thought, I also have to know what side of the circle I'm on
            ns = ["North", "South"]
            if max_location[0] > p2location[0]: #Compare their azimuths
                di = 1
            else:
                di = 0
            #And if we're on the other side of the circle its the opposite
            if max_location[0] < 180:
                di ^= 1
            direction = ns[di]
            print("%s) %s - %s%s degree MEL pass (%s Long) heading %s in %s - duration %s" % (i, nicestarttime, round(max_location[1]), eastwest, longtext, direction, startimetext, durationtext))

    def passlist(self, satname, time_limit):
        satparams = self.getsatparams(satname)
        if satparams is None:
            return None
        #Horizon limit will affect the start and finish times of the pass and the displayed total duration
        #Do we want to know how long the pass will take with our tracking limits or how long the pass is from horizon to horizon?
        #For now we're counting from the tracking limits.
        passlist = satparams.get_next_passes(datetime.now(pytz.utc), time_limit, ANTENNA_GPS_LONG, ANTENNA_GPS_LAT, ANTENNA_GPS_ALT, horizon=TRACKING_START_ELEVATION) #Next 24 hours
        #Filter out passes with max elevations below our filter limit
        passlist = self.filterpasses(satparams, passlist, PASSLIST_FILTER_ELEVATION)
        if len(passlist) > 0:
            print("Found %s passes in the next %s hours for '%s'." % (len(passlist), time_limit, satname))
        else:
            print("No passes above %sMEL for %s in the next %s hours using current TLE data." % (PASSLIST_FILTER_ELEVATION, satname, time_limit))
            return None
        return passlist

    def getsatparams(self, satname):
        #Check if the satellite exists
        try:
            satparams = orbital.Orbital(satname, tle_file=TLEFILEPATH)
        except KeyError:
            closenamecheck = self.findclosestsatname(satname)
            if isinstance(closenamecheck, list):
                print("Couldn't find satellite '%s' in TLE file, did you mean '%s'? Use the 'satlist' command to see a list of available satellites." % (satname, closenamecheck[1]))
                return None
            else:
                print("Returning results for '%s' as '%s' wasn't found in the satellite list." % (closenamecheck, satname))
                satparams = orbital.Orbital(closenamecheck, tle_file=TLEFILEPATH)
        except NotImplementedError:
            print("Pyorbital doesn't yet support calculations for geostationary satellites. There are alternative libraries that I have yet to try that may support them.")
            return None
        return satparams

    def findclosestsatname(self, nxsatname):
        #Use the sequence matcher against the satnames list and return if the match is 95% or greater (needs tuning).
        highestmatch = [None, 0]
        for s in self.satnames:
            possiblematch = SequenceMatcher(None, nxsatname.lower(), s.lower()).ratio()
            #keep track of the best match so far
            if possiblematch > highestmatch[1]: highestmatch = [s, possiblematch]
            if possiblematch > SATNAME_MATCH_RATIO:
                return s
        #If we didn't get a good solid match then just return the closest thing along with None so we know its not a good match
        return [None, highestmatch[0]]

    def updatesatnames(self):
        if os.access(TLEFILEPATH, os.F_OK) is False:
            print("No TLE file found for updatestanames().")
            return
        print("Updating list of satellites names...")
        self.satnames = []
        with open(TLEFILEPATH, "r") as tlefile:
            line = tlefile.readline()
            while line != '':
                if line[0].isdigit() is False:
                    self.satnames.append(line.strip())
                line = tlefile.readline()
        self.satnames.sort()


    def updatetle(self):
        print("Starting TLE update...")
        #Check if we have an updated TLE file for weather satellites, if not grab a fresh one.
        #It should be in the same dir this file
        if os.access(TLEFILEPATH, os.F_OK) is True:
            #Check age
            curtime = time()
            filemodtime = int(os.stat(TLEFILEPATH).st_mtime)
            if curtime - filemodtime > TLE_UPDATE_FREQ_DAYS * 24 * 60 * 60: # 3 days
                print("Updating weather.txt TLE file...")
                urlretrieve("http://celestrak.org/NORAD/elements/weather.txt", TLEFILEPATH)
                self.updatesatnames()
        else:
            print("Downloading weather.txt TLE file...")
            urlretrieve("http://celestrak.org/NORAD/elements/weather.txt", TLEFILEPATH)
            self.updatesatnames()

def create_time_string(seconds_total):
    days = int(seconds_total/86400)
    hours = int((seconds_total%86400)/3600)
    minutes = int((seconds_total%86400%3600)/60)
    seconds = int(seconds_total%86400%3600%60)
    timestring = ""
    if days > 0:
        timestring += "%s day " % days
        if days > 1: timestring = timestring[:-1] + "s" + timestring[-1:]
    if hours > 0:
        timestring += "%s hour " % hours
        if hours > 1: timestring = timestring[:-1] + "s" + timestring[-1:]
    if minutes > 0:
        timestring += "%s minute " % minutes
        if minutes > 1: timestring = timestring[:-1] + "s" + timestring[-1:]
    if seconds > 0:
        timestring += "%s second" % seconds
        if seconds > 1: timestring += "s"
    timestring = timestring.strip()
    return timestring

def check_for_recovery_data():
    posdata = None
    recovery_data = None
    if os.access(POSITION_RECOVERY_FILE_PATH, os.F_OK):
        print("Found active position data file, checking...")
        with open(POSITION_RECOVERY_FILE_PATH) as saved_position_file:
            # Example line:
            # { "AzimuthDeg": 211.813, "AzimuthTick": -955, "ElevationDeg": 45.58, "ElevationTick": 253, "Is_Moving": false }
            try:
                posdata = json_load(saved_position_file)
            except JSONDecodeError:
                print("Couldn't recover position data from file.")
                print("If you think the azimuth axis is currently wound up you may want to cancel running this program and manually unwind it to prevent any damage.")
    else:
        print("No recovery file found. Skipping...")
        return None
    # At this point the variable posdata is either None or has something in it
    # First we need to verify that ALL the data is present that we need.
    if posdata is not None:
        needed_keys = ["AzimuthDeg", "AzimuthTick", "ElevationDeg", "ElevationTick", "Is_Moving"]
        actual_keys = list(posdata.keys())
        if all(k in actual_keys for k in needed_keys):
            #Keys are valid, now lets check if the values are valid, they should all be floats or ints and the last a bool
            ismoving = posdata.pop("Is_Moving")
            if all([isinstance(t, (int, float)) for t in posdata.values()]):
                #Add the ismoving back on
                posdata["Is_Moving"] = ismoving
                #TODO Should probably check values ranges here too but im lazy and just want this working now
                #all the recovery data is hand-written at first anyway since theres no saving functions.
                print("Recovery data passed basic checks. Here's what we got:\n")
                for k,v in posdata.items(): print("%s:\t%s" % (k,v))
                print("\n")
                goodyn = '.'
                while goodyn not in "yn":
                    goodyn = input("Does the above data look sane? (Y/N): ").lower()
                if goodyn == "y":
                    print("Ok! Using the above recovery data instead of homing the antenna.")
                    recovery_data = posdata
                else:
                    print("Not using recovery data. Antenna will go through the homing process normally.")
            else:
                print("Recovery data failed type checks, not using it.")
        else:
            print("Recovery data was incomplete so it won't be used. Here's what data we did have: ")
            print(posdata.keys())

    return recovery_data

def main():
    if os.access(RUNFOLDER, os.F_OK) is False:
        raise(Exception("Couldn't access the RUNFOLDER directory: '%s' - Quitting..." % RUNFOLDER))
    print("Welcome to Satellite Antenna Control v%s!" % _VERSION_)
    sleep(2)
    print("Setting up board and homing axes...")
    sleep(1)

    #Load GPS coords stored in a file if it exists
    loadgpscoords()

    # Check if we have position data to recover to, returns None if there wasn't any to load.
    recovery_data = check_for_recovery_data()

    # Pass the recovery data (if any) to the motor control classes and let them handle the rest
    azmc = AzMotorControl(recovery_data=recovery_data)
    elmc = ElMotorControl(recovery_data=recovery_data)

    try:
        #Start the terminal interface, which will also handle homing and starting the threads
        terminal_interface(azmc, elmc)
    except:
        import traceback
        traceback.print_exc()
    finally:
        print("Satellite Antenna control program exit, cleaning up threads and GPIO...")
        #TODO if we want to be able to quit without moving anything so we can restart with recovery data, these will mess things up.
        #Will have to check if we're doing a recovery restart here and tell the threads to NOT home/park on quit.
        azmc.quitthread()
        elmc.quitthread()
        azmc.join()
        elmc.join()
        GPIO.cleanup()


if __name__ == "__main__":
   main()
