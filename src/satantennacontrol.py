_VERSION_ = 0.1

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
from time import sleep, time
import threading

PWM_FREQUENCY = 2000

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
MAX_AZ_INPUT_ANGLE = 370
MAX_EL_INPUT_ANGLE = 180 #Elevation axis can actually go slightly below 0 or 180 but its not needed
PWM_FREQUENCY = 2000
SLOWDOWN_DEGREES = 3 #Slow down when within this many degrees of target
SLOWDOWN_PCT = 0.25 #Percent of normal speed to slow to

#Roughly how many degrees rotation for each tick. Some maths behind this, don't mess with it if you don't know what you're doing.
# Ring gear is 58 tooth and encoder gear is 10 tooth, so 5.8:1.
# And our encoder *should* emit 100 ticks per revolution, but the code we have working emits 400 ticks per revolution.
# Doesn't matter for our purposes, just have to remember its 400.
#
# So now the maths:
# Our encoder should emit a pulse 400 times per full revolution, so how many degrees per each tick?
# 360/400 = 0.9 degrees per tick
#
# With a gear ratio of 5.8 between our encoder and ring gear, we should expect 5.8*400 encoder ticks per mast revolution.
# Total ticks per mast revolution = 5.8*400 = 2,320
# Which means degrees per tick is: 360/2320 ~= 0.15517
# If we ever want to calculate the current angle from the tick number its:
# current_tick*ANGLE_TICK
AZ_ANGLE_TICK = 0.15517
#Elevation axis uses a 12 tooth and 60 tooth gearset so the math changes slightly:
#Instead of 5.8, our ratio is 60/12 = 5
# That means ticks per full revolution is 5*400 = 2000
#So degrees per tick should be 360/2000 = 0.18
EL_ANGLE_TICK = 0.18
#For 0.1 degree accuracy on the Elevation axis we need what tooth count on the encoder wheel?
# We would need 360/0.1 = 3600 ticks per full revolution
# For a 400 tick per revolution encoder we need a gear ratio of 3600/400 = 9:1
# With a 60-tooth main gear, the encoder gear needs to be 60/9 = 6.66666 teeth
#Probably need a finer precision rotary encoder or larger main gear.
#Where do we park the elevation and azimuth axes after calibrating?
EL_PARK_ANGLE = 90 # Straight up and down
AZ_PARK_ANGLE = 0 # Park azimuth at 0 (which should be due north)

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
                self.curangle -= self.ANGLE_TICK
                self.curtick -= 1
                #print("Clockwise: %s" % self.curangle)
            else:
                self.direction = self.CLOCKWISE
        elif delta == 3:
            if self.direction == self.ANTICLOCKWISE:
                self.curangle += self.ANGLE_TICK
                self.curtick += 1
                #print("Anticlockwise: %s" % self.curangle)
            else:
                self.direction = self.ANTICLOCKWISE
    def getCurrentTick(self): return self.curtick
    def getCurrentAngle(self): return round(self.curangle, 3) #Round curangle to 3 digits to prevent weird scientific notation output for small values near 0

class ElMotorControl(threading.Thread):
    def __init__(self, BIN1=EL_BIN1, BIN2=EL_BIN2, PWMB=EL_PWMB, YELLOW_ENDSTOP=EL_ENDSTOP_YELLOW, BLUE_ENDSTOP=EL_ENDSTOP_BLUE):
        super(ElMotorControl, self).__init__()
        self.encoder = RotaryEncoder(EL_POS_OUTA, EL_POS_OUTB, EL_ANGLE_TICK)
        self.bin1 = BIN1
        self.bin2 = BIN2
        self.pwmb = PWMB
        self.yellow_endstop = YELLOW_ENDSTOP
        self.blue_endstop = BLUE_ENDSTOP
        #Setup pins and PWM channel
        GPIO.setup(self.bin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.bin2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pwmb, GPIO.OUT, initial=GPIO.LOW)
        self.EL_motor_PWM_out = GPIO.PWM(self.pwmb, PWM_FREQUENCY)       #create PWM instance with frequency of PWM_FREQUENCY hz
        self.EL_motor_PWM_out.start(0) #Start PWM0 output at 0 duty cycle
        #Elevation axis is using a pair of physical endstop switches that are pulled high and go low when activated
        GPIO.setup(self.yellow_endstop, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.blue_endstop, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.angle_fuzz = 89.5 #What angle to set to after we go to what we "think" is 90 degrees.
        # New method of moving, instead of being prompted a single angle and moving towards it we will constantly evaluate
        # what the commanded angle is vs our current angle. Speed and direction to be calculated every loop iteration.
        self.commanded_angle = EL_PARK_ANGLE #Starting at park since we go there after calibrating.
        self.quitting = False

    #Start of threaded operation. We'll home the axis and then begin the move_to_commanded_angle() loop
    def run(self):
        self.calhome()
        print("\n\n")
        #We should be parked at 90 degrees at this point. Now we start our loop.
        #How many operations per second should the loop run at? Default value is 100 times per second. Could probably get away with less.
        ops = 100
        while self.quitting is False:
            self.move_to_commanded_angle()
            #print(" "*100, end="\r")
            #print("Current angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
            sleep(1.0/ops)

        print("ElMotorControl thread received quit signal. Exiting thread...")
        self.quitthread()

    def quitthread(self):
        #If we were called externally then set the internal quitting flag true so the loop can stop
        self.quitting = True
        #Stop the PWM channel or it causes problems when quitting python
        #Final app exit will handle the GPIO.cleanup()
        self.EL_motor_PWM_out.stop()

    def move_to_commanded_angle(self):
        # This function is constantly called as a background thread, constantly evaluating and commanding movements.
        #Determine which way to turn the motor
        if self.commanded_angle > self.encoder.curangle:
            movefun = self.move_motor_dir1
            endstop = self.blue_endstop
            #Needed sign information for old function but new function probably could get away with abs() instead. This works though so meh.
            anglesign = 1
        elif self.commanded_angle < self.encoder.curangle:
            movefun = self.move_motor_dir2
            endstop = self.yellow_endstop
            anglesign = -1
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
        if anglediff < self.encoder.ANGLE_TICK:
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
        movespeed = 100 # PWM duty cycle is between 0 and 100
        if anglediff > 5:
            pass # no slowdown, 100 duty cycle
        elif anglediff > 2: # Between 5 and 2 degrees, first slowdown
            movespeed *= SLOWDOWN_PCT
        elif anglediff > 1: #Between 1 and 2 degrees, second slowdown
            movespeed *= SLOWDOWN_PCT**2
        elif anglediff >= self.encoder.ANGLE_TICK*3: #between a degree difference and 3 ANGLE_TICKs
            movespeed *= SLOWDOWN_PCT**3 # Duty cycle of 1.5625%
        else: # Between 3 ticks and our target we really crawl slowly
            movespeed = 0.1 #Slowest move speed possible

        #Ok now we know what motor to activate and how fast it should move.
        movefun(movespeed)

    def stop_motor(self):
        self.EL_motor_PWM_out.ChangeDutyCycle(0)
        GPIO.output(self.bin1, GPIO.LOW)
        GPIO.output(self.bin2, GPIO.LOW)

    #This moves the elevation axis towards the blue endstop
    def move_motor_dir1(self, speed=100):
        self.EL_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.bin1, GPIO.HIGH)
        GPIO.output(self.bin2, GPIO.LOW)

    #This moves the elevation axis towards the yellow endstop
    def move_motor_dir2(self, speed=100):
        self.EL_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.bin1, GPIO.LOW)
        GPIO.output(self.bin2, GPIO.HIGH)

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
        angle_every_seconds = 0.1 #Output current angle every 2 seconds
        curtime = time()
        lasttime = curtime
        while stopstate > 0:
            sleep(0.1)
            stopstate = GPIO.input(es)
            curtime = time()
            if curtime - lasttime > angle_every_seconds:
                lasttime = curtime
                print(" "*50, end="\r")
                print("Current elevation angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
        print("\nHit the %s endstop on the elevation axis, stopping motor!" % nicename)
        self.stop_motor()

    def move_to_angle(self, angle, speed=100):
        #This function simply runs the motor in the correct direction until the curangle of the encoder matches the supplied angle
        #Determine which way to turn the motor
        if angle > self.encoder.curangle:
            movefun = self.move_motor_dir1
            endstop = self.blue_endstop
            nicename = "blue"
            #Could save a few lines of code ignoring the sign information, but we need it to be sure we didnt go past the target.
            anglesign = 1
        else:
            movefun = self.move_motor_dir2
            endstop = self.yellow_endstop
            nicename = "yellow"
            anglesign = -1


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
                    self.EL_motor_PWM_out.ChangeDutyCycle(speed*SLOWDOWN_PCT)
                #Slowdown again when we are SLOWDOWN_DEGREES/2 degrees away
                if anglesign * (angle - self.encoder.curangle) < SLOWDOWN_DEGREES/2:
                    self.EL_motor_PWM_out.ChangeDutyCycle(speed*SLOWDOWN_PCT*SLOWDOWN_PCT)
                #Last slowdown for super fine movements
                if anglesign * (angle - self.encoder.curangle) < 1:
                    self.EL_motor_PWM_out.ChangeDutyCycle(speed*SLOWDOWN_PCT*SLOWDOWN_PCT*SLOWDOWN_PCT)

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

    #Quick homing will just move to the blue endstop and reset the encoder's internal angle variable to 0
    def quickhome(self):
        #Rough home at high speed towards the blue endstop
        self.move_until_stop(direction=2)
        #Not sure this helps accuracy much but I think for consistency its a good idea
        #After finding home first time we back up a few degrees and find it again at a slower speed.
        print("Found rough home, backing up for slow approach to find precise home.")
        #Reset encoder class's current position to re-zero
        self.encoder.reset_position()
        self.move_motor_dir1() #Move backwards a second and find it again, slower.
        sleep(1)
        self.stop_motor()
        print("\nFinding precise home position...")
        self.move_until_stop(direction=2, speed=5)
        print("\nFound final home position!")
        self.stop_motor()
        #Reset encoder position for the final time, should be an accurate and repeatable zero.
        self.encoder.reset_position()


    #Calibrate home will find the blue endstop, then back up and slowly find the precise location of the blue endstop.
    #It will then set its angle var to 0 and move to the other endstop. When it finds the other endstop it again backs
    #up to slowly find the precise location of the yellow endstop. When it finds it, it takes the total number of ticks
    # and divides it by 2 to find the middle. If the endstops are symmetrical in their positions, that middle should be 90 degrees vertical.
    def calhome(self):
        self.move_until_stop(direction=2)
        self.move_motor_dir1()
        sleep(1)
        self.stop_motor()
        self.move_until_stop(direction=2, speed=5)
        print("Found precise location of yellow endstop. Now finding blue endstop...")
        self.encoder.reset_position()
        self.move_until_stop(direction=1)
        self.move_motor_dir2()
        sleep(1)
        self.stop_motor()
        self.move_until_stop(direction=1, speed=5)
        print("Found precise location of blue endstop at %s ticks from blue endstop. Now parking at 90-degrees (vertical)..." % self.encoder.curtick)
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



class AzMotorControl(threading.Thread):
    def __init__(self, AIN1=AZ_AIN1, AIN2=AZ_AIN2, PWMA=AZ_PWMA, he_sensor=AZ_MAG_SENSOR):
        super(AzMotorControl, self).__init__()
        self.encoder = RotaryEncoder(AZ_POS_OUTA, AZ_POS_OUTB, AZ_ANGLE_TICK)
        self.ain1 = AIN1
        self.ain2 = AIN2
        self.pwma = PWMA
        self.he_sensor = he_sensor
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

    #Start of threaded operation. We'll home the axis and then begin the move_to_command_angle() loop
    def run(self):
        self.find_home()
        print("\n\n")
        ops = 100
        while self.quitting is False:
            self.move_to_commanded_angle()
            #print(" "*100, end="\r")
            #print("Current angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
            sleep(1.0/ops)

        print("AzMotorControl thread received quit signal. Exiting thread...")
        self.quitthread()

    def quitthread(self):
        #If we were called externally then set the internal quitting flag true so the loop can stop
        self.quitting = True
        #Stop the PWM channel or it causes problems when quitting python
        #Final app exit will handle the GPIO.cleanup()
        self.AZ_motor_PWM_out.stop()

    def move_to_commanded_angle(self):
        # This function is constantly called as a background thread, constantly evaluating and commanding movements.
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
        if anglediff < self.encoder.ANGLE_TICK:
            self.stop_motor()
            return # Can't reach exact commanded value because its between the smallest increment we can detect.

        movespeed = 100 # PWM duty cycle is between 0 and 100
        if anglediff > 5:
            pass # no slowdown, 100 duty cycle
        elif anglediff > 2: # Between 5 and 2 degrees, first slowdown
            movespeed *= SLOWDOWN_PCT
        elif anglediff > 1: #Between 1 and 2 degrees, second slowdown
            movespeed *= SLOWDOWN_PCT**2
        elif anglediff >= self.encoder.ANGLE_TICK*3: #between a degree difference and 3 ANGLE_TICKs
            movespeed *= SLOWDOWN_PCT**3 # Duty cycle of 1.5625%
        else: # Between 3 ticks and our target we really crawl slowly
            movespeed = 0.1 #Slowest move speed possible

        #Ok now we know what motor to activate and how fast it should move.
        movefun(movespeed)

    def stop_motor(self):
        self.AZ_motor_PWM_out.ChangeDutyCycle(0)
        GPIO.output(self.ain1, GPIO.LOW)
        GPIO.output(self.ain2, GPIO.LOW)

    def move_motor_dir1(self, speed=100):
        self.AZ_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.ain1, GPIO.HIGH)
        GPIO.output(self.ain2, GPIO.LOW)

    def move_motor_dir2(self, speed=100):
        self.AZ_motor_PWM_out.ChangeDutyCycle(speed)
        GPIO.output(self.ain1, GPIO.LOW)
        GPIO.output(self.ain2, GPIO.HIGH)

    def move_until_stop(self, speed=100):
        self.move_motor_dir2(speed=speed)
        stopstate = GPIO.input(self.he_sensor)
        angle_every_seconds = 0.1 #Output current angle every 2 seconds
        curtime = time()
        lasttime = curtime
        while stopstate > 0:
            sleep(0.1)
            stopstate = GPIO.input(self.he_sensor)
            curtime = time()
            if curtime - lasttime > angle_every_seconds:
                lasttime = curtime
                print(" "*50, end="\r")
                print("Current azimuth angle: %s degrees [%s]" % (self.encoder.getCurrentAngle(), self.encoder.getCurrentTick()), end="\r")
        print("\nHit azimuth endstop, stopping motor!")
        self.stop_motor()

    def move_to_angle(self, angle, speed=100):
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
                    self.AZ_motor_PWM_out.ChangeDutyCycle(speed*SLOWDOWN_PCT)
                #Slowdown again when we are SLOWDOWN_DEGREES/2 degrees away
                if anglesign * (angle - self.encoder.curangle) < SLOWDOWN_DEGREES/2:
                    self.AZ_motor_PWM_out.ChangeDutyCycle(speed*SLOWDOWN_PCT*SLOWDOWN_PCT)
                #Last slowdown for super fine movements
                if anglesign * (angle - self.encoder.curangle) < 1:
                    self.AZ_motor_PWM_out.ChangeDutyCycle(speed*SLOWDOWN_PCT*SLOWDOWN_PCT*SLOWDOWN_PCT)

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
                    print("")
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
        #Rough home at high speed
        self.move_until_stop()
        #Not sure this helps accuracy much but I think for consistency its a good idea
        #After finding home first time we back up a few degrees and find it again at a slower speed.
        print("Found rough home, backing up for slow approach to find precise home.")
        #Reset encoder class's current position to re-zero
        self.encoder.reset_position()
        self.move_to_angle(8)
        print("\nFinding precise home position...")
        self.move_until_stop(speed=5)
        print("\nFound final home position! Resetting counters. Be warned that endstops are not used to prevent over-travel on the azimuth axis.")
        #Reset encoder position for the final time, should be an accurate and repeatable zero.
        self.encoder.reset_position()


def terminal_interface(azmc, elmc):
    inp = ""
    while inp.lower() != "quit" and inp.lower() != "q":
        inp = input("CMD: ").strip().lower()
        if inp == "p" or inp == "pos" or inp == "position":
            print("Current mast position is: %s degrees [%s]\n" % (motorcontrol.encoder.getCurrentAngle(), motorcontrol.encoder.getCurrentTick()))

        elif inp == "h" or inp == "home":
            print("Finding home position, current position is: %s degrees [%s]\n" % (motorcontrol.encoder.getCurrentAngle(), motorcontrol.encoder.getCurrentTick()))
            motorcontrol.find_home()

        elif inp[:2] == "go" or inp[:2] == "sg":
            inputangle = inp[3:]
            try:
                inputangle = int(inputangle)
            except:
                print("Incorrect format, command requires a number between 0 and 190. E.g. GO 95  would turn to 95 degrees\n")
                continue
            if inputangle < 0 or inputangle > MAX_AZ_INPUT_ANGLE:
                print("Angle out of range, must be between 0 and 190.\n")
                continue
            #Convert angle to tick number
            angletick = round(inputangle / AZ_ANGLE_TICK)

            printstr = "Rotating antenna mast to %s degrees [%s]...\n" % (inputangle, angletick)
            if inp[:2] == "sg": #Slow movement mode
                movespeed = 25 # This is the duty cycle, 0-100
                printstr = "Slowly r" + printstr[1:]
            else:
                movespeed = 100
            print(printstr)
            motorcontrol.move_to_angle(inputangle, movespeed)
        elif inp == "d1":
            try:
                motorcontrol.move_motor_dir1()
            except:
                pass
        elif inp == "d2":
            try:
                motorcontrol.move_motor_dir2()
            except:
                pass
        elif inp == "v" or inp == "version":
            print("Antenna Control version %s" % _VERSION_)

        sleep(0.5)

def main():
    print("Welcome to Satellite Antenna Control v%s!" % _VERSION_)
    print("Setting up board and homing axes...")

    azmc = AzMotorControl()
    elmc = ElMotorControl()

    try:
        #Now we're zero'd up, we can accept commands from the user as to where we should turn
        terminal_interface(azmc, elmc)
    except:
        import traceback
        traceback.print_exc()
    finally:
        print("Satellite Antenna control program exit, cleaning up threads and GPIO...")
        azmc.quitthread()
        elmc.quitthread()
        azmc.join()
        elmc.join()
        GPIO.cleanup()


if __name__ == "__main__":
   main()