import RPi.GPIO as GPIO
import time
import pygame
import serial
import cv2
from pygame.locals import *
import threading
import numpy as np
GPIO.setwarnings(False)



# initialize the pygame modules
pygame.init()
clock = pygame.time.Clock()

# set up window to initialize keyboard events for pygame
screen = pygame.display.set_mode((640,480))

# initialize the left & right motors to the desired GPIO ports
# blue: speed
# white: direction
# yellow: information from motor
blueSpeedLeft = 12
whiteDirectionLeft = 22
yellowCheckerLeft = 18
blueSpeedRight = 13
whiteDirectionRight = 23
yellowCheckerRight = 24

# setup the GPIO pins input/output functionality
GPIO.setmode(GPIO.BCM)
GPIO.setup(blueSpeedLeft, GPIO.OUT)
GPIO.setup(whiteDirectionLeft, GPIO.OUT)
GPIO.setup(yellowCheckerLeft, GPIO.IN)
GPIO.setup(blueSpeedRight, GPIO.OUT)
GPIO.setup(whiteDirectionRight, GPIO.OUT)
GPIO.setup(yellowCheckerRight, GPIO.IN)

# create the two PWM objects, and set the frequencies for the pins connected to the speed for each motor
blueSpeedLeft = GPIO.PWM(12, 2000)
blueSpeedRight = GPIO.PWM(13, 2000)

# start the two PWMs at 99% duty cycle, forcing it to begin stopped
blueSpeedLeft.start(99)
blueSpeedRight.start(99)

# initialize motor directions as opposites, this ensures robot going straight
GPIO.output(whiteDirectionLeft, GPIO.LOW)
GPIO.output(whiteDirectionRight, GPIO.HIGH)

# handle speeds effected by turning differential
def cycleChange(curCycle, addedVal):
    if (curCycle + addedVal > 40):
        return 40
    elif (curCycle + addedVal < 0):
        return 0
    return curCycle + addedVal

# intialize all potential keys to be used
keys = {
    pygame.K_LEFT: False,
    pygame.K_RIGHT: False,
    pygame.K_UP: False,
    pygame.K_DOWN: False,
    pygame.K_RSHIFT: False,
    pygame.K_q: False,
    pygame.K_d: False,
    pygame.K_f: False,
    pygame.K_g: False,
    pygame.K_c: False
}

forwardSpeedDutyCycleLeft = 99
forwardSpeedDutyCycleRight = 99
reverseSpeedDutyCycleLeft = 99
reverseSpeedDutyCycleRight = 99
leftPress = 0
rightPress = 0
forwardPress = 0
reversePress = 0
spinSpeedLeft = 99
spinSpeedRight = 99

# initialize serial information for laser module
ser = serial.Serial()
ser.port = '/dev/ttyUSB0' #Note usb port should reflect which port the module is plugged in to - use ls /dev/ttyUSB* command to identify port
ser.baudrate = 115200
ser.bytesize = 8
ser.parity = 'N'
ser.topbits = 1
ser.timeout = .5
ser.open()


# create beeping function
stopThread = True
global_flag = True
beepOnPulse = False
beepOnContinue = False
globalBeep_flag = True
def beeping():
    GPIO.setup(5, GPIO.OUT)
    p = GPIO.PWM(5, 1000)
    f = GPIO.PWM(5,1)
    f.start(70)
    while globalBeep_flag:
        if beepOnPulse:
            #laser on
            ser.write(b'~0106003000014805\r\n')
            
            #distance pull
            ser.write(b'~01030100000185F6\r\n')
            time.sleep(0.0025)
            ser.reset_input_buffer()
            #data=ser.read_until(b'\r\n')
            data= ser.readline()
            print(data)
            #decode from bytes to string for formatting
            data = data.decode('utf-8')
            #splice out the distance from the complete message - cast it as an int to eliminate leading 0's
            distance=int(data[9:13], 16)
            sleepTime = distance/1000
            p.start(50)
            time.sleep(sleepTime)
            p.stop()
            time.sleep(sleepTime)
            #totalTime = totalTime + (sleepTime*2)
            #print("Distance is: %smm "%distance)
        if beepOnContinue:
            #laser on
            ser.write(b'~0106003000014805\r\n')
            #distance pull
            ser.write(b'~01030100000185F6\r\n')
            ser.reset_input_buffer()
            data=ser.read_until(b'\r\n')
            #decode from bytes to string for formatting
            data = data.decode('utf-8')
            #splice out the distance from the complete message - cast it as an int to eliminate leading 0's
            distance=int(data[9:13], 16)
            time.sleep(.01)
            #totalTime = totalTime + .01
            #change the frequency based off of distance
            freq = (100/distance)*600
            f.ChangeFrequency(freq)
        time.sleep(0.001)
t1 = threading.Thread(target = beeping)
t1.start()

capture = cv2.VideoCapture('/dev/video0')
capture.set(cv2.CAP_PROP_FPS,15)
cameraOn = False
globalCamera_flag = True
def cameraModule():
    while globalCamera_flag:
        if cameraOn:
            ret, frame = capture.read()
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            surface = pygame.surfarray.make_surface(np.rot90(frame))
            screen.blit(surface, (0,0))
            pygame.display.update()
        time.sleep(0.03)

t2 = threading.Thread(target = cameraModule)
t2.start()


def printControlOption():
    print("\nPress the following commands:")
    print("   Left-Arrow : Left  Right-Arrow : Right  ")
    print("   Up-Arrow : Forward  Down-Arrow: Reverse  ")
    print("   s: Laser OFF")
    print("   a: Laser ON")
    print("   s: Laser OFF")
    print("   d: Return the distance in millimeter")
    print("   f: pitch based on distance ON/OFF")
    print("   g: beep based on distance ON/OFF")
    print("   c: Camera ON/OFF; Warning: high CPU cost, will impact robot controls")
printControlOption()

# loop to handle continuous input
while global_flag:
    # process events in the queue
    pygame.event.pump()
    # store the pressed key
    keys = pygame.key.get_pressed()
    
    # q quits the loop
    if keys[pygame.K_q]:
        globalBeep_flag = False
        globalCamera_flag = False
        global_flag = False
    
    # f weird beep
    if keys[pygame.K_f]:
        print("press f")
        if beepOnContinue == False:
            beepOnPulse = False
            beepOnContinue = True
            globalBeep_flag = True
            pass
        elif beepOnContinue == True:
            beepOnContinue = False
            globalBeep_flag = False
            pass
        time.sleep(1)
    # g cool beep
    if keys[pygame.K_g]:
        print("press g")
        if beepOnPulse == False:
            beelOnContinue = False
            beepOnPulse = True
            globalBeep_flag = True
            pass
        elif beepOnPulse == True:
            beepOnPulse = False
            globalBeep_flag = False
            pass
        time.sleep(1)
    
    # c camera
    if keys[pygame.K_c]:
        print("press c")
        if cameraOn == False:
            cameraOn = True
            pass
        elif cameraOn == True:
            cameraOn = False
            screen.fill((0,0,0))
            pygame.display.update()
            pass
        time.sleep(1)
        
    # forward press
    if keys[pygame.K_UP]:
        # left/right speed differential changes between the motors depending on right/left movement while driving
        if (keys[pygame.K_LEFT]):
            if leftPress > -5 and leftPress < 30:
                leftPress = leftPress + 5
        if (keys[pygame.K_RIGHT]):
            if rightPress > -5 and rightPress < 30:
                rightPress = rightPress + 5
        # change forward speed if there is no reverse speed
        if (forwardSpeedDutyCycleLeft >= 2  and forwardSpeedDutyCycleRight >= 2) and (reverseSpeedDutyCycleLeft > 40 and reverseSpeedDutyCycleRight > 40):
            GPIO.output(whiteDirectionLeft, GPIO.LOW)
            GPIO.output(whiteDirectionRight, GPIO.HIGH)
            # ensure it starts at slowest speed, which is 40% duty cycle
            if forwardSpeedDutyCycleLeft > 45  and forwardSpeedDutyCycleRight > 45:
                forwardSpeedDutyCycleLeft = 40
                forwardSpeedDutyCycleRight = 40
                blueSpeedLeft.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleLeft,leftPress))
                blueSpeedRight.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleRight,rightPress))
            # increase speed, or decrease duty cycle
            forwardSpeedDutyCycleLeft = forwardSpeedDutyCycleLeft - 4
            forwardSpeedDutyCycleRight = forwardSpeedDutyCycleRight - 4
            blueSpeedLeft.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleLeft,leftPress))
            blueSpeedRight.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleRight,rightPress))
        # if there is reverse speed, first degrade reverse
        elif reverseSpeedDutyCycleLeft <= 40 and reverseSpeedDutyCycleRight <= 40:
            GPIO.output(whiteDirectionLeft, GPIO.HIGH)
            GPIO.output(whiteDirectionRight, GPIO.LOW)
            reverseSpeedDutyCycleLeft = reverseSpeedDutyCycleLeft + 8
            reverseSpeedDutyCycleRight = reverseSpeedDutyCycleRight + 8
            blueSpeedLeft.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleLeft,leftPress))
            blueSpeedRight.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleRight,rightPress))
        # allow buffer to give motor time to move
        time.sleep(0.2)
        pass
    # slow when not pressing forward
    if (not keys[pygame.K_UP]) and  (forwardSpeedDutyCycleLeft < 43  and forwardSpeedDutyCycleRight < 43):
        # make sure you only decrease forward speed when not reversing
        if reverseSpeedDutyCycleLeft >40 and reverseSpeedDutyCycleRight > 40:
            GPIO.output(whiteDirectionLeft, GPIO.LOW)
            GPIO.output(whiteDirectionRight, GPIO.HIGH)
            forwardSpeedDutyCycleLeft = forwardSpeedDutyCycleLeft + 8
            forwardSpeedDutyCycleRight = forwardSpeedDutyCycleRight + 8
            blueSpeedLeft.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleLeft,leftPress))
            blueSpeedRight.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleRight,rightPress))
            # ensure stops when duty cycle exceeds 40%, just make it highest duty cycle
            if forwardSpeedDutyCycleLeft >= 40  and forwardSpeedDutyCycleRight >= 40:
                    forwardSpeedDutyCycleLeft =  99
                    forwardSpeedDutyCycleRight = 99
                    blueSpeedLeft.ChangeDutyCycle(99)
                    blueSpeedRight.ChangeDutyCycle(99)
            time.sleep(0.1)
        pass
        
    # reverse
    # setting GPIO.HIGH or GPIO.LOW such that the white direction is opposite for both motors
    if keys[pygame.K_DOWN]:
        # left/right speed differential changes between the motors depending on right/left movement while driving
        if (keys[pygame.K_LEFT]):
            if leftPress > -5 and leftPress < 30:
                leftPress = leftPress + 5
        if (keys[pygame.K_RIGHT]):
            if rightPress > -5 and rightPress < 30:
                rightPress = rightPress + 5
        # change reverse speed if there is no forward speed
        if (forwardSpeedDutyCycleLeft >=40 and forwardSpeedDutyCycleRight >= 40) and (reverseSpeedDutyCycleRight >= 2 and reverseSpeedDutyCycleLeft >= 2) :
            GPIO.output(whiteDirectionLeft, GPIO.HIGH)
            GPIO.output(whiteDirectionRight, GPIO.LOW)
            # ensure it starts at slowest speed, which is 40% duty cycle
            if reverseSpeedDutyCycleLeft > 45  and reverseSpeedDutyCycleRight > 45:
                reverseSpeedDutyCycleLeft = 40
                reverseSpeedDutyCycleRight = 40
                blueSpeedLeft.ChangeDutyCycle(cycleChange(reverseSpeedDutyCycleLeft,leftPress))
                blueSpeedRight.ChangeDutyCycle(cycleChange(reverseSpeedDutyCycleRight,rightPress))
            # increase reverse speed, or decrease duty cycle
            reverseSpeedDutyCycleLeft = (reverseSpeedDutyCycleLeft - 4)
            reverseSpeedDutyCycleRight = (reverseSpeedDutyCycleRight - 4)
            blueSpeedLeft.ChangeDutyCycle(cycleChange(reverseSpeedDutyCycleLeft,leftPress))
            blueSpeedRight.ChangeDutyCycle(cycleChange(reverseSpeedDutyCycleRight,rightPress))
        # if there is reverse speed, first degrade reverse
        elif (forwardSpeedDutyCycleLeft < 40 and forwardSpeedDutyCycleRight < 40):
            forwardSpeedDutyCycleLeft = forwardSpeedDutyCycleLeft + 8
            forwardSpeedDutyCycleRight = forwardSpeedDutyCycleRight + 8
            blueSpeedLeft.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleLeft,leftPress))
            blueSpeedRight.ChangeDutyCycle(cycleChange(forwardSpeedDutyCycleRight,rightPress))
        # allow buffer to give motor time to move
        time.sleep(0.2)
        pass
    # slow when not pressing reverse
    if (not keys[pygame.K_DOWN]) and (reverseSpeedDutyCycleLeft < 43  and reverseSpeedDutyCycleRight < 43):
        # make sure you only decrease reverse speed when not going forward
        if forwardSpeedDutyCycleLeft >40 and forwardSpeedDutyCycleRight > 40:
            GPIO.output(whiteDirectionLeft, GPIO.HIGH)
            GPIO.output(whiteDirectionRight, GPIO.LOW)
            reverseSpeedDutyCycleLeft = reverseSpeedDutyCycleLeft + 8
            reverseSpeedDutyCycleRight = reverseSpeedDutyCycleRight + 8
            blueSpeedLeft.ChangeDutyCycle(cycleChange(reverseSpeedDutyCycleLeft,leftPress))
            blueSpeedRight.ChangeDutyCycle(cycleChange(reverseSpeedDutyCycleRight,rightPress))
            # ensure stops when duty cycle exceeds 40%, just make it highest duty cycle
            if reverseSpeedDutyCycleLeft >= 40  and reverseSpeedDutyCycleRight >= 40:
                reverseSpeedDutyCycleLeft = 99
                reverseSpeedDutyCycleRight = 99
                blueSpeedLeft.ChangeDutyCycle(99)
                blueSpeedRight.ChangeDutyCycle(99)
            time.sleep(0.1)
        pass
    
    # ensure that the left/right differential from turning degrades when not pressing left/right
    if not keys[pygame.K_RIGHT] and ((not keys[pygame.K_LEFT]) or (keys[pygame.K_LEFT])) and rightPress != 0:
        if rightPress > 0:
            rightPress = rightPress - 5
        elif rightPress < 0:
            rightPress = rightPress + 5
        if rightPress == 0 and leftPress == 0:
            forwardSpeedDutyCycleLeft = forwardSpeedDutyCycleRight
            reverseSpeedDutyCycleLeft = reverseSpeedDutyCycleRight
        time.sleep(0.05)
        pass
    if not keys[pygame.K_LEFT] and ((not keys[pygame.K_RIGHT]) or (keys[pygame.K_RIGHT])) and leftPress != 0:
        if leftPress > 0:
            leftPress = leftPress - 5
        elif leftPress < 0:
            leftPress = leftPress + 5
        if rightPress == 0 and leftPress == 0:
            forwardSpeedDutyCycleLeft = forwardSpeedDutyCycleRight
            reverseSpeedDutyCycleLeft = reverseSpeedDutyCycleRight
        time.sleep(0.05)
        pass
        
    # rotate left in place, ensure that no forward or reverse movement is ongoing
    if keys[pygame.K_LEFT]and (forwardSpeedDutyCycleLeft == 99 and forwardSpeedDutyCycleRight == 99 and reverseSpeedDutyCycleLeft == 99 and reverseSpeedDutyCycleRight == 99):
        if(spinSpeedLeft >= 2 and spinSpeedRight > 40) :
            # spin in opposite direction
            GPIO.output(whiteDirectionLeft, GPIO.HIGH)
            GPIO.output(whiteDirectionRight, GPIO.HIGH)
            # start spin speed at slowest speed
            if spinSpeedLeft > 45:
                spinSpeedLeft = 40
            spinSpeedLeft = spinSpeedLeft - 2
            blueSpeedLeft.ChangeDutyCycle(spinSpeedLeft)
            blueSpeedRight.ChangeDutyCycle(spinSpeedLeft)
        # if spinning right, slow right spin before spinning left
        elif(spinSpeedRight <43):
            spinSpeedRight = spinSpeedRight +4
            GPIO.output(whiteDirectionLeft, GPIO.LOW)
            GPIO.output(whiteDirectionRight, GPIO.LOW)
            blueSpeedLeft.ChangeDutyCycle(spinSpeedRight)
            blueSpeedRight.ChangeDutyCycle(spinSpeedRight)
            if(spinSpeedRight >=35):
                spinSpeedRight = 99
        time.sleep(0.2)
        pass
    # slow left spin, ensure that there is no forward or reverse movement
    if (not keys[pygame.K_LEFT] and (forwardSpeedDutyCycleLeft == 99 and forwardSpeedDutyCycleRight == 99 and reverseSpeedDutyCycleLeft == 99 and reverseSpeedDutyCycleRight == 99)):
        if spinSpeedLeft < 43:
            spinSpeedLeft = spinSpeedLeft + 2
            if spinSpeedLeft > 35:
                spinSpeedLeft = 99
            blueSpeedLeft.ChangeDutyCycle(spinSpeedLeft)
            blueSpeedRight.ChangeDutyCycle(spinSpeedLeft)
            time.sleep(0.05)
        pass
    
    # rotate right in place, ensure that no forward or reverse movement is ongoing
    if keys[pygame.K_RIGHT] and (forwardSpeedDutyCycleLeft == 99 and forwardSpeedDutyCycleRight == 99 and reverseSpeedDutyCycleLeft == 99 and reverseSpeedDutyCycleRight == 99):
        if(spinSpeedRight >= 2 and spinSpeedLeft > 40):
            # spin in opposite direction
            GPIO.output(whiteDirectionLeft, GPIO.LOW)
            GPIO.output(whiteDirectionRight, GPIO.LOW)
            # start spin speed at slowest speed
            if spinSpeedRight > 45:
                spinSpeedRight = 40
            spinSpeedRight = spinSpeedRight - 2
            blueSpeedLeft.ChangeDutyCycle(spinSpeedRight)
            blueSpeedRight.ChangeDutyCycle(spinSpeedRight)
        # if spinning left, slow left spin before spinning right
        elif(spinSpeedLeft <43) and (forwardSpeedDutyCycleLeft == 99 and forwardSpeedDutyCycleRight == 99 and reverseSpeedDutyCycleLeft == 99 and reverseSpeedDutyCycleRight == 99):
            spinSpeedLeft = spinSpeedLeft +4
            GPIO.output(whiteDirectionLeft, GPIO.HIGH)
            GPIO.output(whiteDirectionRight, GPIO.HIGH)
            blueSpeedLeft.ChangeDutyCycle(spinSpeedLeft)
            blueSpeedRight.ChangeDutyCycle(spinSpeedLeft)
            if(spinSpeedLeft >=35):
                spinSpeedLeft = 99
        time.sleep(0.2)
        pass
    # slow left spin, ensure that there is no forward or reverse movement
    if (not keys[pygame.K_RIGHT]and (forwardSpeedDutyCycleLeft == 99 and forwardSpeedDutyCycleRight == 99 and reverseSpeedDutyCycleLeft == 99 and reverseSpeedDutyCycleRight == 99)):
        if(spinSpeedRight <43):
            spinSpeedRight = spinSpeedRight + 2
            if spinSpeedRight > 35:
                spinSpeedRight = 99
            blueSpeedLeft.ChangeDutyCycle(spinSpeedRight)
            blueSpeedRight.ChangeDutyCycle(spinSpeedRight)
            time.sleep(0.05)
        pass
        
t1.join()
t2.join()
ser.close()
blueSpeedLeft.stop()
blueSpeedRight.stop()
GPIO.cleanup()
pygame.quit()

