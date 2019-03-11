#!/usr/bin/env python

# Libraries
import math
from transitions import Machine

#Motors and actuators libraries
from roboclaw import Roboclaw 
#Servo libraries
import RPi.GPIO as GPIO
import time


class scienceLab (object):

    #Flags
    samples=6
    sample=6
    start=True
    gyroChange=False
    fullSample=True
    upPossition=True
    passedSample=True

    def __init__(self, name, port, baud, pwmLimit, motorsID):
        self.name=name
        self.port=port
        self.baud=baud
        self.pwmLimit=pwmLimit
        self.motorsID=motorsID
        self.rc = self.createRC(port,baud) #128 ttyACM0
        self.open(self.rc, port)


        #self.motor1=0 #129 Puerto 1 cable blanco, M1 transition, M2 broca
        #self.motor2=1
        #Machine states
        self.states=[
        {'name':'home', 'on_enter': 'preHome'}, 
        {'name':'extraction', 'on_enter': 'preExtraction', 'on_exit': 'postExtraction'}, 
        {'name':'break', 'on_enter': 'preBreak', 'on_exit': 'postBreak'}, 
        {'name':'transition', 'on_enter': 'preTransition', 'on_exit': 'postTransition'}, 
        {'name':'empty', 'on_enter': 'preEmpty', 'on_exit': 'postEmpty'}, 
        {'name':'fullSpeed', 'on_enter': 'preFullSpeed'}, 
        {'name':'nextSample', 'on_enter': 'preNextSample'}, 
        {'name':'idle', 'on_enter': 'preIdle', 'on_exit': 'postIdle'},  
        ]

        #Transitions
        self.transitions = [
        { 'trigger': 'drill', 'source': 'home', 'dest': 'extraction'},
        { 'trigger': 'punch', 'source': 'extraction', 'dest': 'break'},
        { 'trigger': 'retry', 'source': 'break', 'dest': 'extraction'},    
        { 'trigger': 'giveup', 'source': 'break', 'dest': 'idle'},
        { 'trigger': 'store', 'source': 'extraction', 'dest': 'transition'},
        { 'trigger': 'fill', 'source': 'transition', 'dest': 'empty'},
        { 'trigger': 'next', 'source': 'empty', 'dest': 'nextSample'},
        { 'trigger': 'spin', 'source': 'empty', 'dest': 'fullSpeed'},
        { 'trigger': 'wait', 'source': 'nextSample', 'dest': 'idle'},
        { 'trigger': 'wait', 'source': 'fullSpeed', 'dest': 'idle'},
        { 'trigger': 'drill', 'source': 'idle', 'dest': 'extraction'},
        ]

        #Init servos
        SERVO1=32
        SERVO2=33

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO1, GPIO.OUT) #Set pins as outputs
        GPIO.setup(SERVO2, GPIO.OUT)
        
        self.servo1 = GPIO.PWM(SERVO1, 50) #Pin and pulses per second
        self.servo2 = GPIO.PWM(SERVO2, 50) 

        #DutyCycle=((Angle/180)+1)*5
        self.servo1.start(4.5) #Center servo
        self.servo2.start(13) #Center servo
        
        #Create Machine
        self.machine = Machine (model=self, states=self.states, transitions=self.transitions, initial='home')

        #self.preHome()
        self.test()

    #Init functions
    def open(self, rc, port):
        if rc.Open():
            print("RC in: "+port)
        else: 
            exit("Error: cannot open port: " + port)

    def createRC(self,port,baud):
        listrc = Roboclaw('/dev/' + port, baud)
        return listrc

	#128 m1 actuator m2 broca, 129 m1 centrifug m2 transition
    def slow (self,servo, duty, new):
        aux=duty
        if duty<new:
		    while (aux<=new):
		        servo.ChangeDutyCycle(aux)
		        aux=aux+0.1
		        time.sleep(.007)
        elif duty>new:
		    while (aux>=new):
		        servo.ChangeDutyCycle(aux)
		        aux=aux-0.1
		        time.sleep(.007)

        

    #Test function
    def test(self):
        #Extraction

        '''#Transition
        traPWM=3

        self.rc.BackwardM1(129, traPWM)
        self.rc.BackwardM1(129,0)'''
        
        extPWM=100
        self.rc.ForwardM1(128,extPWM)
        self.rc.ForwardM2(128,120)
        time.sleep(3.7)
        self.rc.BackwardM1(128,0)
        time.sleep(5)
        self.rc.BackwardM1(128,extPWM)
        time.sleep(3.5)
        self.rc.ForwardM1(128,0)
        self.rc.ForwardM2(128, 0)
        
        #Transition sube
        traPWM=75
        self.rc.ForwardM1(129,traPWM)
        self.rc.ForwardM1(129,0)

        #Empty sample
        #time.sleep(1)
        #self.slow(self.servo2,13,11.9) #recibe muestra
        time.sleep(1)
        self.slow(self.servo1,4.5,11) # vacia muestra
        time.sleep(1)
        self.slow(self.servo1, 11, 4.5) #regresa
        #time.sleep(1)
        #self.slow(self.servo2,11.9,13) #recibe muestra
        
        #Transition regresa
        self.rc.BackwardM1(129, traPWM)
        self.rc.BackwardM1(129,0)
 
        #Levanta rampa y gira
        time.sleep(1)
        self.slow(self.servo2, 13, 5)
        time.sleep(4)
        self.slow(self.servo2, 5, 13)

        #Centrifuge
        spinPWM=120
        #self.rc.ForwardM2(129,spinPWM)
        time.sleep(5)
        #self.rc.ForwardM2(129,0)

        '''n=.5
        self.drill()
        time.sleep(n)
        self.store()
        time.sleep(n)
        self.fill()
        time.sleep(n)
        self.spin()
        time.sleep(n)
        self.wait()'''

    #Changes when entering a state
    def preHome(self):
        print('Going home')
        #Linear Actuator goes up
        self.rc.BackwardM1(128,self.pwmLimit)
        #Centrifuge centered
        self.rc.ForwardM1(129,self.pwmLimit)
        #Pololu down
        #self.rc.ForwardM2(129,self.pwmLimit)
		#

    def preExtraction(self):
        print('Drilling')
        #Linear Actuator goes down
        self.rc.ForwardM1(128,self.pwmLimit)
        #Servo city 1 rotates
        self.rc.ForwardM2(128,self.pwmLimit)

    def preBreak(self):
        print('Start hitting')
        #Linear Actuator goes up
        self.rc.BackwardM1(128,0)

    def preTransition(self ):
        print('Moving sample')
        #Pololu rotates to go up
        self.rc.BackwardM1(129,self.pwmLimit)
        #Servo 1 evades the sample slide
        self.servo1.ChangeDutyCycle(5)
        #Stop actuator
        self.rc.BackwardM1(128,0)

    def preEmpty(self):
        print('Passing sample')
        #Servo 1 throws the sample
        self.servo1.ChangeDutyCycle(10)

    def preFullSpeed(self):
        print('Max speed')
        #Servo city 2 max speed
        #self.rc.ForwardM2(129,self.pwmLimit)

    def preNextSample(self):
        print('Preparing next sample')
        #Servo city 2 new possition

    def preIdle(self):
        print('Waiting next sample')
        #Servo city 2 stops
        #self.rc.ForwardM1(129, 0)
        #Pololu
        self.rc.ForwardM1(129, self.pwmLimit)
        #time.sleep(0.5)
        self.rc.ForwardM1(129, 0)

    #Changes when leaving a state
    def postExtraction(self):
        print('Stop drilling')
        #Linear Actuator goes up
        self.rc.BackwardM1(128,self.pwmLimit)
        #Servo city 1 stops
        self.rc.BackwardM2(128,0)


    def postBreak(self):
        print('Stop hitting')
        #Linear Actuator goes up
        self.rc.BackwardM1(128,self.pwmLimit)

    def postTransition(self ):
        print('Sample in possition')
        #Pololu stops
        self.rc.ForwardM1(129,0)
        #Servo 1 sample centerd
        self.servo1.ChangeDutyCycle(7.5)

    def postEmpty(self):
        print('Sample passed')
        #Servo 1 stops
        self.servo1.ChangeDutyCycle(7.5)
        #Servo 2 vibrates
        for i in range (0,20):
            time.sleep(0.1)
            self.servo2.ChangeDutyCycle(10)
            time.sleep(0.1)
            self.servo2.ChangeDutyCycle(15) 
        #Returns to centered possition
        self.servo2.ChangeDutyCycle(7.5)

    def postIdle(self):
        print('New sample')
        #Pololu goes down with sample box
        #self.rc.ForwardM2(129,self.pwmLimit)

#!/usr/bin/env python
