# -*- coding: utf-8 -*-
import math, serial, copy
import numpy as np
from sys import exit

ratioA = (80/16)*(64/20)
ratioB = (92/16)*(100/32)
ratioC = (64/16)*(64/20)
ratioD = (64/16)*(64/20)
stepsPerRevolution = 200
degreeOffsetB = 77.97913
degreeOffsetA = 198
degreeOffsetC = 90
degreeOffsetD = 75.9375

PossibleOmegaList = []
RobotGeoAOffset = 28.5
RobotGeoA = 155
RobotGeoB = 153.375
RobotGeoC = 154.126
toolType = 1
RobotGeoD2 = 238.631 #Nástroj 2
RobotGeoD1 = 102.82+70 #Nástroj 1
RobotGeoD = RobotGeoD1

limitStepsAMin = int(0)
limitStepsBMin = int(0)
limitStepsCMin = int(0)
limitStepsDMin = int(0)
limitStepsAMax = int(2900)
limitStepsBMax = int(780)
limitStepsCMax = int(1230)
limitStepsDMax = int(1540)

limitSqueezeTool1Min = int(88)
limitSqueezeTool1Max = int(180)
limitSqueezeTool2Min = int(88)
limitSqueezeTool2Max = int(180)
limitTurnTool1Min = int(0)
limitTurnTool1Max = int(180)
limitTurnTool2Min = int(0)
limitTurnTool2Max = int(180)

gripperAngle = int(180)
wristAngle = int(60)
pyCommand = int(1)
stepsA = int(0)
stepsB = int(0)
stepsC = int(0)
stepsD = int(0)
delaySteppers = int(40)
delayServos = int(30)

toolType1OpenGripper = 180
toolType1ClosedGripper = 88
toolType2OpenGripper = 180
toolType2ClosedGripper = 88

homePosition = [1760, 120, 0, 100, "s"]

def setToolSpeed(servoDelay):
    global delayServos
    delayServos = servoDelay
    
def setRobotSpeed(steppersDelay):
    global delaySteppers
    delaySteppers = int(round(steppersDelay/2))

def chooseTool(Tool):
    global RobotGeoD, toolType
    toolType = Tool
    print("Vybrán nástroj " + str(toolType))
    if toolType==1:
        RobotGeoD = RobotGeoD1
        
    if toolType==2:
        RobotGeoD = RobotGeoD2
    
    if toolType!=2 or toolType!=2:
        "Zadán neplatný nástroj. Byl zvolen nástroj 1!"
        toolType = 1
        RobotGeoD = RobotGeoD1
        
def openGripper():
    global toolType, gripperAngle, wristAngle, toolType1OpenGripper, toolType2OpenGripper, delayServos
    pyCommand = int(7)
    
    if toolType==1:
        gripperAngle = toolType1OpenGripper
        sendDataToRobot(int(1), int(1), int(1), int(1), gripperAngle, wristAngle, pyCommand, int(1), delayServos)

    if toolType==2:
        gripperAngle = toolType2OpenGripper         
        sendDataToRobot(int(1), int(1), int(1), int(1), gripperAngle, wristAngle, pyCommand, int(1), delayServos)

    print("Nástroj otevřen.") 

def closeGripper():
    global toolType, gripperAngle, wristAngle, toolType1ClosedGripper, toolType2ClosedGripper, delayServos
    pyCommand = int(7)
    
    if toolType==1:
        gripperAngle = toolType1ClosedGripper
        sendDataToRobot(int(1), int(1), int(1), int(1), gripperAngle, wristAngle, pyCommand, int(1), delayServos)

    if toolType==2:
        gripperAngle = toolType2ClosedGripper         
        sendDataToRobot(int(1), int(1), int(1), int(1), gripperAngle, wristAngle, pyCommand, int(1), delayServos)

    print("Nástroj uzavřen.")

def sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos):
    global serialcom, toolType
    
    if toolType==1:
        if wristAngle>=limitTurnTool1Min and wristAngle<=limitTurnTool1Max and gripperAngle>=limitSqueezeTool1Min and gripperAngle<=limitSqueezeTool1Max:
            "okay"
        else:
            print("Chyba: Hodnoty natočení serv nástroje jsou mimo rozsah. Přikaz nebyl proveden.")
            print("Program předčasně ukončen!")
            exit()
            
    if toolType==2:
        if wristAngle>=limitTurnTool2Min and wristAngle<=limitTurnTool2Max and gripperAngle>=limitSqueezeTool2Min and gripperAngle<=limitSqueezeTool2Max:
            "okay"
        else:
            print("Chyba: Hodnoty natočení serv nástroje jsou mimo rozsah. Přikaz nebyl proveden.")
            print("Program předčasně ukončen!")
            exit()
            
    if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and ((stepsB>=limitStepsBMin) and (stepsB<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
            byteStepsA = stepsA.to_bytes(2, 'little')
            byteStepsB = stepsB.to_bytes(2, 'little')
            byteStepsC = stepsC.to_bytes(2, 'little')
            byteStepsD = stepsD.to_bytes(2, 'little')
            byteGripperAngle = gripperAngle.to_bytes(1, 'little')
            byteWristAngle = wristAngle.to_bytes(1, 'little')
            bytePyCommand = pyCommand.to_bytes(1, 'little')
            byteDelaySteppers = delaySteppers.to_bytes(1, 'little')
            byteDelayServos = delayServos.to_bytes(1, 'little')
            serialcom.write(byteStepsA)
            serialcom.write(byteStepsB)
            serialcom.write(byteStepsC)
            serialcom.write(byteStepsD)
            serialcom.write(byteGripperAngle)
            serialcom.write(byteWristAngle)
            serialcom.write(bytePyCommand)
            serialcom.write(byteDelaySteppers)
            serialcom.write(byteDelayServos)
            waitForArduino()
    else:
        print("Chyba: Hodnoty natočení pohonů jsou mimo rozsah. Přikaz nebyl proveden.")
        print("Program předčasně ukončen!")
        exit()

def waitForArduino():
    global serialcom
    while True:
       c = serialcom.readline()
       if c==bytes(b'y\n'):
          break
       if c==bytes(b'n\n'):
          print("Chyba: ")
          print(serialcom.readline().decode('ascii'))
          break

def initializeRobot(COM="COM6", BaudRate=int(9600)):
    global serialcom
    serialcom = serial.Serial(COM,BaudRate)
    print("Pro spuštění robota stiskněte zelené tlačítko!")
    while True: 
        greenBttnConfirm= serialcom.readline().decode('ascii')
        if  (greenBttnConfirm=="Arduino pripraveno\n"):
            print("Robot připraven")
            break
        
def readSensor1Value():
    pyCommand = int(8)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    data = serialcom.readline().decode('ascii')
    print("Data ze senzoru 1 přečtena. Jejich hodnota je: " + data)
    data = float(data)
    return data

def tareSensor1():
    pyCommand = int(9)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)  
    print("Hodnota sensoru 1 (váha) byla vynulována.")
    
def degreesToStepsB(degrees):
    degrees = 180 - degrees - degreeOffsetB
    steps = degrees/((360/stepsPerRevolution)/ratioB)
    steps = round(steps)
    return steps

def stepsToDegreesB(steps):
    degrees = ((360/stepsPerRevolution)/ratioB) * steps
    degrees = 180 - degrees - degreeOffsetB
    return degrees

def degreesToStepsA(degrees):
    degrees = degreeOffsetA+degrees
    steps = degrees/((360/stepsPerRevolution)/ratioA)
    steps = round(steps)
    return steps

def stepsToDegreesA(steps):
    degrees = ((360/stepsPerRevolution)/ratioA) * steps
    degrees = -degreeOffsetA + degrees
    return degrees

def degreesToStepsC(degrees):
    degrees = degrees - degreeOffsetC
    steps = degrees/((360/stepsPerRevolution)/ratioC)
    steps = round(steps)
    return steps

def stepsToDegreesC(steps):
    degrees = ((360/stepsPerRevolution)/ratioC) * (steps)
    degrees = degreeOffsetC + degrees
    return degrees

def degreesToStepsD(degrees):
    degrees = degrees - degreeOffsetD
    steps = degrees/((360/stepsPerRevolution)/ratioD)
    steps = round(steps)
    return steps

def stepsToDegreesD(steps):
    degrees = ((360/stepsPerRevolution)/ratioD) * (steps)
    degrees = degreeOffsetD + degrees
    return degrees

def degreesToSteps(degrees, gearBoxRatio):
    steps = degrees/((360/stepsPerRevolution)/gearBoxRatio)
    steps = round(steps)
    return steps

def stepsToDegrees(steps, gearBoxRatio):
    degrees = ((360/stepsPerRevolution)/gearBoxRatio) * steps
    return degrees

def inverseKinematicsEngine(x, y, z, omega):
    R=math.sqrt(x**2 + y**2)
    R3=RobotGeoAOffset
    R2=RobotGeoD * math.cos(omega)
    R1=R-R2-R3
    z3=RobotGeoD * math.sin(omega)
    z1=z-z3-RobotGeoA
    e=math.sqrt(R1**2 + z1**2)
    beta1=math.asin(z1/e)
    beta2=math.acos((e**2 +RobotGeoB**2 - RobotGeoC**2)/(2*e*RobotGeoB))
    beta=beta1+beta2
    gamma=math.acos((RobotGeoB**2 +RobotGeoC**2 - e**2)/(2*(RobotGeoB)*(RobotGeoC)))
    delta1=math.radians(90)
    delta2=math.radians(180)-math.radians(90)-beta1
    delta3=math.radians(180)-beta2-gamma
    delta=delta1+delta2+delta3+omega
    
    kappa = math.atan2(y, x)
    kappa = math.degrees(kappa)
    
    #Výpočet pro 1. a 2. a 4. kvadrant
    if (kappa >= -90) and (kappa <= 180):
        alpha = kappa - 90 

    #Výpočet pro 3. kvadrant
    if (kappa < -90 and kappa >= -180):
        alpha = kappa + 270
    
    kappa = math.radians(kappa)
    alpha = math.radians(alpha)

    return alpha, beta, gamma, delta;  

def inverseKinematics(x, y, z, omega=0, omegaStep=1):
    try:
        alpha, beta, gamma, delta = inverseKinematicsEngine(x, y, z, omega)
        stepsAlpha = degreesToStepsA(math.degrees(alpha))
        stepsBeta = degreesToStepsB(math.degrees(beta))
        stepsGamma = degreesToStepsC(math.degrees(gamma))
        stepsDelta = degreesToStepsD(math.degrees(delta))

        if stepsAlpha < limitStepsAMin or stepsAlpha > limitStepsAMax:
            raise ValueError('Úhel alfa je mimo rozsah.')          
        if stepsBeta < limitStepsBMin or stepsBeta > limitStepsBMax:
            raise ValueError('Úhel beta je mimo rozsah.') 
        if stepsGamma < limitStepsCMin or stepsGamma > limitStepsCMax:
            raise ValueError('Úhel gamma je mimo rozsah.')    
        if stepsDelta < limitStepsDMin or stepsDelta > limitStepsDMax:
            raise ValueError('Úhel delta je mimo rozsah.')

    except:
          global PossibleOmegaList
          PossibleOmegaList = []
          for i in np.arange(-180, 180, omegaStep):
              try:
                  omega = math.radians(i)
                  alpha, beta, gamma, delta = inverseKinematicsEngine(x, y, z, omega)
                  stepsAlpha = degreesToStepsA(math.degrees(alpha))
                  stepsBeta = degreesToStepsB(math.degrees(beta))
                  stepsGamma = degreesToStepsC(math.degrees(gamma))
                  stepsDelta = degreesToStepsD(math.degrees(delta))

                  if stepsAlpha < limitStepsAMin or stepsAlpha > limitStepsAMax:
                      raise ValueError('Úhel alfa je mimo rozsah.')          
                  if stepsBeta < limitStepsBMin or stepsBeta > limitStepsBMax:
                      raise ValueError('Úhel beta je mimo rozsah.') 
                  if stepsGamma < limitStepsCMin or stepsGamma > limitStepsCMax:
                      raise ValueError('Úhel gamma je mimo rozsah.')    
                  if stepsDelta < limitStepsDMin or stepsDelta > limitStepsDMax:
                      raise ValueError('Úhel delta je mimo rozsah.')
                  PossibleOmegaList.append(omega)
              except:
                  ""       
          PossibleOmegaListDif = []
          for i in PossibleOmegaList:
              PossibleOmegaListDif.append(abs(i - omega))
          try:
              index = PossibleOmegaListDif.index(min(PossibleOmegaListDif))
              omega=PossibleOmegaList[index]
          except:
              print("Nelze vypočítat hodnoty úhlů. Vypočítané hodnoty jsou mimo rozsah.")
              print("Program předčasně ukončen!")
              exit()

    alpha, beta, gamma, delta = inverseKinematicsEngine(x, y, z, omega)             
    return alpha, beta, gamma, delta;

def forwardKinematics(alpha, beta, gamma, delta):
    R3=RobotGeoAOffset
    e=math.sqrt(RobotGeoB**2 + RobotGeoC**2 - 2*RobotGeoB*RobotGeoC*math.cos(gamma))
    
    beta2=math.acos((e**2 +RobotGeoB**2 - RobotGeoC**2)/(2*e*RobotGeoB))
    delta3=math.radians(180)-beta2-gamma
    delta1=math.radians(90)
    beta1=beta-beta2
    delta2=math.radians(180)-math.radians(90)-beta1
    R1=math.cos(beta1)*e
    omega=delta-delta1-delta2-delta3
    z3=RobotGeoD * math.sin(omega)
    R2=RobotGeoD * math.cos(omega)
    R=R1+R2+R3
    z1=e*math.sin(beta1)
    z2=z1+RobotGeoA
    z=z2+z3
    
    #Přepočet úhlů
    if math.degrees(alpha) < -180:
       alpha = math.radians(180)-(-alpha-math.radians(180))

    if math.degrees(alpha) > 180:
       alpha = -(math.radians(180)-(alpha-math.radians(180)))

    kappa = alpha+math.radians(90)
  
    y=R*math.sin(kappa)
    x=R*math.cos(kappa)

    return x, y, z, omega;  

def calibrateRobot():
    global stepsA, stepsB, stepsC, stepsD
    pyCommand = int(1)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    stepsA = int(0)
    stepsB = int(0)
    stepsC = int(0)
    stepsD = int(0)
    print("Byla provedena kalibrace všech os.") 
    
def calibrateAxisA():
    global stepsA
    pyCommand = int(2)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    stepsA = int(0)
    print("Byla provedena kalibrace osy A.") 
    
def calibrateAxisB():
    global stepsB
    pyCommand = int(3)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    stepsB = int(0)
    print("Byla provedena kalibrace osy B.") 
    
def calibrateAxisC():
    global stepsC
    pyCommand = int(4)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    stepsC = int(0)
    print("Byla provedena kalibrace osy C.") 
    
def calibrateAxisD():
    global stepsD
    pyCommand = int(5)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    stepsD = int(0)
    print("Byla provedena kalibrace osy D.") 
    
def resetArduino():
    pyCommand = int(199)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    print("Arduino bylo resetováno! Pro jeho opětovnou aktivaci nezapomeňte stisknout zelené tlačítko!") 
    
def allStepsToXYZOmega():
    degA = stepsToDegreesA(stepsA)                        
    degB = stepsToDegreesB(stepsB)
    degC = stepsToDegreesC(stepsC)
    degD = stepsToDegreesD(stepsD)
    
    alpha = math.radians(degA)               
    beta = math.radians(degB)
    gamma = math.radians(degC)
    delta = math.radians(degD)        
 
    x, y, z, omega = forwardKinematics(alpha, beta, gamma, delta)               
    return x, y, z, omega;

def moveRobot(point):
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, PossibleOmegaList
    if len(point)==5:
        if point[4]=="s" or point[4]=="S":
            stepsA = point[0]
            stepsB = point[1]
            stepsC = point[2]
            stepsD = point[3]
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            
        if point[4]=="d" or point[4]=="D":
            alpha = point[0]
            beta = point[1]
            gamma = point[2]
            delta = point[3]
            stepsA = degreesToStepsA(alpha)
            stepsB = degreesToStepsB(beta)
            stepsC = degreesToStepsC(gamma)
            stepsD = degreesToStepsD(delta)
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            
        if point[4]=="r" or point[4]=="R":
            alpha = point[0]
            beta = point[1]
            gamma = point[2]
            delta = point[3]
            stepsA = degreesToStepsA(math.degrees(alpha))
            stepsB = degreesToStepsB(math.degrees(beta))
            stepsC = degreesToStepsC(math.degrees(gamma))
            stepsD = degreesToStepsD(math.degrees(delta))
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            
        if point[4]=="cr" or point[4]=="CR" or point[4]=="rc" or point[4]=="RC":
           x = point[0]
           y = point[1]
           z = point[2]
           omega = point[3]
           alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
           stepsA = degreesToStepsA(math.degrees(alpha))
           stepsB = degreesToStepsB(math.degrees(beta))
           stepsC = degreesToStepsC(math.degrees(gamma))
           stepsD = degreesToStepsD(math.degrees(delta))
           pyCommand = int(6)
           sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
           
        if point[4]=="cd" or point[4]=="CD" or point[4]=="dc" or point[4]=="DC":
            x = point[0]
            y = point[1]
            z = point[2]
            omega = point[3]
            omega = math.radians(omega)
            alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
            stepsA = degreesToStepsA(math.degrees(alpha))
            stepsB = degreesToStepsB(math.degrees(beta))
            stepsC = degreesToStepsC(math.degrees(gamma))
            stepsD = degreesToStepsD(math.degrees(delta))
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)  
        
        if point[4]!="cd" and point[4]!="CD" and point[4]!="dc" and point[4]!="DC" and point[4]!="cr" and point[4]!="CR" and point[4]!="rc" and point[4]!="RC" and point[4]!="R" and point[4]!="r" and point[4]!="d" and point[4]!="D" and point[4]!="s" and point[4]!="S":
           print("Nesprávný formát dat zadané pozice.")
           print("Program předčasně ukončen.")
           exit()
        
    if len(point)==7:
        if point[4]=="s" or point[4]=="S":
            
            stepsA = point[0]
            stepsB = point[1]
            stepsC = point[2]
            stepsD = point[3]
            wristAngle = point[5]
            gripperAngle = point[6]
            pyCommand = int(6)   
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            pyCommand = int(7)  
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            
        if point[4]=="d" or point[4]=="D":
            
            alpha = point[0]
            beta = point[1]
            gamma = point[2]
            delta = point[3]
            wristAngle = point[5]
            gripperAngle = point[6]
            
            stepsA = degreesToStepsA(alpha)
            stepsB = degreesToStepsB(beta)
            stepsC = degreesToStepsC(gamma)
            stepsD = degreesToStepsD(delta)
            
            pyCommand = int(6) 
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            pyCommand = int(7)  
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)

        if point[4]=="r" or point[4]=="R":
            
            alpha = point[0]
            beta = point[1]
            gamma = point[2]
            delta = point[3]
            wristAngle = point[5]
            gripperAngle = point[6]
            
            stepsA = degreesToStepsA(math.degrees(alpha))
            stepsB = degreesToStepsB(math.degrees(beta))
            stepsC = degreesToStepsC(math.degrees(gamma))
            stepsD = degreesToStepsD(math.degrees(delta))
            
            pyCommand = int(6) 
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            pyCommand = int(7)  
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            
        if point[4]=="cr" or point[4]=="CR" or point[4]=="rc" or point[4]=="RC":
            
            x = point[0]
            y = point[1]
            z = point[2]
            omega = point[3]
            alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
            stepsA = degreesToStepsA(math.degrees(alpha))
            stepsB = degreesToStepsB(math.degrees(beta))
            stepsC = degreesToStepsC(math.degrees(gamma))
            stepsD = degreesToStepsD(math.degrees(delta))
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            
            wristAngle = point[5]
            gripperAngle = point[6]
            pyCommand = int(7)  
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            
        if point[4]=="cd" or point[4]=="CD" or point[4]=="dc" or point[4]=="DC":
            
            x = point[0]
            y = point[1]
            z = point[2]
            omega = point[3]
            omega = math.radians(omega)
            alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
            stepsA = degreesToStepsA(math.degrees(alpha))
            stepsB = degreesToStepsB(math.degrees(beta))
            stepsC = degreesToStepsC(math.degrees(gamma))
            stepsD = degreesToStepsD(math.degrees(delta))
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)     
            
            wristAngle = point[5]
            gripperAngle = point[6]
            pyCommand = int(7)  
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)

        if  point[4]!="cd" and point[4]!="CD" and point[4]!="dc" and point[4]!="DC" and point[4]!="cr" and point[4]!="CR" and point[4]!="rc" and point[4]!="RC" and point[4]!="R" and point[4]!="r" and point[4]!="d" and point[4]!="D" and point[4]!="s" and point[4]!="S":
            print("Nesprávný formát dat zadané pozice.")
            print("Program předčasně ukončen.")
            exit()

    if len(point)==2:
        wristAngle = point[0]
        gripperAngle = point[1]
        pyCommand = int(7)   
        sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    print("Robot se nachází v požadované pozici.")

def createLineListXYZ(startPoint, endPoint, splitnumber):
    startPoint = np.array(startPoint)
    endPoint = np.array(endPoint)
    step = np.divide(np.subtract(endPoint, startPoint), splitnumber)
    
    line = []
    addition = startPoint
    for x in range(splitnumber):
      addition = addition + step
      line.append(list(addition)) 
    return line

def offset(point, offsetValue):
    newPoint = copy.copy(point) 
    
    for i in range(len(offsetValue)):
        if i != 4:
            newPoint[i] = newPoint[i] + offsetValue[i]
    return newPoint


def createLineListSteps(startPoint, endPoint, splitnumber, omega=math.radians(-90)):
    line = createLineListXYZ(startPoint, endPoint, splitnumber)
    allSteps = []
    for i in line:
        
        x = i[0]
        y = i[1]
        z = i[2]
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        stepsA = degreesToStepsA(math.degrees(alpha))
        stepsB = degreesToStepsB(math.degrees(beta))
        stepsC = degreesToStepsC(math.degrees(gamma))
        stepsD = degreesToStepsD(math.degrees(delta))
        
        steps = [stepsA, stepsB, stepsC, stepsD]
        allSteps.append(steps)
    return allSteps   

def moveRobotLinear(point, splitnumber=150): 
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, PossibleOmegaList
    if len(point)==5 or len(point)==7:
        
        startPointX, startPointY, startPointZ, startPointOmega = allStepsToXYZOmega()
        
        if point[4]=="s" or point[4]=="S":
            stepsA = point[0]
            stepsB = point[1]
            stepsC = point[2]
            stepsD = point[3]
            endPointX, endPointY, endPointZ, endPointOmega = allStepsToXYZOmega()
            
        if point[4]=="d" or point[4]=="D":
            alpha = point[0]
            beta = point[1]
            gamma = point[2]
            delta = point[3]
            stepsA = degreesToStepsA(alpha)
            stepsB = degreesToStepsB(beta)
            stepsC = degreesToStepsC(gamma)
            stepsD = degreesToStepsD(delta)
            endPointX, endPointY, endPointZ, endPointOmega = allStepsToXYZOmega()
            
        if point[4]=="r" or point[4]=="R":
            alpha = point[0]
            beta = point[1]
            gamma = point[2]
            delta = point[3]
            stepsA = degreesToStepsA(math.degrees(alpha))
            stepsB = degreesToStepsB(math.degrees(beta))
            stepsC = degreesToStepsC(math.degrees(gamma))
            stepsD = degreesToStepsD(math.degrees(delta))
            endPointX, endPointY, endPointZ, endPointOmega = allStepsToXYZOmega()
            
        if point[4]=="cr" or point[4]=="CR" or point[4]=="rc" or point[4]=="RC":
           x = point[0]
           y = point[1]
           z = point[2]
           omega = point[3]
           alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
           stepsA = degreesToStepsA(math.degrees(alpha))
           stepsB = degreesToStepsB(math.degrees(beta))
           stepsC = degreesToStepsC(math.degrees(gamma))
           stepsD = degreesToStepsD(math.degrees(delta))
           endPointX, endPointY, endPointZ, endPointOmega = allStepsToXYZOmega()
           
        if point[4]=="cd" or point[4]=="CD" or point[4]=="dc" or point[4]=="DC":
            x = point[0]
            y = point[1]
            z = point[2]
            omega = point[3]
            omega = math.radians(omega)
            alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
            stepsA = degreesToStepsA(math.degrees(alpha))
            stepsB = degreesToStepsB(math.degrees(beta))
            stepsC = degreesToStepsC(math.degrees(gamma))
            stepsD = degreesToStepsD(math.degrees(delta))
            endPointX, endPointY, endPointZ, endPointOmega = allStepsToXYZOmega()
        
        if point[4]!="cd" and point[4]!="CD" and point[4]!="dc" and point[4]!="DC" and point[4]!="cr" and point[4]!="CR" and point[4]!="rc" and point[4]!="RC" and point[4]!="R" and point[4]!="r" and point[4]!="d" and point[4]!="D" and point[4]!="s" and point[4]!="S":
           print("Nesprávný formát dat zadané pozice.")
           print("Program předčasně ukončen.")
           exit()
           
    else:
      print("Nesprávný formát dat zadané pozice.")
      print("Program předčasně ukončen.")
      exit()     
    
    startPoint = [startPointX, startPointY, startPointZ]
    endPoint = [endPointX, endPointY, endPointZ]
    allSteps = createLineListSteps(startPoint, endPoint, splitnumber, startPointOmega)
    for i in allSteps:
            stepsA = i[0]
            stepsB = i[1]
            stepsC = i[2]
            stepsD = i[3]
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    print("Robot se nachází v požadované pozici.")