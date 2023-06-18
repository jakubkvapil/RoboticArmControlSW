# -*- coding: utf-8 -*-
from tkinter import *
import numpy as np
import time, math
import serial
from tkinter.filedialog import asksaveasfile
from tkinter.messagebox import askokcancel, showinfo, showerror, askyesno

window = Tk()
window.title("Ovládání robotického ramene")
window.resizable(width=False, height=False)
window.eval('tk::PlaceWindow . center')

gripperAngle = int(180)
wristAngle = int(60)
pyCommand = int(1)
stepsA = int(0)
stepsB = int(0)
stepsC = int(0)
stepsD = int(0)
delaySteppers = int(20)
delayServos = int(30)
addSteps = int(20)

limitStepsAMin = int(0);
limitStepsBMin = int(0);
limitStepsCMin = int(0);
limitStepsDMin = int(0);
limitStepsAMax = int(2900);
limitStepsBMax = int(780);
limitStepsCMax = int(1230);
limitStepsDMax = int(1540);

ratioA = (80/16)*(64/20)
ratioB = (92/16)*(100/32)
ratioC = (64/16)*(64/20)
ratioD = (64/16)*(64/20)
stepsPerRevolution = 200
degreeOffsetB = 77.97913
degreeOffsetA = 198
degreeOffsetC = 90
degreeOffsetD = 75.9375
pointIndex = 0

stepManipulatorStep = int(10)
stepManipulatorAngle = float(0.2)
stepManipulatorDist = float(1)
stepServoAngleTurn = float(1)
stepServoAngleSqueeze = float(1)

pointsRecorded = ""

PossibleOmegaList = []
RobotGeoAOffset = 28.5
RobotGeoA = 155
RobotGeoB = 153.375
RobotGeoC = 154.126

toolType = 1
RobotGeoD2 = 238.631 #nastroj 2
RobotGeoD1 = 102.82+70 #nastroj 1
RobotGeoD1 = 102.82+70+78
RobotGeoD = RobotGeoD1

limitSqueezeTool1Min = int(88);
limitSqueezeTool1Max = int(180);
limitSqueezeTool2Min = int(88);
limitSqueezeTool2Max = int(180);
limitTurnTool1Min = int(0);
limitTurnTool1Max = int(180);
limitTurnTool2Min = int(0);
limitTurnTool2Max = int(180);

def clickButtonReadSensor():
    pyCommand = int(8)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    data = serialcom.readline().decode('ascii')
    data = float(data)
    labelInfo.configure(text = "Data ze senzoru přečtena. Zvážená hmotnost je: " + str(data) + " g")

def inverseKinematicsEngine(x, y, z, omega):
    R=math.sqrt(x**2 + y**2)
    R3=RobotGeoAOffset
    R2=RobotGeoD * math.cos(omega)
    R1=R-R2-R3
    z3=RobotGeoD * math.sin(omega)
    z1=z-z3-RobotGeoA
    e=math.sqrt(R1**2 + z1**2)
    beta1=math.asin(z1/e)
    print("Vyprintena beta1 " + str(beta1))
    beta2=math.acos((e**2 +RobotGeoB**2 - RobotGeoC**2)/(2*e*RobotGeoB))
    beta=beta1+beta2
    gamma=math.acos((RobotGeoB**2 +RobotGeoC**2 - e**2)/(2*(RobotGeoB)*(RobotGeoC)))
    delta1=math.radians(90)
    delta2=math.radians(180)-math.radians(90)-beta1
    delta3=math.radians(180)-beta2-gamma
    delta=delta1+delta2+delta3+omega
    kappa = math.atan2(y, x)
    kappa = math.degrees(kappa)
    
    #Výpočet pro 1. kvadrant
    if (kappa >= 0) and (kappa <= 90):
        alpha = -(90-kappa)
        
    #Výpočet pro 2. kvadrant    
    if kappa > 90 and kappa <= 180:
        alpha = (kappa-90)
    
    #Výpočet pro 3. kvadrant
    if (kappa <= -90 and kappa >= -180):
        alpha = 180-(-kappa-90)
    
    #Výpočet pro 4. kvadrant        
    if (kappa < 0 and kappa >= -90):
        alpha = kappa-90
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
          print("Nelze vypočítat hodnoty úhlů se zadaným omega. Vypočítané hodnoty jsou mimo rozsah.")
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
              return 10000, 10000, 10000, 10000;

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
        
    if (math.degrees(alpha) < 0) and (math.degrees(alpha) >= -180):
        kappa = math.radians(90)-(-alpha)
    
    if (math.degrees(alpha) >= 0) and (math.degrees(alpha) <= 180):
        kappa = alpha+math.radians(90)
  
    y=R*math.sin(kappa)
    x=R*math.cos(kappa)

    return x, y, z, omega;  

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

def waitForArduino():
    while True:
       c = serialcom.readline()
       if c==bytes(b'y\n'):
          break
       if c==bytes(b'n\n'):
          print("Chyba: ")
          print(serialcom.readline().decode('ascii'))
          break

def sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos):
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
                updateLabels()
                labelInfo.configure(text = "Pohyb vykonán")
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
  
def updateLabels():
            global controlMode, stepsA, stepsB, stepsC, stepsD, stepManipulatorStep, stepManipulatorAngle, stepManipulatorDist, stepServoAngle
            
            buttonToolStep.configure(text ="Krok  stisku: " + str(stepServoAngleSqueeze) + " °  " + " natočení: " + str(stepServoAngleTurn) + " °"   )
            if (controlMode == "ABCDAngles"):
                degB = round(stepsToDegreesB(stepsB),1)
                degA = round(stepsToDegreesA(stepsA),1)
                degC = round(stepsToDegreesC(stepsC),1)
                degD = round(stepsToDegreesD(stepsD),1)
                
                labelAXValue.configure(text = str(degA) + " °")
                labelBYValue.configure(text = str(degB) + " °")
                labelCZValue.configure(text = str(degC) + " °")
                labelDValue.configure(text = str(degD) + " °")
                labelManipulator.configure(text = "Manipulátor (režim: úhly)")
                
                buttonManipulatorStep.configure(text ="Krok: " + str(stepManipulatorAngle) + " °")
            
            if (controlMode == "ABCDSteps"):                
                labelAXValue.configure(text = str(stepsA))
                labelBYValue.configure(text = str(stepsB))
                labelCZValue.configure(text = str(stepsC))
                labelDValue.configure(text = str(stepsD))
                labelManipulator.configure(text = "Manipulátor (režim: kroky)")
                
                buttonManipulatorStep.configure(text ="Krok: " + str(stepManipulatorStep) + " kroků")
                
                
            if (controlMode == "XYZΩ"):      
                
                degB = stepsToDegreesB(stepsB)
                degA = stepsToDegreesA(stepsA)
                degC = stepsToDegreesC(stepsC)
                degD = stepsToDegreesD(stepsD)
                
                alpha = math.radians(degA)               
                beta = math.radians(degB)
                gamma = math.radians(degC)
                delta = math.radians(degD)
                   
                x, y, z, omega = forwardKinematics(alpha, beta, gamma, delta)
                round(math.degrees(omega),1)

                labelAXValue.configure(text = str(round(x,1)) + " mm")
                labelBYValue.configure(text = str(round(y,1)) + " mm")
                labelCZValue.configure(text = str(round(z,1)) + " mm")
                labelDValue.configure(text = str(round(math.degrees(omega),1)) + " °")
                labelManipulator.configure(text = "Manipulátor (režim: XYZΩ)")

                buttonManipulatorStep.configure(text ="Krok: " + str(stepManipulatorDist) + " mm")
                
            labelGripperCtrlVal.configure(text = str(gripperAngle) + " °")
            labelGripperTurnVal.configure(text = str(wristAngle) + " °")
 
def clickButtonChooseTool():
    global RobotGeoD, toolType, toolTypeVal, windowChooseTool
    windowChooseTool = Tk() 
    windowChooseTool.eval('tk::PlaceWindow . center')
    windowChooseTool.title('Výběr nástroje')
    windowChooseTool.resizable(width=False, height=False)
    toolTypeVal = StringVar(windowChooseTool)
    
    radioButtonTool1 = Radiobutton(windowChooseTool, text="Nástroj 1", variable=toolTypeVal, value="1")
    radioButtonTool1.grid(row=2, column=1, sticky=EW)
    radioButtonTool2 = Radiobutton(windowChooseTool, text="Nástroj 2", variable=toolTypeVal, value="2")
    radioButtonTool2.grid(row=3, column=1, sticky=EW)

    buttonChooseToolConfirm = Button(windowChooseTool, text="Potvrdit", command=clickButtonChooseToolConfirm)
    buttonChooseToolConfirm.grid(row=12, column=1, sticky=EW)

def clickButtonChooseToolConfirm():
    global RobotGeoD, toolType, toolTypeVal, windowChooseTool
    
    toolType = int(toolTypeVal.get())
    labelInfo.configure(text = "Vybrán nástroj " + str(toolType))
    windowChooseTool.destroy()
    
    if toolType==1:
        RobotGeoD = RobotGeoD1
        
    if toolType==2:
        RobotGeoD = RobotGeoD2   

def clickButtonStartSerial():
    global serialConnected
    if serialConnected == False: 
        global windowStartSerial
        windowStartSerial = Tk() 
        windowStartSerial.eval('tk::PlaceWindow . center')
        windowStartSerial.title('Navázání sériové komunikace')
        windowStartSerial.resizable(width=False, height=False)
    
        global COMVal
        global BaudRateVal
        COMVal = StringVar(windowStartSerial)
        BaudRateVal = StringVar(windowStartSerial)
        radioButtonCOM0 = Radiobutton(windowStartSerial, text="COM0", variable=COMVal, value="COM0")
        radioButtonCOM0.grid(row=2, column=1, sticky=EW)
        radioButtonCOM1 = Radiobutton(windowStartSerial, text="COM1", variable=COMVal, value="COM1")
        radioButtonCOM1.grid(row=3, column=1, sticky=EW)
        radioButtonCOM2 = Radiobutton(windowStartSerial, text="COM2", variable=COMVal, value="COM2")
        radioButtonCOM2.grid(row=4, column=1, sticky=EW)
        radioButtonCOM3 = Radiobutton(windowStartSerial, text="COM3", variable=COMVal, value="COM3")
        radioButtonCOM3.grid(row=5, column=1, sticky=EW) 
        radioButtonCOM4 = Radiobutton(windowStartSerial, text="COM4", variable=COMVal, value="COM4")
        radioButtonCOM4.grid(row=6, column=1, sticky=EW)
        radioButtonCOM5 = Radiobutton(windowStartSerial, text="COM5", variable=COMVal, value="COM5")
        radioButtonCOM5.grid(row=7, column=1, sticky=EW)
        radioButtonCOM6 = Radiobutton(windowStartSerial, text="COM6", variable=COMVal, value="COM6")
        radioButtonCOM6.grid(row=8, column=1, sticky=EW)
        radioButtonCOM7 = Radiobutton(windowStartSerial, text="COM7", variable=COMVal, value="COM7")
        radioButtonCOM7.grid(row=9, column=1, sticky=EW)
        radioButtonCOM8 = Radiobutton(windowStartSerial, text="COM8", variable=COMVal, value="COM8")
        radioButtonCOM8.grid(row=10, column=1, sticky=EW)
        radioButtonCOM9 = Radiobutton(windowStartSerial, text="COM9", variable=COMVal, value="COM9")
        radioButtonCOM9.grid(row=11, column=1, sticky=EW)
    
        radioButtonBR300 = Radiobutton(windowStartSerial, text="300 Bauds", variable=BaudRateVal, value="300")
        radioButtonBR300.grid(row=2, column=2, sticky=EW)
        radioButtonBR600 = Radiobutton(windowStartSerial, text="600 Bauds", variable=BaudRateVal, value="600")
        radioButtonBR600.grid(row=3, column=2, sticky=EW)
        radioButtonBR1200 = Radiobutton(windowStartSerial, text="1 200 Bauds", variable=BaudRateVal, value="1200")
        radioButtonBR1200.grid(row=4, column=2, sticky=EW)
        radioButtonBR2400 = Radiobutton(windowStartSerial, text="2 400 Bauds", variable=BaudRateVal, value="2400")
        radioButtonBR2400.grid(row=5, column=2, sticky=EW)
        radioButtonBR4800 = Radiobutton(windowStartSerial, text="4 800 Bauds", variable=BaudRateVal, value="4800")
        radioButtonBR4800.grid(row=6, column=2, sticky=EW)
        radioButtonBR9600 = Radiobutton(windowStartSerial, text="9 600 Bauds", variable=BaudRateVal, value="9600")
        radioButtonBR9600.grid(row=7, column=2, sticky=EW)
        radioButtonBR28800 = Radiobutton(windowStartSerial, text="28 800 Bauds", variable=BaudRateVal, value="28800")
        radioButtonBR28800.grid(row=8, column=2, sticky=EW)
        radioButtonBR38400 = Radiobutton(windowStartSerial, text="38 400 Bauds", variable=BaudRateVal, value="38400")
        radioButtonBR38400.grid(row=9, column=2, sticky=EW)
        radioButtonBR57600 = Radiobutton(windowStartSerial, text="57 600 Bauds", variable=BaudRateVal, value="57600")
        radioButtonBR57600.grid(row=10, column=2, sticky=EW)
        radioButtonBR115200 = Radiobutton(windowStartSerial, text="115 200 Bauds", variable=BaudRateVal, value="115200")
        radioButtonBR115200.grid(row=11, column=2, sticky=EW)
        
        buttonChooseCOMBR = Button(windowStartSerial, text="Vybrat", command=clickButtonChooseCOMBR)
        buttonChooseCOMBR.grid(row=12, column=1, columnspan=2, sticky=EW)
    else:
        serialcom.close()
        serialConnected = False
        labelInfoLeft.configure(text = "Port úspěšně odpojen")
        labelInfo.configure(text = "Arduino odpojeno")
        buttonStartSerial.configure(text="Navázat komunikaci")
        deactivateButtons()
      
def deactivateButtons():
    buttonABCDAngles.configure(state="disabled")
    buttonAXMinus.configure(state="disabled")
    buttonAXPlus.configure(state="disabled")
    buttonBYMinus.configure(state="disabled")
    buttonBYPlus.configure(state="disabled")
    buttonCZMinus.configure(state="disabled")
    buttonCZPlus.configure(state="disabled")
    buttonCalibration.configure(state="disabled")
    buttonDMinus.configure(state="disabled")
    buttonDPlus.configure(state="disabled")
    buttonGripperCtrlMinus.configure(state="disabled")
    buttonGripperCtrlPlus.configure(state="disabled")
    buttonGripperTurnPlus.configure(state="disabled")
    buttonGripperTurnMinus.configure(state="disabled")
    buttonMoveManipulator.configure(state="disabled")
    buttonReset.configure(state="disabled")
    buttonSavePos.configure(state="disabled")
    buttonCalibrationSingle.configure(state="disabled")
    buttonReadSensor.configure(state="disabled")
    buttonHomePosSteppers.configure(state="disabled")
    buttonHomePosTool.configure(state="disabled")
    buttonChooseTool.configure(state="disabled")
    buttonSpeedSteppers.configure(state="disabled")
    buttonXYZOmega.configure(state="disabled")
    buttonABCDAngles.configure(state="disabled")
    buttonABCDSteps.configure(state="disabled")
    buttonToolStep.configure(state="disabled")
    buttonManipulatorStep.configure(state="disabled")
    buttonMoveTool.configure(state="disabled")
    
def activateButtons():
    buttonABCDAngles.configure(state="normal")
    buttonAXMinus.configure(state="normal")
    buttonAXPlus.configure(state="normal")
    buttonBYMinus.configure(state="normal")
    buttonBYPlus.configure(state="normal")
    buttonCZMinus.configure(state="normal")
    buttonCZPlus.configure(state="normal")
    buttonCalibration.configure(state="normal")
    buttonDMinus.configure(state="normal")
    buttonDPlus.configure(state="normal")
    buttonGripperCtrlMinus.configure(state="normal")
    buttonGripperCtrlPlus.configure(state="normal")
    buttonGripperTurnPlus.configure(state="normal")
    buttonGripperTurnMinus.configure(state="normal")
    buttonMoveManipulator.configure(state="normal")
    buttonReset.configure(state="normal")
    buttonSavePos.configure(state="normal")
    buttonCalibrationSingle.configure(state="normal")
    buttonReadSensor.configure(state="normal")
    buttonHomePosSteppers.configure(state="normal")
    buttonHomePosTool.configure(state="normal")
    buttonChooseTool.configure(state="normal")
    buttonSpeedSteppers.configure(state="normal")
    buttonXYZOmega.configure(state="normal")
    buttonABCDAngles.configure(state="normal")
    buttonABCDSteps.configure(state="normal")
    buttonToolStep.configure(state="normal")
    buttonManipulatorStep.configure(state="normal")
    buttonMoveTool.configure(state="normal")
    
def clickButtonHomePosTool():    
    global wristAngle, gripperAngle
    if toolType==1:
        pyCommand = int(7)
        gripperAngle = int(180) 
        wristAngle = int(60)
        delayServos = int(20)
        sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    if toolType==2:
        pyCommand = int(7)
        gripperAngle = int(180) 
        wristAngle = int(60)
        delayServos = int(20)  
        sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        
def clickButtonChooseCOMBR():
    global COM
    global BaudRate
    COM = COMVal.get()
    BaudRate = BaudRateVal.get()
    labelInfoLeft.configure(text=COM+"   "+BaudRate+" Bauds")
    windowStartSerial.destroy()
    global serialcom
    
    try:    
        serialcom = serial.Serial(COM,int(BaudRate))
        serialcom.timeout = 0.1 
    except:
        labelInfoLeft.configure(text="Spojení se nezdařilo")
        showerror(title="Chyba", message="Připojení se nezdařilo. Ujistěte se, že je Arduino připojeno ke správnému COM portu!")
        return
    else:
        showinfo(title="Potvrzení", message="Stistkněte zelené tlačítko. Poté nezapoměňte provést kalibraci!")
        while True: 
            greenBttnConfirm= serialcom.readline().decode('ascii')
            if  (greenBttnConfirm=="Arduino pripraveno\n"):
                break
        global serialConnected
        serialConnected = True
    buttonStartSerial.configure(text="Zrušit spojení")
    labelInfo.configure(text = "Arduino připraveno")
    activateButtons()
    
def clickButtonCalib():
    global stepsA, stepsB, stepsC, stepsD
    pyCommand = int(1)
    sendDataToRobot(int(1), int(1), int(1), int(1), int(1), int(1), pyCommand, int(1), int(1))
    stepsA = int(0)
    stepsB = int(0)
    stepsC = int(0)
    stepsD = int(0)
    updateLabels()
    
def clickButtonCalibSingle():    
    windowCalibSingle = Tk()
    windowCalibSingle.title('')
    windowCalibSingle.eval('tk::PlaceWindow . center')
    windowCalibSingle.resizable(width=False, height=False)
 
    buttonCalibSingleA = Button(windowCalibSingle, text="Kalibrovat osu A", width=20, command=clickButtonCalibSingleA)
    buttonCalibSingleA.grid(row=2, column=2, sticky=EW)
    
    buttonCalibSingleB = Button(windowCalibSingle, text="Kalibrovat osu B", width=20, command=clickButtonCalibSingleB)
    buttonCalibSingleB.grid(row=3, column=2, sticky=EW)
    
    buttonCalibSingleC = Button(windowCalibSingle, text="Kalibrovat osu C", width=20, command=clickButtonCalibSingleC)
    buttonCalibSingleC.grid(row=4, column=2, sticky=EW)
    
    buttonCalibSingleD = Button(windowCalibSingle, text="Kalibrovat osu D", width=20, command=clickButtonCalibSingleD)
    buttonCalibSingleD.grid(row=5, column=2, sticky=EW)
    
def clickButtonCalibSingleA():
    global stepsA
    pyCommand = int(2)
    sendDataToRobot(int(1), int(1), int(1), int(1), int(1), int(1), pyCommand, int(1), int(1))
    stepsA = int(0)
    labelInfo.configure(text = "Byla provedena kalibrace osy A") 
    updateLabels()

def clickButtonCalibSingleB():
    global stepsB
    pyCommand = int(3)
    sendDataToRobot(int(1), int(1), int(1), int(1), int(1), int(1), pyCommand, int(1), int(1))
    stepsB = int(0)
    labelInfo.configure(text = "Byla provedena kalibrace osy B") 
    updateLabels()

def clickButtonCalibSingleC():
    global stepsC
    pyCommand = int(4)
    sendDataToRobot(int(1), int(1), int(1), int(1), int(1), int(1), pyCommand, int(1), int(1))
    stepsC = int(0)
    labelInfo.configure(text = "Byla provedena kalibrace osy C") 
    updateLabels()

def clickButtonCalibSingleD():
    global stepsD
    pyCommand = int(5)
    sendDataToRobot(int(1), int(1), int(1), int(1), int(1), int(1), pyCommand, int(1), int(1))
    stepsD = int(0)
    labelInfo.configure(text = "Byla provedena kalibrace osy D") 
    updateLabels()    
    
def clickButtonReset():
    pyCommand = int(199)
    sendDataToRobot(int(1), int(1), int(1), int(1), int(1), int(1), pyCommand, int(1), int(1))
    labelInfo.configure(text = "Arduino bylo resetováno! Pro jeho aktivaci nezapomeňte stisknout zelené tlačítko!")   
    updateLabels()   

def clickButtonSavePos():
    global controlMode, pointIndex, stepsA, stepsB, stepsC, stepsD, pointsRecorded, gripperAngle, wristAngle
    answer = askyesno(title='Jméno uložené pozice',
                    message='Chcete pojmenovat uloženou pozici automaticky?')
    if answer:
        if (controlMode == "ABCDAngles"):
            degA = stepsToDegreesA(stepsA)                        
            degB = stepsToDegreesB(stepsB)
            degC = stepsToDegreesC(stepsC)
            degD = stepsToDegreesD(stepsD)
            newPoint = "point" + str(pointIndex) + " = [" + str(degA) + ", " + str(degB) + ", " + str(degC) + ", " + str(degD) + ", " + "'d', " + str(wristAngle) + ", " + str(gripperAngle) + "]\n"
         
        if (controlMode == "ABCDSteps"):            
            newPoint = "point" + str(pointIndex) + " = [" + str(stepsA) + ", " + str(stepsB) + ", " + str(stepsC) + ", " + str(stepsC) + ", " + "'s', " + str(wristAngle) + ", " + str(gripperAngle) + "]\n"
    
        if (controlMode == "XYZΩ"):
            x, y, z, omega = allStepsToXYZOmega()
            newPoint = "point" + str(pointIndex) + " = [" + str(x) + ", " + str(y) + ", " + str(z) + ", " + str(omega) + ", " + "'cr', " + str(wristAngle) + ", " + str(gripperAngle) + "]\n"
    
        pointIndex = pointIndex + 1  
        pointsRecorded = pointsRecorded + newPoint
        labelInfo.configure(text = "Pozice byla uložena")        
    else:
        global strVarPositionName, entryPositionName, windowPositionName        
        windowPositionName = Tk()
        windowPositionName.title('')
        windowPositionName.eval('tk::PlaceWindow . center')
        windowPositionName.resizable(width=False, height=False)
        
        labelPositionName = Label(windowPositionName, text="Jméno uložené pozice:")
        labelPositionName.grid(row=2, column=1, sticky=EW)
        
        strVarPositionName = StringVar(windowPositionName, value="Point")
        
        entryPositionName = Entry(windowPositionName, width=10, textvariable=strVarPositionName)
        entryPositionName.grid(row=2, column=2)    
        
        buttonPositionName = Button(windowPositionName, text="Potvrdit", width=20, command=clickButtonPositionName)
        buttonPositionName.grid(row=5, column=2, sticky=EW)
        
def clickButtonPositionName():
    global clickButtonPositionName, windowPositionName, entryPositionName, controlMode, stepsA, stepsB, stepsC, stepsD, pointsRecorded
    PositionName = entryPositionName.get()
    window.lift()
    windowPositionName.destroy()
    print(PositionName)
    if (controlMode == "ABCDAngles"):
        degA = stepsToDegreesA(stepsA)                        
        degB = stepsToDegreesB(stepsB)
        degC = stepsToDegreesC(stepsC)
        degD = stepsToDegreesD(stepsD)
        newPoint = str(PositionName) + " = [" + str(degA) + ", " + str(degB) + ", " + str(degC) + ", " + str(degD) + ", " + "'d', " + str(wristAngle) + ", " + str(gripperAngle) + "]\n"
     
    if (controlMode == "ABCDSteps"):            
        newPoint = str(PositionName) + " = [" + str(stepsA) + ", " + str(stepsB) + ", " + str(stepsC) + ", " + str(stepsC) + ", " + "'s', " + str(wristAngle) + ", " + str(gripperAngle) + "]\n"

    if (controlMode == "XYZΩ"):
        x, y, z, omega = allStepsToXYZOmega()
        newPoint = str(PositionName) + " = [" + str(x) + ", " + str(y) + ", " + str(z) + ", " + str(omega) + ", " + "'cr', " + str(wristAngle) + ", " + str(gripperAngle) + "]\n"

    pointsRecorded = pointsRecorded + newPoint
    labelInfo.configure(text = "Pozice byla uložena")      
     
def clickButtonSavedPos():
    windowPoints = Tk()
    windowPoints.eval('tk::PlaceWindow . center')
    windowPoints.title('Uložené pozice')
    windowPoints.resizable(width=False, height=False)
    textPoints = Text(windowPoints,height=12,width=40)
    textPoints.insert('end', pointsRecorded)
    textPoints.pack(expand=True)
    
def clickButtonDelSavedPos():
    
    answer = askokcancel(title="Pozor", message="Opravdu si přejete smazat uložené pozice?")
    if answer:
        global pointsRecorded, pointIndex
        pointsRecorded = ""
        labelInfo.configure(text = "Uložené pozice byly smazány")
        pointIndex = 0
    
def clickButtonExportPos():
    files = [('Textový dokument', '*.txt')]
    file = asksaveasfile(filetypes = files, defaultextension = files)
    file.write(pointsRecorded)
    labelInfo.configure(text = "Soubor byl uložen")

def clickButtonABCDAngles():
    global controlMode
    controlMode = "ABCDAngles"
    labelAX.configure(text = "Osa A")
    labelBY.configure(text = "Osa B")
    labelCZ.configure(text = "Osa C")
    labelD.configure(text = "Osa D")
    buttonDPlus.configure(state="normal")
    buttonDMinus.configure(state="normal")
    labelInfo.configure(text = "Režim ovládání ABCD - úhly nastaven")
    updateLabels()

def clickButtonABCDSteps():
    global controlMode
    controlMode = "ABCDSteps"
    labelAX.configure(text = "Osa A")
    labelBY.configure(text = "Osa B")
    labelCZ.configure(text = "Osa C")
    labelD.configure(text = "Osa D")
    buttonDPlus.configure(state="normal")
    buttonDMinus.configure(state="normal")
    labelInfo.configure(text = "Režim ovládání ABCD - kroky nastaven")
    updateLabels()
    
def clickButtonXYZOmega():
    global controlMode
    controlMode = "XYZΩ"
    labelAX.configure(text = "Osa X")
    labelBY.configure(text = "Osa Y")
    labelCZ.configure(text = "Osa Z")
    labelD.configure(text = "Ω")
    labelInfo.configure(text = "Režim ovládání XYZΩ nastaven")
    buttonDPlus.configure(state="disabled")
    buttonDMinus.configure(state="disabled")
    updateLabels()       
    
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
    
def clickButtonAXPlus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps, stepManipulatorDist, PossibleOmegaList    
    if (controlMode == "ABCDAngles") or (controlMode == "ABCDSteps"):
        
        if (controlMode == "ABCDAngles"):     
            addSteps = degreesToSteps(stepManipulatorAngle, ratioA)
        if (controlMode == "ABCDSteps"):    
            addSteps = stepManipulatorStep
        if ((((stepsA+addSteps)>=limitStepsAMin) and ((stepsA+addSteps)<=limitStepsAMax)) and ((stepsB>=limitStepsBMin) and (stepsB<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
            stepsA = stepsA + addSteps
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:        
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
            
    if (controlMode == "XYZΩ"):
        x, y, z, omega = allStepsToXYZOmega()
        x = x + stepManipulatorDist
              
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        newStepsA = degreesToStepsA(math.degrees(alpha))
        newStepsB = degreesToStepsB(math.degrees(beta))
        newStepsC = degreesToStepsC(math.degrees(gamma))
        newStepsD = degreesToStepsD(math.degrees(delta))


        if ((((newStepsA)>=limitStepsAMin) and ((newStepsA)<=limitStepsAMax)) and ((newStepsB>=limitStepsBMin) and (newStepsB<=limitStepsBMax)) and ((newStepsC>=limitStepsCMin) and (newStepsC<=limitStepsCMax)) and ((newStepsD>=limitStepsDMin) and (newStepsD<=limitStepsDMax))):
            pyCommand = int(6)
            stepsA = newStepsA
            stepsB = newStepsB
            stepsC = newStepsC
            stepsD = newStepsD
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
    
def clickButtonAXMinus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps, stepManipulatorDist, PossibleOmegaList    
    if (controlMode == "ABCDAngles") or (controlMode == "ABCDSteps"):    
        if (controlMode == "ABCDAngles"):     
            addSteps = degreesToSteps(stepManipulatorAngle, ratioA)
        if (controlMode == "ABCDSteps"):
            addSteps = stepManipulatorStep
        if ((((stepsA-addSteps)>=limitStepsAMin) and ((stepsA-addSteps)<=limitStepsAMax)) and ((stepsB>=limitStepsBMin) and (stepsB<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
            stepsA = stepsA - addSteps
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:        
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
            
    if (controlMode == "XYZΩ"):
        x, y, z, omega = allStepsToXYZOmega()
        x = x - stepManipulatorDist
              
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        newStepsA = degreesToStepsA(math.degrees(alpha))
        newStepsB = degreesToStepsB(math.degrees(beta))
        newStepsC = degreesToStepsC(math.degrees(gamma))
        newStepsD = degreesToStepsD(math.degrees(delta))

        if ((((newStepsA)>=limitStepsAMin) and ((newStepsA)<=limitStepsAMax)) and ((newStepsB>=limitStepsBMin) and (newStepsB<=limitStepsBMax)) and ((newStepsC>=limitStepsCMin) and (newStepsC<=limitStepsCMax)) and ((newStepsD>=limitStepsDMin) and (newStepsD<=limitStepsDMax))):
            pyCommand = int(6)
            stepsA = newStepsA
            stepsB = newStepsB
            stepsC = newStepsC
            stepsD = newStepsD
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
            
def clickButtonBYPlus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps, stepManipulatorDist, PossibleOmegaList 
    if (controlMode == "ABCDAngles") or (controlMode == "ABCDSteps"): 
        if (controlMode == "ABCDAngles"):     
            addSteps = degreesToSteps(stepManipulatorAngle, ratioB)
            if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and (((stepsB-addSteps)>=limitStepsBMin) and ((stepsB-addSteps)<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
                stepsB = stepsB - addSteps
                pyCommand = int(6)
                sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
                
            else:        
                showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
                labelInfo.configure(text = "Pohyb nebyl vykonán")                
  
        if (controlMode == "ABCDSteps"): 
            addSteps = stepManipulatorStep
            if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and (((stepsB+addSteps)>=limitStepsBMin) and ((stepsB+addSteps)<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
                stepsB = stepsB + addSteps
                pyCommand = int(6)
                sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            else:        
                showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
                labelInfo.configure(text = "Pohyb nebyl vykonán")
        
    if (controlMode == "XYZΩ"):
        x, y, z, omega = allStepsToXYZOmega()
        y = y + stepManipulatorDist
              
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        newStepsA = degreesToStepsA(math.degrees(alpha))
        newStepsB = degreesToStepsB(math.degrees(beta))
        newStepsC = degreesToStepsC(math.degrees(gamma))
        newStepsD = degreesToStepsD(math.degrees(delta))


        if ((((newStepsA)>=limitStepsAMin) and ((newStepsA)<=limitStepsAMax)) and ((newStepsB>=limitStepsBMin) and (newStepsB<=limitStepsBMax)) and ((newStepsC>=limitStepsCMin) and (newStepsC<=limitStepsCMax)) and ((newStepsD>=limitStepsDMin) and (newStepsD<=limitStepsDMax))):
            pyCommand = int(6)
            stepsA = newStepsA
            stepsB = newStepsB
            stepsC = newStepsC
            stepsD = newStepsD
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
    
def clickButtonBYMinus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps, stepManipulatorDist, PossibleOmegaList 
    if (controlMode == "ABCDAngles") or (controlMode == "ABCDSteps"): 
        if (controlMode == "ABCDAngles"):     
            addSteps = degreesToSteps(stepManipulatorAngle, ratioB)
            if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and (((stepsB+addSteps)>=limitStepsBMin) and ((stepsB+addSteps)<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
                stepsB = stepsB + addSteps
                pyCommand = int(6)
                sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            else: 
                showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
                labelInfo.configure(text = "Pohyb nebyl vykonán")
        if (controlMode == "ABCDSteps"): 
            addSteps = stepManipulatorStep
            if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and (((stepsB-addSteps)>=limitStepsBMin) and ((stepsB-addSteps)<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
                stepsB = stepsB - addSteps
                pyCommand = int(6)
                sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            else:        
                showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
                labelInfo.configure(text = "Pohyb nebyl vykonán")
            
    if (controlMode == "XYZΩ"):
        x, y, z, omega = allStepsToXYZOmega()
        y = y - stepManipulatorDist
              
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        newStepsA = degreesToStepsA(math.degrees(alpha))
        newStepsB = degreesToStepsB(math.degrees(beta))
        newStepsC = degreesToStepsC(math.degrees(gamma))
        newStepsD = degreesToStepsD(math.degrees(delta))


        if ((((newStepsA)>=limitStepsAMin) and ((newStepsA)<=limitStepsAMax)) and ((newStepsB>=limitStepsBMin) and (newStepsB<=limitStepsBMax)) and ((newStepsC>=limitStepsCMin) and (newStepsC<=limitStepsCMax)) and ((newStepsD>=limitStepsDMin) and (newStepsD<=limitStepsDMax))):
            pyCommand = int(6)
            stepsA = newStepsA
            stepsB = newStepsB
            stepsC = newStepsC
            stepsD = newStepsD
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
    
def clickButtonCZPlus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps, stepManipulatorDist, PossibleOmegaList 
    if (controlMode == "ABCDAngles") or (controlMode == "ABCDSteps"): 
        if (controlMode == "ABCDAngles"):     
            addSteps = degreesToSteps(stepManipulatorAngle, ratioC)
        if (controlMode == "ABCDSteps"): 
            addSteps = stepManipulatorStep
        if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and ((stepsB>=limitStepsBMin) and (stepsB<=limitStepsBMax)) and (((stepsC+addSteps)>=limitStepsCMin) and ((stepsC+addSteps)<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
            stepsC = stepsC + addSteps
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:        
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")
  
    if (controlMode == "XYZΩ"):
        x, y, z, omega = allStepsToXYZOmega()
        z = z + stepManipulatorDist
              
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        newStepsA = degreesToStepsA(math.degrees(alpha))
        newStepsB = degreesToStepsB(math.degrees(beta))
        newStepsC = degreesToStepsC(math.degrees(gamma))
        newStepsD = degreesToStepsD(math.degrees(delta))


        if ((((newStepsA)>=limitStepsAMin) and ((newStepsA)<=limitStepsAMax)) and ((newStepsB>=limitStepsBMin) and (newStepsB<=limitStepsBMax)) and ((newStepsC>=limitStepsCMin) and (newStepsC<=limitStepsCMax)) and ((newStepsD>=limitStepsDMin) and (newStepsD<=limitStepsDMax))):
            pyCommand = int(6)
            stepsA = newStepsA
            stepsB = newStepsB
            stepsC = newStepsC
            stepsD = newStepsD
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")        
   
def clickButtonCZMinus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps, stepManipulatorDist, PossibleOmegaList 
    if (controlMode == "ABCDAngles") or (controlMode == "ABCDSteps"): 
        if (controlMode == "ABCDAngles"):     
            addSteps = degreesToSteps(stepManipulatorAngle, ratioC)
        if (controlMode == "ABCDSteps"): 
            addSteps = stepManipulatorStep
        if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and ((stepsB>=limitStepsBMin) and (stepsB<=limitStepsBMax)) and (((stepsC-addSteps)>=limitStepsCMin) and ((stepsC-addSteps)<=limitStepsCMax)) and ((stepsD>=limitStepsDMin) and (stepsD<=limitStepsDMax))):
            stepsC = stepsC - addSteps
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:        
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")

    if (controlMode == "XYZΩ"):
        x, y, z, omega = allStepsToXYZOmega()
        z = z - stepManipulatorDist
              
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        newStepsA = degreesToStepsA(math.degrees(alpha))
        newStepsB = degreesToStepsB(math.degrees(beta))
        newStepsC = degreesToStepsC(math.degrees(gamma))
        newStepsD = degreesToStepsD(math.degrees(delta))

        if ((((newStepsA)>=limitStepsAMin) and ((newStepsA)<=limitStepsAMax)) and ((newStepsB>=limitStepsBMin) and (newStepsB<=limitStepsBMax)) and ((newStepsC>=limitStepsCMin) and (newStepsC<=limitStepsCMax)) and ((newStepsD>=limitStepsDMin) and (newStepsD<=limitStepsDMax))):
            pyCommand = int(6)
            stepsA = newStepsA
            stepsB = newStepsB
            stepsC = newStepsC
            stepsD = newStepsD
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")        
  
def clickButtonDPlus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps, stepManipulatorDist, PossibleOmegaList
    if (controlMode == "ABCDAngles") or (controlMode == "ABCDSteps"): 
        if (controlMode == "ABCDAngles"):     
            addSteps = degreesToSteps(stepManipulatorAngle, ratioD)
        if (controlMode == "ABCDSteps"): 
            addSteps = stepManipulatorStep
        if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and ((stepsB>=limitStepsBMin) and (stepsB<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and (((stepsD+addSteps)>=limitStepsDMin) and ((stepsD+addSteps)<=limitStepsDMax))):
            stepsD = stepsD + addSteps
            pyCommand = int(6)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:        
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán") 
            
def clickButtonDMinus():
    global stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos, controlMode, addSteps
    if (controlMode == "ABCDAngles"):     
        addSteps = degreesToSteps(stepManipulatorAngle, ratioD)
    if (controlMode == "ABCDSteps"): 
        addSteps = stepManipulatorStep
    if (((stepsA>=limitStepsAMin) and (stepsA<=limitStepsAMax)) and ((stepsB>=limitStepsBMin) and (stepsB<=limitStepsBMax)) and ((stepsC>=limitStepsCMin) and (stepsC<=limitStepsCMax)) and (((stepsD-addSteps)>=limitStepsDMin) and ((stepsD-addSteps)<=limitStepsDMax))):
        stepsD = stepsD - addSteps
        pyCommand = int(6)
        sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    else:        
        showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
        labelInfo.configure(text = "Pohyb nebyl vykonán")

def clickButtonGripperCtrlPlus():
    
    global limitSqueezeTool1Max, limitSqueezeTool1Min, limitTurnTool1Max, limitTurnTool1Min
    global limitSqueezeTool2Max, limitSqueezeTool2Min, limitTurnTool2Max, limitTurnTool2Min, toolType, gripperAngle, wristAngle, stepServoAngleTurn

    if toolType==1:
        if ((gripperAngle+stepServoAngleTurn>=limitSqueezeTool1Min) and (gripperAngle+stepServoAngleTurn<=limitSqueezeTool1Max)):     
            gripperAngle = round((gripperAngle + stepServoAngleSqueeze))
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")

    if toolType==2:
        if ((gripperAngle+stepServoAngleTurn>=limitSqueezeTool2Min) and (gripperAngle+stepServoAngleTurn<=limitSqueezeTool2Max)):     
            gripperAngle = round(gripperAngle + stepServoAngleTurn)
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")
    labelInfo.configure(text = "Pohyb byl vykonán")
    updateLabels()

def clickButtonGripperCtrlMinus():
    
    global limitSqueezeTool2Max, limitSqueezeTool2Min, toolType, gripperAngle, wristAngle, stepServoAngleSqueeze, limitSqueezeTool1Max, limitSqueezeTool1Min

    if toolType==1:
        if ((gripperAngle-stepServoAngleSqueeze>=limitSqueezeTool1Min) and (gripperAngle-stepServoAngleSqueeze<=limitSqueezeTool1Max)):     
            gripperAngle = round(gripperAngle - stepServoAngleSqueeze)
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")

    if toolType==2:
        if ((gripperAngle-stepServoAngleSqueeze>=limitSqueezeTool2Min) and (gripperAngle-stepServoAngleSqueeze<=limitSqueezeTool2Max)):     
            gripperAngle = round(gripperAngle - stepServoAngleSqueeze)
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")
    labelInfo.configure(text = "Pohyb byl vykonán")
    updateLabels()

def clickButtonGripperTurnPlus():
    global limitTurnTool1Max, limitTurnTool1Min, limitTurnTool2Max, limitTurnTool2Min, toolType, gripperAngle, wristAngle, stepServoAngleTurn

    if toolType==1:
        if ((wristAngle + stepServoAngleTurn>=limitTurnTool1Min) and (wristAngle + stepServoAngleTurn<=limitTurnTool1Max)):     
            wristAngle = round((wristAngle + stepServoAngleTurn))
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")

    if toolType==2:
        if ((wristAngle+stepServoAngleTurn>=limitTurnTool2Min) and (wristAngle+stepServoAngleTurn<=limitTurnTool2Max)):     
            wristAngle = round(wristAngle + stepServoAngleTurn)
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")
    labelInfo.configure(text = "Pohyb byl vykonán")
    updateLabels()

def clickButtonGripperTurnMinus():
    global limitTurnTool1Max, limitTurnTool1Min, limitTurnTool2Max, limitTurnTool2Min, toolType, gripperAngle, wristAngle, stepServoAngleTurn

    if toolType==1:
        if ((wristAngle - stepServoAngleTurn>=limitTurnTool1Min) and (wristAngle - stepServoAngleTurn<=limitTurnTool1Max)):     
            wristAngle = round((wristAngle - stepServoAngleTurn))
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")

    if toolType==2:
        if ((wristAngle - stepServoAngleTurn>=limitTurnTool2Min) and (wristAngle - stepServoAngleTurn<=limitTurnTool2Max)):     
            wristAngle = round(wristAngle - stepServoAngleTurn)
            pyCommand = int(7)
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnota natočení serva nástroje je mimo rozsah!")
    labelInfo.configure(text = "Pohyb byl vykonán")
    updateLabels()
    
def clickButtonMoveManipulator():
    global limitStepsAMax, limitStepsAMin, limitStepsBMax, limitStepsBMin, limitStepsCMax, limitStepsCMin, limitStepsDMax, limitStepsDMin, stepsA, stepsB, stepsC, stepsD, PossibleOmegaList
    AX = entryAX.get()
    BY = entryBY.get()
    CZ = entryCZ.get()
    D = entryD.get()
    
    if (controlMode == "ABCDAngles"):
        
        try:
            AX = float(AX)
            BY = float(BY)
            CZ = float(CZ)
            D = float(D)       
        except:
            showerror(title="Nesprávný vstup", message="Zadejte prosím čísla. Pro zadání desetinného čísla nutno použít desetinnou tečku!")
        else:
            AX = degreesToStepsA(AX)                        
            BY = degreesToStepsB(BY)
            CZ = degreesToStepsC(CZ)
            D = degreesToStepsD(D)
            
            if (((AX>=limitStepsAMin) and (AX<=limitStepsAMax)) and ((BY>=limitStepsBMin) and (BY<=limitStepsBMax)) and ((CZ>=limitStepsCMin) and (CZ<=limitStepsCMax)) and (((D)>=limitStepsDMin) and ((D)<=limitStepsDMax))):     
                stepsA = AX
                stepsB = BY
                stepsC = CZ
                stepsD = D
                pyCommand = int(6)
                sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            else:
                showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            entryAX.delete(0, 'end')
            entryBY.delete(0, 'end')
            entryCZ.delete(0, 'end')
            entryD.delete(0, 'end') 
                    
    if (controlMode == "XYZΩ"):
        x = float(AX)
        y = float(BY)
        z = float(CZ)
        omega = math.radians(float(D))
        entryAX.delete(0, 'end')
        entryBY.delete(0, 'end')
        entryCZ.delete(0, 'end')
        entryD.delete(0, 'end')         
              
        alpha, beta, gamma, delta = inverseKinematics(x, y, z, omega, 0.5)
        newStepsA = degreesToStepsA(math.degrees(alpha))
        newStepsB = degreesToStepsB(math.degrees(beta))
        newStepsC = degreesToStepsC(math.degrees(gamma))
        newStepsD = degreesToStepsD(math.degrees(delta))

        if ((((newStepsA)>=limitStepsAMin) and ((newStepsA)<=limitStepsAMax)) and ((newStepsB>=limitStepsBMin) and (newStepsB<=limitStepsBMax)) and ((newStepsC>=limitStepsCMin) and (newStepsC<=limitStepsCMax)) and ((newStepsD>=limitStepsDMin) and (newStepsD<=limitStepsDMax))):
            pyCommand = int(6)
            stepsA = newStepsA
            stepsB = newStepsB
            stepsC = newStepsC
            stepsD = newStepsD
            sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
        else:
            showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
            labelInfo.configure(text = "Pohyb nebyl vykonán")

    if (controlMode == "ABCDSteps"):
            try:      
                AX = int(AX)
                BY = int(BY)
                CZ = int(CZ)
                D = int(D)
            except:
                showerror(title="Nesprávný vstup", message="Zadejte prosím čísla. Pro zadání desetinného čísla nutno použít desetinnou tečku!")
            else:
                if (((AX>=limitStepsAMin) and (AX<=limitStepsAMax)) and ((BY>=limitStepsBMin) and (BY<=limitStepsBMax)) and ((CZ>=limitStepsCMin) and (CZ<=limitStepsCMax)) and (((D)>=limitStepsDMin) and ((D)<=limitStepsDMax))):     
                    stepsA = AX
                    stepsB = BY
                    stepsC = CZ
                    stepsD = D
                    pyCommand = int(6)
                    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
                else:
                    showerror(title="Chyba", message="Hodnoty natočení motoru jsou mimo rozsah. Příkaz nebyl proveden!")
                entryAX.delete(0, 'end')
                entryBY.delete(0, 'end')
                entryCZ.delete(0, 'end')
                entryD.delete(0, 'end')                

def clickButtonMoveTool():
    global limitSqueezeTool1Max, limitSqueezeTool1Min, limitTurnTool1Max, limitTurnTool1Min
    global limitSqueezeTool2Max, limitSqueezeTool2Min, limitTurnTool2Max, limitTurnTool2Min, toolType, gripperAngle, wristAngle

    enteredSqueezeAngle = entryGripperControl.get()
    enteredTurnAngle = entryGripperTurn.get()
    
    try:      
        enteredSqueezeAngle = int(enteredSqueezeAngle)
        enteredTurnAngle = int(enteredTurnAngle)
    except:
        showerror(title="Nesprávný vstup", message="Zadejte prosím celá čísla!")
    else:
        if toolType==1:
            if ((enteredSqueezeAngle>=limitSqueezeTool1Min) and (enteredSqueezeAngle<=limitSqueezeTool1Max) and (enteredTurnAngle>=limitTurnTool1Min) and (enteredTurnAngle<=limitTurnTool1Max)):     
                gripperAngle = enteredSqueezeAngle
                wristAngle = enteredTurnAngle
                pyCommand = int(7)
                sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            else:
                showerror(title="Chyba", message="Hodnoty natočení serv nástroje jsou mimo rozsah. Příkaz nebyl proveden!")
            entryGripperControl.delete(0, 'end')
            entryGripperTurn.delete(0, 'end')
        if toolType==2:
            if ((enteredSqueezeAngle>=limitSqueezeTool2Min) and (enteredSqueezeAngle<=limitSqueezeTool2Max) and (enteredTurnAngle>=limitTurnTool2Min) and (enteredTurnAngle<=limitTurnTool2Max)):     
                gripperAngle = enteredSqueezeAngle
                wristAngle = enteredTurnAngle
                pyCommand = int(7)
                sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
            else:
                showerror(title="Chyba", message="Hodnoty natočení serv nástroje jsou mimo rozsah. Příkaz nebyl proveden!")
            entryGripperControl.delete(0, 'end')
            entryGripperTurn.delete(0, 'end')

def clickButtonSpeed():
    global delaySteppers, scaleSpeedSteppers, windowSpeed, delayServos, scaleSpeedServos, windowSpeed
    windowSpeed = Tk()
    windowSpeed.title('Nastavení rychlostí pohonů a serv nástroje manipulátoru')
    windowSpeed.resizable(width=False, height=False)
    scaleSpeedSteppers = Scale(windowSpeed, from_=1, to=98, orient=HORIZONTAL, width = 20, length=350)
    scaleSpeedSteppers.grid(row=3, column=3, sticky=EW)
    scaleSpeedSteppers.set(2*delaySteppers)
    labelScaleSpeedSteppersFaster = Label(windowSpeed, text="Rychleji", width = 13)
    labelScaleSpeedSteppersFaster.grid(row=3, column=2, sticky=EW)   
    labelScaleSpeedSteppersSlower = Label(windowSpeed, text="Pomaleji", width = 13)
    labelScaleSpeedSteppersSlower.grid(row=3, column=5, sticky=EW)
    labelScaleSpeedSteppersMs = Label(windowSpeed, text="ms/krok", width = 7)
    labelScaleSpeedSteppersMs.grid(row=3, column=4, sticky=SW)
    
    labelSpeedSteppersTitle1 = Label(windowSpeed, text="Rychlost pohonů manipulátoru")
    labelSpeedSteppersTitle1.grid(row=0, column=3, columnspan=2, sticky=EW)
    labelSpeedSteppersTitle2 = Label(windowSpeed, text="Hodnota představuje dobu mezi jednotlivými kroky motorů v milisekundách")
    labelSpeedSteppersTitle2.grid(row=1, column=3, columnspan=2, sticky=EW)
    labelSpeedSteppersValue = Label(windowSpeed, text="Současná hodnota: ")
    labelSpeedSteppersValue.grid(row=2, column=3, columnspan=2, sticky=EW)
    labelSpeedSteppersValue.configure(text="Současná hodnota: " + str(2*delaySteppers) + " ms/krok")

    windowSpeed.eval('tk::PlaceWindow . center')
    scaleSpeedServos = Scale(windowSpeed, from_=0, to=250, orient=HORIZONTAL, width = 20, length=350)
    scaleSpeedServos.grid(row=13, column=3, sticky=EW)
    scaleSpeedServos.set(delayServos)
    labelScaleSpeedServosFaster = Label(windowSpeed, text="Rychleji", width = 13)
    labelScaleSpeedServosFaster.grid(row=13, column=2, sticky=EW)   
    labelScaleSpeedServosSlower = Label(windowSpeed, text="Pomaleji", width = 13)
    labelScaleSpeedServosSlower.grid(row=13, column=5, sticky=EW)
    labelScaleSpeedServosMs = Label(windowSpeed, text="ms/1°", width = 6)
    labelScaleSpeedServosMs.grid(row=13, column=4, sticky=SW)
    
    labelSpeedEmpty1 = Label(windowSpeed, text="       ")
    labelSpeedEmpty1.grid(row=8, column=3, columnspan=2, sticky=EW)
    labelSpeedServosTitle1 = Label(windowSpeed, text="Rychlost serv nástroje")
    labelSpeedServosTitle1.grid(row=9, column=3, columnspan=2, sticky=EW)
    labelSpeedServosTitle2 = Label(windowSpeed, text="Hodnota představuje dobu mezi natočením serv o stupeň v milisekundách")
    labelSpeedServosTitle2.grid(row=11, column=3, columnspan=2, sticky=EW)
    labelSpeedServosValue = Label(windowSpeed, text="Současná hodnota: ")
    labelSpeedServosValue.grid(row=12, column=3, columnspan=2, sticky=EW)
    labelSpeedServosValue.configure(text="Současná hodnota: " + str(delayServos) + " ms/1°")
    labelSpeedEmpty2 = Label(windowSpeed, text="       ")
    labelSpeedEmpty2.grid(row=14, column=3, columnspan=2, sticky=EW)
    buttonSetSpeedSteppers = Button(windowSpeed, text="Potvrdit", width=20, command=clickButtonSetSpeed)
    buttonSetSpeedSteppers.grid(row=15, column=3, columnspan=2, sticky=EW)
    
def clickButtonSetSpeed():
    global delaySteppers, windowSpeed, scaleSpeedSteppers, delayServos, scaleSpeedServos
    delaySteppers = round(int(scaleSpeedSteppers.get())/2)
    delayServos = int(scaleSpeedServos.get())
    if delaySteppers==0:
        delaySteppers = 1
    windowSpeed.destroy()  
    labelInfo.configure(text = "Byla nastavena rychlost pohonů manipulátoru a serv nástroje")

def clickButtonToolStep():
    global windowToolStep, stepServoAngleTurn, stepServoAngleSqueeze, entryClickButtonToolTurn, entryClickButtonToolSqueeze
    windowToolStep = Tk()
    windowToolStep.title('Nastavení kroku')
    labelClickButtonToolTurn = Label(windowToolStep, text="Natočení nástroje - úhly ve °:")
    labelClickButtonToolTurn.grid(row=2, column=1, sticky=EW)
    labelClickButtonToolSqueeze = Label(windowToolStep, text="Stisk nástroje - úhly ve °:")
    labelClickButtonToolSqueeze.grid(row=3, column=1, sticky=EW)     

    strVarStepToolTurn = StringVar(windowToolStep, value=str(stepServoAngleTurn))
    strVarStepToolSqueeze = StringVar(windowToolStep, value=str(stepServoAngleSqueeze))
    
    entryClickButtonToolTurn = Entry(windowToolStep, width=10, textvariable=strVarStepToolTurn)
    entryClickButtonToolTurn.grid(row=2, column=2)    
    
    entryClickButtonToolSqueeze = Entry(windowToolStep, width=10, textvariable=strVarStepToolSqueeze)   
    entryClickButtonToolSqueeze.grid(row=3, column=2)
    
    buttonClickButtonToolStep = Button(windowToolStep, text="Potvrdit", width=20, command=clickButtonClickButtonToolStep)
    buttonClickButtonToolStep.grid(row=5, column=2, sticky=EW)
    windowToolStep.resizable(width=False, height=False)
    windowToolStep.eval('tk::PlaceWindow . center')

def clickButtonClickButtonToolStep():
    global windowToolStep, stepServoAngleTurn, stepServoAngleSqueeze, entryClickButtonToolTurn, entryClickButtonToolSqueeze
    
    stepServoAngleTurnNew = entryClickButtonToolTurn.get()
    stepServoAngleSqueezeNew = entryClickButtonToolSqueeze.get()
    
    try:      
        stepServoAngleTurn = float(stepServoAngleTurnNew)
        stepServoAngleSqueeze = float(stepServoAngleSqueezeNew)
 
    except:
        showerror(title="Nesprávný vstup", message="Zadejte prosím čísla. Pro zadání desetinného čísla nutno použít desetinnou tečku!")
        windowToolStep.lift()
    else:
        labelInfo.configure(text = "Nové hodnoty kroků nástroje byly nastaveny")
        windowToolStep.destroy()
        updateLabels()
    
def clickButtonManipulatorStep():
    global windowManipulatorStep
    windowManipulatorStep = Tk()
    windowManipulatorStep.title('Nastavení kroku')
    labelClickButtonManStep = Label(windowManipulatorStep, text="Kroky motoru:")
    labelClickButtonManStep.grid(row=2, column=1, sticky=EW)
    labelClickButtonManAngle = Label(windowManipulatorStep, text="Úhly ve °:")
    labelClickButtonManAngle.grid(row=3, column=1, sticky=EW)     
    labelClickButtonManDist = Label(windowManipulatorStep, text="Vzdálenost mm:")
    labelClickButtonManDist.grid(row=4, column=1, sticky=EW)
    
    global entryClickButtonManStep, entryClickButtonManAngle, entryClickButtonManDist
    
    strVarStepManipulatorStep = StringVar(windowManipulatorStep, value=str(stepManipulatorStep))
    strVarStepManipulatorAngle = StringVar(windowManipulatorStep, value=str(stepManipulatorAngle))
    strVarStepManipulatorDist = StringVar(windowManipulatorStep, value=str(stepManipulatorDist))
    
    entryClickButtonManStep = Entry(windowManipulatorStep, width=10, textvariable=strVarStepManipulatorStep)
    entryClickButtonManStep.grid(row=2, column=2)    
    
    entryClickButtonManAngle = Entry(windowManipulatorStep, width=10, textvariable=strVarStepManipulatorAngle)   
    entryClickButtonManAngle.grid(row=3, column=2)
    
    entryClickButtonManDist = Entry(windowManipulatorStep, width=10, textvariable=strVarStepManipulatorDist) 
    entryClickButtonManDist.grid(row=4, column=2)
    
    buttonClickButtonManStep = Button(windowManipulatorStep, text="Potvrdit", width=20, command=clickButtonClickButtonManStep)
    buttonClickButtonManStep.grid(row=5, column=2, sticky=EW)
    windowManipulatorStep.resizable(width=False, height=False)
    windowManipulatorStep.eval('tk::PlaceWindow . center')

def clickButtonClickButtonManStep():
    global entryClickButtonManStep, entryClickButtonManAngle, entryClickButtonManDist, windowManipulatorStep
    global stepManipulatorStep, stepManipulatorAngle, stepManipulatorDist
    stepManipulatorStepNew = entryClickButtonManStep.get()
    stepManipulatorAngleNew = entryClickButtonManAngle.get()
    stepManipulatorDistNew = entryClickButtonManDist.get()
    
    try:      
        stepManipulatorStep = int(stepManipulatorStepNew)
        stepManipulatorAngle = float(stepManipulatorAngleNew)
        stepManipulatorDist = float(stepManipulatorDistNew)
 
    except:
        showerror(title="Nesprávný vstup", message="Zadejte prosím čísla. Pro zadání desetinného čísla nutno použít desetinnou tečku!")
        windowManipulatorStep.lift()
    else:
        labelInfo.configure(text = "Nové hodnoty kroků manipulátoru nastaveny")
        windowManipulatorStep.destroy()
        updateLabels()

def clickButtonHomePosSteppers():
    global stepsA, stepsB, stepsC, stepsD
    stepsA = int(1760)
    stepsB = int(120)
    stepsC = int(0)
    stepsD = int(100)
    pyCommand = int(6)
    sendDataToRobot(stepsA, stepsB, stepsC, stepsD, gripperAngle, wristAngle, pyCommand, delaySteppers, delayServos)
    labelInfo.configure(text = "Robot se nachází v domovské pozici")

# Tlacitka
buttonStartSerial = Button(window, text="Navázat komunikaci", width=20, command=clickButtonStartSerial)
buttonStartSerial.grid(row=1, column=1, sticky=W)

buttonXYZOmega = Button(window, text="XYZΩ", width=20, command=clickButtonXYZOmega)
buttonXYZOmega.grid(row=1, column=9, sticky=W)


buttonCalibration = Button(window, text="Kalibrace všech os", width=20, command=clickButtonCalib)
buttonCalibration.grid(row=2, column=1, sticky=W)

buttonABCDAngles = Button(window, text="ABCD - úhly", width=20, command=clickButtonABCDAngles)
buttonABCDAngles.grid(row=2, column=9, sticky=W)

buttonCalibrationSingle = Button(window, text="Kalibrace jednotlivých os", width=20, command=clickButtonCalibSingle)
buttonCalibrationSingle.grid(row=3, column=1, sticky=W)

buttonABCDSteps = Button(window, text="ABCD - kroky", width=20, command=clickButtonABCDSteps)
buttonABCDSteps.grid(row=3, column=9, sticky=W)

buttonReset = Button(window, text="Reset" ,width=20, command=clickButtonReset)
buttonReset.grid(row=4, column=1, sticky=W)

buttonHomePosSteppers = Button(window, text="Domovská pozice pohonů" ,width=20, command=clickButtonHomePosSteppers)
buttonHomePosSteppers.grid(row=4, column=9, sticky=W)

buttonSavePos = Button(window, text="Uložit pozici", width=20, command=clickButtonSavePos)
buttonSavePos.grid(row=5, column=1, sticky=W)

buttonHomePosTool = Button(window, text="Domovská pozice nástroje" ,width=20, command=clickButtonHomePosTool)
buttonHomePosTool.grid(row=5, column=9, sticky=W)

buttonSavedPos = Button(window, text="Uložené pozice", width=20, command=clickButtonSavedPos)
buttonSavedPos.grid(row=6, column=1, sticky=W)

buttonReadSensor = Button(window, text="Číst hodnotu senzoru", width=20, command=clickButtonReadSensor)
buttonReadSensor.grid(row=6, column=9, sticky=W)

buttonDelSavedPos = Button(window, text="Smazat uložené pozice", width=20, command=clickButtonDelSavedPos)
buttonDelSavedPos.grid(row=7, column=1, sticky=W)

buttonSpeedSteppers = Button(window, text="Rychlost", width=20, command=clickButtonSpeed)
buttonSpeedSteppers.grid(row=7, column=9, sticky=W)

buttonExportPos = Button(window, text="Exportovat", width=20, command=clickButtonExportPos)
buttonExportPos.grid(row=8, column=1, sticky=W)

buttonChooseTool = Button(window, text="Výběr nástroje", width=20, command=clickButtonChooseTool)
buttonChooseTool.grid(row=8, column=9, sticky=W)

buttonAXPlus = Button(window, text="+", width=3, command=clickButtonAXPlus)
buttonAXPlus.grid(row=4, column=3)

buttonAXMinus = Button(window, text="-", width=3, command=clickButtonAXMinus)
buttonAXMinus.grid(row=5, column=3)

buttonBYPlus = Button(window, text="+", width=3, command=clickButtonBYPlus)
buttonBYPlus.grid(row=4, column=4)

buttonBYMinus = Button(window, text="-", width=3, command=clickButtonBYMinus)
buttonBYMinus.grid(row=5, column=4)

buttonCZPlus = Button(window, text="+", width=3, command=clickButtonCZPlus)
buttonCZPlus.grid(row=4, column=5)

buttonCZMinus = Button(window, text="-", width=3, command=clickButtonCZMinus)
buttonCZMinus.grid(row=5, column=5)

buttonDPlus = Button(window, text="+", width=3, command=clickButtonDPlus)
buttonDPlus.grid(row=4, column=6)

buttonDMinus = Button(window, text="-", width=3, command=clickButtonDMinus)
buttonDMinus.grid(row=5, column=6)

buttonGripperCtrlPlus = Button(window, text="+", width=3, command=clickButtonGripperCtrlPlus)
buttonGripperCtrlPlus.grid(row=4, column=7)

buttonGripperCtrlMinus = Button(window, text="-", width=3, command=clickButtonGripperCtrlMinus)
buttonGripperCtrlMinus.grid(row=5, column=7)

buttonGripperTurnPlus = Button(window, text="+", width=3, command=clickButtonGripperTurnPlus)
buttonGripperTurnPlus.grid(row=4, column=8)

buttonGripperTurnMinus = Button(window, text="-", width=3, command=clickButtonGripperTurnMinus)
buttonGripperTurnMinus.grid(row=5, column=8)

buttonMoveManipulator = Button(window, text="Vykonat pohyb manipulátoru", command=clickButtonMoveManipulator)
buttonMoveManipulator.grid(row=8, column=3, columnspan=4, sticky=EW)

buttonMoveTool = Button(window, text="Vykonat pohyb nástroje", command=clickButtonMoveTool)
buttonMoveTool.grid(row=8, column=7, columnspan=2, sticky=EW)

buttonManipulatorStep = Button(window, text="Krok:", command=clickButtonManipulatorStep)
buttonManipulatorStep.grid(row=6, column=3, columnspan=4, sticky=EW)

buttonToolStep = Button(window, text="Krok:", command=clickButtonToolStep)
buttonToolStep.grid(row=6, column=7, columnspan=2, sticky=EW)

#Labely
labelManipulator = Label(window, text="Manipulátor")
labelManipulator.grid(row=1, column=3, columnspan=4, rowspan=1, sticky=EW)

labelTool = Label(window, text="Nástroj")
labelTool.grid(row=1, column=7, columnspan=2, rowspan=1, sticky=EW)

labelAX = Label(window, text="Osa A", width=10)
labelAX.grid(row=2, column=3, sticky=EW)

labelBY = Label(window, text="Osa B", width=10)
labelBY.grid(row=2, column=4, sticky=EW)

labelCZ = Label(window, text="Osa C", width=10)
labelCZ.grid(row=2, column=5, sticky=EW)

labelD = Label(window, text="Osa D", width=10)
labelD.grid(row=2, column=6, sticky=EW)

labelGripperControl = Label(window, text="Stisk", width=10)
labelGripperControl.grid(row=2, column=7, sticky=EW)

labelGripperTurn = Label(window, text="Natočení", width=10)
labelGripperTurn.grid(row=2, column=8, sticky=EW)

labelAXValue = Label(window, text="0")
labelAXValue.grid(row=3, column=3, sticky=EW)

labelBYValue = Label(window, text="0")
labelBYValue.grid(row=3, column=4, sticky=EW)

labelCZValue = Label(window, text="0")
labelCZValue.grid(row=3, column=5, sticky=EW)

labelDValue = Label(window, text="0")
labelDValue.grid(row=3, column=6, sticky=EW)

labelGripperCtrlVal = Label(window, text="0")
labelGripperCtrlVal.grid(row=3, column=7, sticky=EW)

labelGripperTurnVal = Label(window, text="0")
labelGripperTurnVal.grid(row=3, column=8, sticky=EW)

labelInfo = Label(window, text="Arduino nepřipojeno")
labelInfo.grid(row=10, column=2, columnspan=8, sticky=EW)

labelInfoLeft = Label(window, text="Spojení nenavázáno")
labelInfoLeft.grid(row=10, column=1, sticky=EW)

#Vstupní políčka
entryAX = Entry(window, width=5)
entryAX.grid(row=7, column=3)

entryBY = Entry(window, width=5)
entryBY.grid(row=7, column=4)

entryCZ = Entry(window, width=5)
entryCZ.grid(row=7, column=5)

entryD = Entry(window, width=5)
entryD.grid(row=7, column=6)

entryGripperControl = Entry(window, width=5)
entryGripperControl.grid(row=7, column=7)

entryGripperTurn = Entry(window, width=5)
entryGripperTurn.grid(row=7, column=8)

labelAXValue.configure(text = str(stepsA))
labelBYValue.configure(text = str(stepsB))
labelCZValue.configure(text = str(stepsC))
labelDValue.configure(text = str(stepsD))

controlMode = "ABCDSteps" #Výchozí ovládací mód při spuštění programu
updateLabels()
serialConnected = False
deactivateButtons()

window.eval('tk::PlaceWindow . center')

mainloop()