from RobotFunctions import *

#Definice pozic
closedGripperBox1Pos = [50, 113]
closedGripperBox2Pos = [57, 113]
closedGripperBox3Pos = [63, 113]
closedGripperBox4Pos = [63, 113]
closedGripperBox5Pos = [51, 113]
closedGripperBox6Pos = [46, 113]

openedGripperBox1Pos = [50, 136]
openedGripperBox2Pos = [57, 136]
openedGripperBox3Pos = [63, 136]
openedGripperBox4Pos = [63, 136]
openedGripperBox5Pos = [51, 136]
openedGripperBox6Pos = [46, 136]

box1Pos = [277.7958452157261, -75.41038512228114, -52.16135684080166, -1.5063304964773834, 'cr']
box2Pos = [280.76007371847385, -29.323298610078435, -52.74046608446625, -1.5113058231037755, 'cr']
box3Pos = [280.6175445076336, 21.530760587235434, -52.8234270054746, -1.5109694678033636, 'cr']
box4Pos = [159.56775348759166, 12.243051673096224, -44.39172136369692, -1.9078345745427177, 'cr']
box5Pos = [166.93943315484353, -26.440608688325227, -43.86011518043665, -1.8993292246493172, 'cr']
box6Pos = [169.3274301184264, -64.36373236821821, -44.75039841800549, -1.8879665620897297, 'cr']

offsetPickUpBox = [0, 0, 55, 0, "cr"]

scalePos = [186.32601748041014, -157.2487692690248, -25.928142906262366, -1.6326088620531995,'cr']

closedGripperScalePos = [46, 113]
openedGripperScalePos = [46, 150]

waitForScalePos = [175.08367629531455, -147.76086017847268, -24.581812807887047, -1.691513724308008]
afterScalePos = [209.70314802714012, -119.61256385546903, 105.38381211297104, -1.3406584371515944]
sortedBoxAbove20gPos = [438.0650950953644, -178.4366812019666, 212.57839725130748, -0.3832281966921498]
sortedBoxBelow20gPos = [468.3694854479415, -47.05911107912635, 228.08342559588957, -0.348260904547846]
readyForSortingPos = [276.55625023298234, -29.98257726032457, 76.68072375982368, -1.233272312180111, 'cr', 46, 136]
afterSortedPos = [246.05126296713223, -128.4275083073415, 106.15091805283885, -1.2367024532893836, 'cr']

boxes = [[box1Pos, closedGripperBox1Pos, openedGripperBox1Pos],[box2Pos, closedGripperBox2Pos, openedGripperBox2Pos],[box3Pos, closedGripperBox3Pos, openedGripperBox3Pos],[box4Pos, closedGripperBox4Pos, openedGripperBox4Pos],[box5Pos, closedGripperBox5Pos, openedGripperBox5Pos],[box6Pos, closedGripperBox6Pos, openedGripperBox6Pos]]

initializeRobot("COM6", 115200)
setToolSpeed(40)
setRobotSpeed(40)
calibrateRobot()

def sortBox(boxPos, closedGripperBoxPos, openedGripperBoxPos):
    #Startovni pozice
    moveRobot(readyForSortingPos)
    #55 mm nad uchopnou pozici  
    moveRobot(offset(boxPos, offsetPickUpBox))
    #Otevreni nastroje a jeho spravne natoceni
    moveRobot(openedGripperBoxPos)
    #Linearni pohyb dolu
    moveRobotLinear(boxPos)
    #Uchopeni dilu
    moveRobot(closedGripperBoxPos)
    #Linearni pohyb nahoru
    moveRobotLinear(offset(boxPos, offsetPickUpBox))
    #Presun 55 mm nad vahu
    moveRobot(offset(scalePos, offsetPickUpBox))
    #Zavreni nastroje a jeho spravne natoceni
    moveRobot(closedGripperScalePos)
    #Pohyb linearne dolu k vaze
    moveRobotLinear(scalePos)          
    #Otevreni gripperu
    moveRobot(openedGripperBoxPos)
    #Pozice, kde rameno ceka do dokonceni vazeni
    moveRobot(waitForScalePos)
    #Samotne vazeni
    mass = readSensor1Value()
    #Pozice pro uchopeni dilu
    moveRobot(scalePos)
    #Linearni pohyb nahoru
    moveRobotLinear(offset(scalePos, offsetPickUpBox))
    moveRobot(afterScalePos)
    
    #Rozhodnu, do jakeho boxu bude dil vytrizen
    if mass >= 20:
        moveRobot(sortedBoxAbove20gPos)
    else:
        moveRobot(sortedBoxBelow20gPos)
    
    openGripper()
    moveRobot(afterSortedPos)

for i in boxes:
    sortBox(i[0], i[1], i[2])
    
moveRobot(homePosition)