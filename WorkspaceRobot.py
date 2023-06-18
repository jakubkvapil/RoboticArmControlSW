from RobotFunctions import *
import csv

xyz = []

for i in range(limitStepsAMin, limitStepsAMax+1, 1):
    i = stepsToDegreesA(i)
    i = math.radians(i) 
    for j in range(limitStepsBMin,limitStepsBMax+1, 100):
        j = stepsToDegreesB(j)
        j = math.radians(j)
        for k in range(limitStepsCMin, limitStepsCMax+1, 100):
            k = stepsToDegreesC(k)
            k = math.radians(k)
            for l in range (limitStepsDMin, limitStepsDMax+1, 20):
                l = stepsToDegreesD(l)
                l = math.radians(l)
                try: 
                    xnew, ynew, znew, omeganew = forwardKinematics(i, j, k, l)
                except:
                    "Něco se nepovedlo."
                else:
                    xyz.append([xnew, ynew, znew])

fields = ['x', 'y', 'z']  
rows = xyz  
with open('data.csv', 'w') as f:
    write = csv.writer(f)
    write.writerow(fields)
    write.writerows(rows)