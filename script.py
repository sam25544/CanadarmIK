import time
from telnetlib import Telnet 

from ikpy.chain import Chain
from ikpy.link import URDFLink, OriginLink
import numpy as np
#from scipy.spatial.transform import Rotation as R

#import ikpy.utils.plot as plot_utils

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.spatial.transform import Rotation

import krpc
conn = krpc.connect(name='RMS Control')
vessel = conn.space_center.active_vessel
canvas = conn.ui.stock_canvas

PartsList = vessel.parts.all

def toGameAngles(IKAngles):
    AnglesDeg = [a*(180/np.pi) for a in IKAngles]
    AnglesDeg[2] = 90+AnglesDeg[2]
    AnglesDeg[3] = -AnglesDeg[3]
    AnglesDeg = AnglesDeg[1:-1]
    return AnglesDeg
def toIKAngles(AnglesDeg):
    AnglesDeg.insert(0, 0.0)
    AnglesDeg.append(0.0)
    AnglesDeg[3] = -AnglesDeg[3]
    AnglesDeg[2] = AnglesDeg[2]-90
    IKAngles = [a*(np.pi/180) for a in AnglesDeg]
    return IKAngles

def getModule(part, modName):
    for mod in part.modules:
        if mod.name == modName:
            return mod
    return None
def getPart(nametag):
    for part in vessel.parts.all:
        nametagMod = getModule(part, 'KOSNameTag')
        if nametagMod.has_field('name tag'):
            if nametagMod.get_field('name tag') == nametag:
                return part

for part in PartsList:
    mods = part.modules
    for mod in mods:
        if mod.name == 'KOSNameTag':
            #print(mod.fields)
            if mod.has_field('name tag'):
                #print('gloogus')
                #print(mod.get_field('name tag'))
                if mod.get_field('name tag') == 'ShoulderYaw':
                    ShoulderYaw = mod.part
                if mod.get_field('name tag') == 'ShoulderPitch':
                    ShoulderPitch = mod.part
                if mod.get_field('name tag') == 'ElbowPitch':
                    ElbowPitch = mod.part
                if mod.get_field('name tag') == 'WristPitch':
                    WristPitch = mod.part
                if mod.get_field('name tag') == 'WristYaw':
                    WristYaw = mod.part
                if mod.get_field('name tag') == 'WristRoll':
                    WristRoll = mod.part

jointslist = [ShoulderYaw, ShoulderPitch, ElbowPitch, WristPitch, WristYaw, WristRoll]
servoList = []
for joint in jointslist:
    mods = joint.modules
    for mod in mods:
        if (mod.name == 'ModuleRoboticRotationServo') or mod.name == ('ModuleRoboticServoHinge'):
            servoList.append(mod)
#print(ser.get_field('Target Angle')servoList)

screen_size = canvas.rect_transform.size
panel = canvas.add_panel()
rect = panel.rect_transform
rect.size = (180, 180)
rect.position = (150-(screen_size[0]/2), 0)

panelRoll = canvas.add_panel()
rectRoll = panelRoll.rect_transform
rectRoll.size = (180, 180)
rectRoll.position = (300-(screen_size[0]/2), (-1*screen_size[1]/2)+200)

#buttonUp = panel.add_button("Up")
#buttonUp.rect_transform.position = (0, 0)
#button_clicked = conn.add_stream(getattr, buttonUp, 'clicked')
#while True:
    # Handle the throttle button being clicked
#if button_clicked():
#    print('glooog!!')
#    print(ShoulderPitchMod.get_field('Current Angle'))
#    newAngle = float(ShoulderPitchMod.get_field('Target Angle'))-5.0
#    print(newAngle)
#    ShoulderPitchMod.set_field_float('Target Angle', newAngle)
#    button.clicked = False

time.sleep(0.1)


RMS_Chain = Chain(name='RMS', links=[
    OriginLink(),
    URDFLink(
        name="ShoulderYaw",
        origin_translation=[0,0,0],
        origin_orientation=[0, 0, 0],
        rotation=[0,0,1]
    ),
    URDFLink(
        name="ShoulderPitch",
        bounds=(-np.pi, 0),
        origin_translation=[0,0,215],
        origin_orientation=[0, 0, 0],
        rotation=[0,1,0]
    ),
    URDFLink(
        name="ElbowPitch",
        bounds=(0, np.pi),
        origin_translation=[3675,0,0],
        origin_orientation=[0, 0, 0],
        rotation=[0,1,0]
    ),
    URDFLink(
        name="WristPitch",
        origin_translation=[3665,0,0],
        origin_orientation=[0, 0, 0],
        bounds=(-2*np.pi/3, 2*np.pi/3),
        rotation=[0,1,0]
    ),
    URDFLink(
        name="WristYaw",
        origin_translation=[365,0,0],
        origin_orientation=[0, 0, 0],
        bounds=(-2*np.pi/3, 2*np.pi/3),
        rotation=[0,0,1]
    ),
    URDFLink(
        name="WristRoll",
        origin_translation=[205,0,0],
        origin_orientation=[0, 0, 0],
        rotation=[0,0,0]
    ),
    URDFLink(
        name="EndEffector",
        #use_symbolic_matrix=False,
        origin_translation=[610,0,0],
        origin_orientation=[0, 0, 0],
        rotation=[0,0,0]
    )
])

buttonUp = panel.add_button("Up")
buttonUp.rect_transform.position = (0, 50)
buttonUp.rect_transform.size = (40, 40)
clickUp = conn.add_stream(getattr, buttonUp, 'clicked')

buttonDown = panel.add_button("Down")
buttonDown.rect_transform.position = (0, -50)
buttonDown.rect_transform.size = (40, 40)
clickDown = conn.add_stream(getattr, buttonDown, 'clicked')

buttonL = panel.add_button("L")
buttonL.rect_transform.position = (-50, 0)
buttonL.rect_transform.size = (40, 40)
clickL = conn.add_stream(getattr, buttonL, 'clicked')

buttonR = panel.add_button("R")
buttonR.rect_transform.position = (50, 0)
buttonR.rect_transform.size = (40, 40)
clickR = conn.add_stream(getattr, buttonR, 'clicked')

buttonFwd = panel.add_button("Fwd")
buttonFwd.rect_transform.position = (50, 50)
buttonFwd.rect_transform.size = (40, 40)
clickFwd = conn.add_stream(getattr, buttonFwd, 'clicked')

buttonBack = panel.add_button("Back")
buttonBack.rect_transform.position = (-50, 50)
buttonBack.rect_transform.size = (40, 40)
clickBack = conn.add_stream(getattr, buttonBack, 'clicked')



buttonPitchUp = panelRoll.add_button("Up")
buttonPitchUp.rect_transform.position = (0, 50)
buttonPitchUp.rect_transform.size = (40, 40)
clickPitchUp = conn.add_stream(getattr, buttonPitchUp, 'clicked')

buttonPitchDown = panelRoll.add_button("Down")
buttonPitchDown.rect_transform.position = (0, -50)
buttonPitchDown.rect_transform.size = (40, 40)
clickPitchDown = conn.add_stream(getattr, buttonPitchDown, 'clicked')

buttonYawL = panelRoll.add_button("L")
buttonYawL.rect_transform.position = (-50, 0)
buttonYawL.rect_transform.size = (40, 40)
clickYawL = conn.add_stream(getattr, buttonYawL, 'clicked')

buttonYawR = panelRoll.add_button("R")
buttonYawR.rect_transform.position = (50, 0)
buttonYawR.rect_transform.size = (40, 40)
clickYawR = conn.add_stream(getattr, buttonYawR, 'clicked')

buttonRollL = panelRoll.add_button("Roll L")
buttonRollL.rect_transform.position = (-50, 50)
buttonRollL.rect_transform.size = (40, 40)
clickRollL = conn.add_stream(getattr, buttonRollL, 'clicked')

buttonRollR = panelRoll.add_button("Roll R")
buttonRollR.rect_transform.position = (50, 50)
buttonRollR.rect_transform.size = (40, 40)
clickRollR = conn.add_stream(getattr, buttonRollR, 'clicked')

#dockport = getPart('dockport')
#LEE = getPart('LEE')
#relPosGame = np.subtract(dockport.position(vessel.reference_frame), LEE.position(vessel.reference_frame))
#relPos = [-1000*(relPosGame[1]), 1000*relPosGame[0], -1000*(relPosGame[2])]
#print(relPos)

armPos = [8520, 0, 215]
armAngles = [0, 0, 0]
rotVec = [1,0,0]
posReadout = panel.add_text('X:' + str(int(armPos[0])) + ' Y:' + str(int(armPos[1])) + ' Z:' + str(int(armPos[2])))
posReadout.rect_transform.position = (0, 70)
posReadout.color = (255, 255, 255)

orReadout = panelRoll.add_text('P:' + str(int(armAngles[0])) + ' Y:' + str(int(armAngles[1])) + ' R:' + str(int(armAngles[2])))
orReadout.rect_transform.position = (0, 70)
orReadout.color = (255, 255, 255)

currentAngles = [0, 0, 0, 0, 0, 0]
for i in range(6):
    currentAngles[i] = float(servoList[i].get_field('Target Angle'))
#currentPos = RMS_Chain.forward_kinematics()
currentAnglesIK = toIKAngles(currentAngles)

armPos = RMS_Chain.forward_kinematics(currentAnglesIK)[:3, 3]
armOr = RMS_Chain.forward_kinematics(currentAnglesIK)[:3, 0]
armAngles = [
    (180/np.pi)*(np.arcsin(armOr[2])),
    (180/np.pi)*(np.arcsin(armOr[1])),
    (180/np.pi)*currentAngles[5]
]
#print(armOr)

def rotateMatrix(matrix, theta, axis):
    if axis == 'X':
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(theta), -np.sin(theta)],
                                    [0, np.sin(theta), np.cos(theta)]])
    elif axis == 'Y':
        rotation_matrix = np.array([[np.cos(theta), 0, np.sin(theta)],
                                    [0, 1, 0],
                                    [-np.sin(theta), 0, np.cos(theta)]])
    elif axis == 'Z':
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), np.cos(theta), 0],
                                    [0, 0, 1]])
    else:
        raise ValueError("Invalid axis value")

    return np.dot(matrix, rotation_matrix)


while True:
    currentAngles = [0, 0, 0, 0, 0, 0]
    for i in range(6):
        currentAngles[i] = float(servoList[i].get_field('Target Angle'))
    #currentPos = RMS_Chain.forward_kinematics()
    currentAnglesIK = toIKAngles(currentAngles)
    #print(len(currentAnglesIK))
    armOr = RMS_Chain.forward_kinematics(currentAnglesIK)[:3, :3]
    armRotation = Rotation.from_matrix(armOr)
    
    #print(armOr)

    if clickUp():
        armPos[2] += 100
        buttonUp.clicked = False
    if clickDown():
        armPos[2] -= 100
        buttonDown.clicked = False
    if clickL():
        armPos[1] += 100
        buttonL.clicked = False
    if clickR():
        armPos[1] -= 100
        buttonR.clicked = False
    if clickFwd():
        armPos[0] += 100
        buttonFwd.clicked = False
    if clickBack():
        armPos[0] -= 100
        buttonBack.clicked = False

    if clickPitchUp():
        #armOr = rotateMatrix(armOr, -5.0*(np.pi/180), 'Y')
        armAngles[0] += 5.0
        buttonPitchUp.clicked = False
    if clickPitchDown():
        #armOr = rotateMatrix(armOr, 5.0*(np.pi/180), 'Y')
        buttonPitchDown.clicked = False
        armAngles[0] -= 5.0
    if clickYawL():
        #armOr = rotateMatrix(armOr, 5.0*(np.pi/180), 'Z')
        buttonYawL.clicked = False
        armAngles[1] += 5.0
    if clickYawR():
        #armOr = rotateMatrix(armOr, -5.0*(np.pi/180), 'Z')
        buttonYawR.clicked = False
        armAngles[1] -= 5.0
    if clickRollL():
        #armOr = rotateMatrix(armOr, 5.0*(np.pi/180), 'X')
        buttonRollL.clicked = False
        armAngles[2] += 5.0
    if clickRollR():
        #armOr = rotateMatrix(armOr, -5.0*(np.pi/180), 'X')
        buttonRollR.clicked = False
        armAngles[2] -= 5.0
    
    rotVec = [
        np.cos(armAngles[1] * (np.pi/180)),
        np.sin(armAngles[1] * (np.pi/180)),
        np.sin(armAngles[0] * (np.pi/180))
    ]
    print(rotVec)

    IKSolve = RMS_Chain.inverse_kinematics(
        target_position=armPos,
        initial_position=currentAnglesIK,
        #target_orientation=Rotation.from_euler('YZX', [armAngles[0], armAngles[1], armAngles[2]], degrees=True).as_matrix(),
        target_orientation=rotVec,
        orientation_mode="X"
    )
    #print(armPos)
    #print(toGameAngles(IKSolve))
    posReadout.content = 'X:' + str(int(armPos[0])) + ' Y:' + str(int(armPos[1])) + ' Z:' + str(int(armPos[2]))
    orReadout.content = 'P:' + str(int(armAngles[0])) + ' Y:' + str(int(armAngles[1])) + ' R:' + str(int(armAngles[2]))

    for i in range(5):
        servoList[i].set_field_float('Target Angle', toGameAngles(IKSolve)[i])
    servoList[5].set_field_float('Target Angle', armAngles[2])

    time.sleep(0.1)




ax = plt.figure().add_subplot(111, projection='3d')
ax.axis('scaled')

IKSolve = RMS_Chain.inverse_kinematics(
    target_position=[6000, 500, 2500],
    target_orientation=[1,1,0],
    orientation_mode="X"
)
RMS_Chain.plot(IKSolve, ax)


#print(AnglesDeg)

AnglesRads = []
#for servo in group.servos:
#    AnglesRads.append(servo.position)
#print(AnglesRads)

plt.show()