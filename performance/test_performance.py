
from performance import *

area = [0.05, 0.125, 1]
C_D = [0.5, 1, 2]
coaxialEfficiency = [1.6/2, 1, 0.6]
ESCefficiency = [0.82, 0.9, 0.6]
DCDCefficiency = [1, 0.94, 0.7]
batteryEnergy = [300, 244.2, 70] #kWh
mass = [1, 3.9, 7]
numProps = [8, 6, 4]
payloadPower = [150, 175, 125]
motorEfficiency = [0.75, 0.9, 0.5]
path = "8.0_E.csv"
name = ["PER3_6x4E.dat", "PER3_5x75E.dat", "PER3_4x4E-3.dat"]

paramsSet = []

for i in range(len(coaxialEfficiency)):
    paramsSet.append(
        parameters(area[i], C_D[i], coaxialEfficiency[i], DCDCefficiency[i], batteryEnergy[i], ESCefficiency[i],
                   mass[i], motorEfficiency[i], numProps[i], payloadPower[i],
                   path, name[i])
    )

# area is non negative
def test_parameterClass_01():
    for p in paramsSet:
        assert p.area > 0

# mech efficiency is physical
def test_parameterClass_02():
    for p in paramsSet:
        assert 0 < p.mechEfficiency < 1

# parameters are the same as given
def test_parameterClass_03():
    for i in range(len(paramsSet)):
        assert paramsSet[i].C_D == C_D[i]
        assert paramsSet[i].coaxialEfficiency == coaxialEfficiency[i]
        assert paramsSet[i].DCDCefficiency == DCDCefficiency[i]
        assert paramsSet[i].batteryEnergy == batteryEnergy[i]
        assert paramsSet[i].ESCefficiency == ESCefficiency[i]
        assert paramsSet[i].mass == mass[i]
        assert paramsSet[i].motorEfficiency == motorEfficiency[i]
        assert paramsSet[i].numProps == numProps[i]
        assert paramsSet[i].payloadPower == payloadPower[i]

# power required due to parasitic drag 0 at hover
def test_parasiticPower_01():
    for p in paramsSet:
        assert parasiticPower(0, p) == 0

# parisitic power is an increasing function of velocity
def test_parasiticPower_02():
    for p in paramsSet:
        Ppar0 = 0
        for v in np.linspace(0.1, 10, 4):
            Ppar1 = parasiticPower(v, p)
            assert Ppar1 >= Ppar0
            Ppar0 = Ppar1

def test_inducedPower():
    for p in paramsSet:
        for v in np.linspace(0.1, 10, 4):
            assert inducedPower(v, p) > 0

def test_powerRequired():
    for p in paramsSet:
        for v in np.linspace(0.1, 10, 4):
            assert powerRequired(v, p) >= parasiticPower(v, p) + inducedPower(v, p)

# max endurance velocity is non-negative
def test_maxEnduranceVelocity_01():
    for p in paramsSet:
        assert maxEnduranceVelocity(p) >= 0

# power required at other velocities is smaller than or approximately equal to
# power required for max endurance velocity
def test_maxEnduranceVelocity_02():
    for p in paramsSet:
        v_me = maxEnduranceVelocity(p)
        assert powerRequired(v_me, p) <= 1.01 * powerRequired(0, p)
        assert powerRequired(v_me, p) <= 1.01 * powerRequired(v_me * 1.1, p)
        assert powerRequired(v_me, p) <= 1.01 * powerRequired(v_me * 0.9, p)

 # test convergence
def test_maxEnduranceVelocity_03():
    for p in paramsSet:
        v_me1 = maxEnduranceVelocity(p, 1000)
        v_me2 = maxEnduranceVelocity(p, 1000+1)
        v_me3 = maxEnduranceVelocity(p, 1000+10)

        assert abs(1 - v_me2 / v_me1) < 0.01
        assert abs(1 - v_me3 / v_me1) < 0.01

# max range velocity is greater than max endurance velocity (so also, positive)
def test_maxRangeVelocity_01():
    for p in paramsSet:
        assert maxRangeVelocity(p) > maxEnduranceVelocity(p)

# max range velocity has greater than or approximately equal specific range compared to other velocities
def test_maxRangeVelocity_02():
    for p in paramsSet:
        v_mr = maxRangeVelocity(p)
        print(v_mr)
        assert 1.01 * v_mr / powerRequired(v_mr, p) >= v_mr * 1.1 / powerRequired(v_mr * 1.1, p)
        assert 1.01 * v_mr / powerRequired(v_mr, p) >= v_mr * 0.9 / powerRequired(v_mr * 0.9, p)

# test convergence
def test_maxEnduranceVelocity_03():
    for p in paramsSet:
        v_mr1 = maxRangeVelocity(p, 1000)
        v_mr2 = maxRangeVelocity(p, 1000 + 1)
        v_mr3 = maxRangeVelocity(p, 1000 + 10)
        v_mr4 = maxRangeVelocity(p, 1000 * 2)

        assert abs(1 - v_mr2 / v_mr1) < 0.01
        assert abs(1 - v_mr3 / v_mr1) < 0.01
        assert abs(1 - v_mr4 / v_mr1) < 0.01

# test efficiency is in physical range
def test_findMechanicalEfficiency_01():
    for p in paramsSet:
        assert 0 < findMechEfficiency(p.propeller, p.propArea) < 1

# test convergence
def test_findMechanicalEfficiency_02():
    for p in paramsSet:
        e1 = findMechEfficiency(p.propeller, p.propArea, 10000)
        e2 = findMechEfficiency(p.propeller, p.propArea, 10000 + 11)
        e3 = findMechEfficiency(p.propeller, p.propArea, 10000 + 20)
        e4 = findMechEfficiency(p.propeller, p.propArea, 10000 * 2)

        assert abs(1 - e2 / e1) < 0.01
        assert abs(1 - e3 / e1) < 0.01
        assert abs(1 - e4 / e1) < 0.01

