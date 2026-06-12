import numpy as np
import matplotlib.pyplot as plt
import propulsion.propClass as propClass

rho = 1.0225
gravity = 9.80665
inchToM = 2.54 / 100

class parameters:
    def __init__(self, area:float, C_D:float, coaxialEfficiency:float,
                 DCDCefficiency:float, batteryEnergy:float, ESCefficiency:float,
                 mass:float, motorEfficiency:float, numProps:int, payloadPower:float,
                 propellerPath:str, propellerName:str):
        self.area = area
        self.C_D = C_D
        self.coaxialEfficiency = coaxialEfficiency
        self.DCDCefficiency = DCDCefficiency
        self.batteryEnergy = batteryEnergy
        self.ESCefficiency = ESCefficiency
        self.mass = mass
        self.motorEfficiency = motorEfficiency
        self.numProps = numProps
        self.payloadPower = payloadPower
        propeller = propClass.Propeller.from_csv(propellerPath, propellerName)
        self.propeller = propeller
        self.propArea = np.pi * ( propeller.Diameter*inchToM/2 ) ** 2
        print(propeller.Diameter*inchToM/2)
        self.mechEfficiency = findMechEfficiency(propeller, self.propArea)

def parasiticPower(velocity, p:parameters)-> float:
    return 1/2 * p.C_D * rho * np.pow(velocity, 3) * p.area / p.coaxialEfficiency

def inducedPower(velocity, p:parameters)-> float:
    vHoverSq = p.mass * gravity / (2 * p.propArea * p.numProps * rho * p.coaxialEfficiency)
    vHover   = np.sqrt(vHoverSq)
    vSquared = np.square(velocity)
    vRelSq   = vSquared / vHoverSq
    inducedVelocity = vHover * np.sqrt(np.sqrt(1/4 * vRelSq * vRelSq + 1) - 1/2 * vRelSq)
    return inducedVelocity * np.sqrt((p.mass * gravity / ( p.numProps * p.coaxialEfficiency ))**2 +
           np.square(1/2 * p.C_D * rho * vSquared * p.area) )

def powerRequired(velocity, p:parameters)-> float:
    idealPower = parasiticPower(velocity, p) + inducedPower(velocity, p)
    realPower = p.numProps / ( p.mechEfficiency * p.motorEfficiency * p.ESCefficiency ) * idealPower
    safetyMargin = 1.1
    return realPower + p.payloadPower / p.DCDCefficiency * safetyMargin

def maxEnduranceVelocity(p:parameters, maxIter = 1000)-> float:
    v0 = 0
    P0 = powerRequired(v0, p)
    dv = 0.05
    for i in range(maxIter):
        P1 = powerRequired(v0 + dv, p)
        dP = (P1 - P0)/dv
        v0 -= 0.1 * dP
        P0 = powerRequired(v0, p)
    return v0

def maxRangeVelocity(p:parameters, maxIter = 1000)-> float:
    v0 = 1
    R0 = powerRequired(v0, p) / v0
    dv = 0.05
    for i in range(maxIter):
        R1 = powerRequired(v0 + dv, p) / ( v0 + dv )
        dR = (R1 - R0)/dv
        v0 -= 0.01 * np.sqrt(np.abs(dR)) * np.sign(dR)
        R0 = powerRequired(v0, p) / v0
    return v0

def findMechEfficiency(propeller, propArea, maxIter= 10000)-> float:
    thrust = np.array(list(propeller.Thrust.values()))
    power  = np.array(list(propeller.Power.values()))
    N = len(power)
    k0 = 0.5
    C0vec = ( power - 1/k0 * np.sqrt(np.pow(thrust, 3) / (2 * rho * propArea)) ) / power
    C0 = np.dot(C0vec, C0vec)
    dk = 0.01
    for i in range(maxIter):
        C1vec = ( power - 1/(k0+dk) * np.sqrt(np.pow(thrust, 3) / (2 * rho * propArea)) ) / power
        C1 = np.dot(C1vec, C1vec)
        dC = (C1 - C0)/dk
        k0 -= 0.01 * dC
        C0vec = ( power - 1 / k0 * np.sqrt(np.pow(thrust, 3) / (2 * rho * propArea)) ) / power
        C0 = np.dot(C0vec, C0vec)
    assert np.sqrt(np.abs(C0) / N) < 0.05
    return k0


def performanceAnalysis(parameters:parameters):
    p = parameters
    prop = p.propeller
    propThrust = np.array(list(prop.Thrust.values()))
    propPower = np.array(list(prop.Power.values()))
    energy = p.batteryEnergy * 3600 #J

    V_me = maxEnduranceVelocity(p)
    V_mr = maxRangeVelocity(p)
    print("mechanical efficiency:", p.mechEfficiency, "percent")
    print("max endurance velocity:", V_me, "m/s")
    print("max range velocity:", V_mr, "m/s")
    print("hover power:", powerRequired(0, p), "W")
    print("hover time with payload:", energy / powerRequired(0, p) / 60, "minutes")
    print("max endurance with payload:", energy / powerRequired(V_me, p) / 60, "minutes")
    print("max endurance without payload:", energy / (powerRequired(V_me, p) - p.payloadPower) / 60,
          "minutes")
    print("max range with payload:", energy * V_mr / powerRequired(V_mr, p))
    print("max range without payload:",  energy * V_mr / (powerRequired(V_mr, p) - p.payloadPower))

    thrusts = np.linspace(0, np.max(propThrust) * 1.1, 200)

    plt.plot(thrusts, np.sqrt(np.pow(thrusts, 3) / (2 * rho * p.propArea)) / p.mechEfficiency, label='prediction')
    #plt.plot(propThrust, propPower, c='orange')
    plt.scatter(propThrust, propPower, c='orange', label='data')
    plt.legend()
    plt.xlabel("Thrust (N)")
    plt.ylabel("Power (W)")
    plt.show()

    velocities = np.linspace(0, V_mr * 1.1, 200)
    powers = powerRequired(velocities, p)

    plt.plot(velocities, powers)
    plt.scatter(V_me, powerRequired(V_me, p), label='max endurance')
    plt.legend()
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Power (W)")
    plt.show()

    plt.plot(velocities, velocities / powers * 3600)
    plt.scatter(V_mr, V_mr / powerRequired(V_mr, p) * 3600)
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Specific range (m/Wh)")
    plt.show()

    plt.plot(velocities, energy / powers / 60)
    plt.scatter(V_me, energy / powerRequired(V_me, p) / 60, label='max endurance')
    plt.legend()
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Endurance (min)")
    plt.show()

    plt.plot(velocities, energy / powers * velocities)
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Range (m)")
    plt.show()


# print("propeller radius:", (prop.Diameter*2.54/2/100)**2*np.pi, "vs", params.propArea)
# plt.plot(list(prop.Thrust.values()), list(prop.Power.values()))
#
# print("efficiency:", params.mechEfficiency)
#
# thrusts = np.linspace(0, 10, 100)
#
# plt.plot(thrusts, np.sqrt(np.pow(thrusts, 3)/(2 * rho * params.propArea))/params.mechEfficiency)
# plt.scatter(list(prop.Thrust.values()), list(prop.Power.values()))
# plt.xlabel("Thrust (N)")
# plt.ylabel("Power (W)")
# plt.show()
#
# velocities = np.linspace(0, 8, 100)
# powers = powerRequired(velocities, params)
#
# V_me = maxEnduranceVelocity(params)
# print("max endurance velocity:", V_me)
#
# plt.plot(velocities, powers)
# plt.scatter(V_me, powerRequired(V_me, params))
# plt.xlabel("Velocity (m/s)")
# plt.ylabel("Power (W)")
# plt.show()
#
# V_mr = maxRangeVelocity(params)
# print("max range velocity:", V_mr)
# print("hover power:", powerRequired(0, params))
#
# plt.plot(velocities, velocities/powers*3600)
# plt.scatter(V_mr, V_mr/powerRequired(V_mr, params)*3600)
#
# plt.xlabel("Velocity (m/s)")
# plt.ylabel("Specific range (m/Wh)")
# plt.show()
#
# Energy = 244.2 * 3600
#
# plt.plot(velocities, Energy/powers / 60)
# plt.xlabel("Velocity (m/s)")
# plt.ylabel("Endurance (min)")
# print("hover time with payload:", Energy/powerRequired(0, params) /60, "minutes")
# print("max endurance with payload:", Energy/powerRequired(V_me, params) /60, "minutes")
# print("max endurance without payload:", Energy/( powerRequired(V_me, params) - params.payloadPower ) /60, "minutes")
#
# plt.show()
#
# plt.plot(velocities, Energy/powers * velocities)
# plt.xlabel("Velocity (m/s)")
# plt.ylabel("Range (m)")
#
# print("max range with payload:", Energy * V_mr / powerRequired(V_mr, params))
# print("max range without payload:", Energy * V_mr / ( powerRequired(V_mr, params) - params.payloadPower ))
#
# plt.show()
