from performance import performanceAnalysis
from performance import parameters

area = 0.0625
C_D = 1.2
coaxialEfficiency = 1.6/2
DCDCefficiency = 0.94
ESCefficiency = 0.82
batteryEnergy = 244.2 #kWh
mass = 3.9
payloadPower = 150
motorEfficiency = 0.75
numProps = 8
# prop = Propeller.from_csv("8.0_E.csv", "PER3_6x4E.dat")
params = parameters(area, C_D, coaxialEfficiency, DCDCefficiency, batteryEnergy, ESCefficiency,
                    mass, motorEfficiency, numProps, payloadPower,
                    "C:\\Users\\SID-DRW\\PycharmProjects\\AE3200-DSE-Project-11\\performance\\8.0_E.csv", "PER3_6x4E.dat")

performanceAnalysis(params)