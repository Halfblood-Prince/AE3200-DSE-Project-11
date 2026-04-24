class Budget:
    def __init__(self, power, mass, volume, cost):
        self.power = power
        self.mass = mass
        self.volume = volume
        self.cost = cost

# init random example
powerSys = Budget(100, 50, 0.5, 1000)
propulsionSys = Budget(200, 100, 1.0, 5000)
flightControlSys = Budget(50, 20, 0.2, 2000)
navigationSys = Budget(30, 10, 0.1, 1500)
communicationSys = Budget(20, 5, 0.05, 1000)
payloadSys = Budget(150, 80, 0.8, 3000)