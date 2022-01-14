class PhysicalVehicle:
    
    def __init__(self):
        print ("New Physical Vehicle Created")
        self.servo_input = []
        for x in range(17):
            self.servo_input.append(0)
        self.adc0 = 0
        self.adc1 = 0
        self.adc2 = 0
        self.adc3 = 0
        self.adc4 = 0
        self.adc5 = 0
        self.adc6 = 0
        self.adc7 = 0

        self.pitch = 0
        self.roll = 0
        self.yaw = 0

        self.pressure = 0
        self.temp_c = 0
        self.alt = 0
    
    def print_stats(self):
        print("Vehicle Stats")