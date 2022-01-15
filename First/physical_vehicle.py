class PhysicalVehicle:
    
    def __init__(self):
        
        print ("New Physical Vehicle Created")
        
        self.servo_input = []
        for x in range(17):
            self.servo_input.append(0)
        
        self.adc = []
        for x in range(8):
            self.adc.append(0)

        self.pitch = 0
        self.roll = 0
        self.yaw = 0

        self.pressure = 0
        self.temp_c = 0
        self.alt = 0

        self.longitude = 0
        self.latitude = 0
    
    def print_stats(self):
        print("Vehicle Stats")