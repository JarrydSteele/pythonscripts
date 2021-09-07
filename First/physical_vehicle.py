class PhysicalVehicle:
    
    def __init__(self):
        print ("New Physical Vehicle Created")
        self.servo_input = []
        for x in range(17):
            self.servo_input.append(0)
    
    def print_stats(self):
        print("Vehicle Stats")