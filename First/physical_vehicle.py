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

        self.gps_longitude = 0
        self.gps_latitude = 0
        self.gps_age = 0
        self.gps_altitude = 0
        self.gps_speed_mps = 0
        self.gps_speed_kmph = 0
        self.gps_numsats = 0
    
    def print_stats(self):
        print("Vehicle Stats: ")
        print("  ADC1:                %s" % self.adc[0])
        print("  ADC2:                %s" % self.adc[1])
        print("  ADC3:                %s" % self.adc[2])
        print("  ADC4:                %s" % self.adc[3])
        print("  ADC5:                %s" % self.adc[4])
        print("  ADC6:                %s" % self.adc[5])
        print("  ADC7:                %s" % self.adc[6])
        print("  ADC8:                %s" % self.adc[7])
        print("  Pitch:               %s" % self.pitch)
        print("  Roll:                %s" % self.roll)
        print("  Yaw:                 %s" % self.yaw)
        print("  Pressure:            %s" % self.pressure)
        print("  Temp (C):            %s" % self.temp_c)
        print("  Alt (m):             %s" % self.alt)
        print("  Latitude:            %s" % self.gps_latitude)
        print("  Longitude:           %s" % self.gps_longitude)
        print("  GPS Age:             %s" % self.gps_age)
        print("  GPS Alt (m):         %s" % self.gps_altitude)
        print("  GPS Speed (m/s):     %s" % self.gps_speed_mps)
        print("  GPS Speed (km/h):    %s" % self.gps_speed_kmph)
        print("  Number of Satelites: %s" % self.gps_numsats)
