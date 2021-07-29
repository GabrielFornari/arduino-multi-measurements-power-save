# multi-measurements-power-save

Copy of arduino-multi-measurements. The main idea is to get the same measurements, but with a rate of 60 sec (1 min). 


Created by Gabriel Fornari on 29/07/2021

New changes
   - Stabilized version for using with battery
     -> "prints' removed
   - Pressure measurement fixed 
     -> Temperature measurements caused problems to the pressure measurement
     
   # Some issues   
     # DHT is giving 1º higher and 1% higher then normal
     # MPU temperature is wrong
     # MPU acce and gyro should be improved

     # Turn out the leds to save energy (not working)
     # Too much time is wasted to save data in File
       # It can be reduced if File open() and close() are removed
       # Maybe if I add flush()...
       # If File open()/close() are removed, the time to measure should be rearranged
     # Vin voltage should be measured
     # Saving data into File sometimes takes 200 ms (don't know why)
     # Change millis() to reset it every new second
