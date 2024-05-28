# ir_sensor package
This package:
1. Reads the second channel of ADC NAVIO2 for getting distance information from the Sharp 2y0a02
2. Uses a median filter to filter the obtained data
3. Converts voltage readings to distance
4. Sends spatial information to ArduCopter via fake_gps plugin present in mavros_extras package    
