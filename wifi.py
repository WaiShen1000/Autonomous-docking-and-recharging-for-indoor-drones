import time
import os
import subprocess
import marker_navigate

connected = 0
telloWifi = "TELLO-EF8AC4"

while 1:
	os.system(f'''cmd /c "netsh wlan connect name={telloWifi}"''')

	returned_value = subprocess.check_output("netsh wlan show interfaces")
	time.sleep(5)

	if b'TELLO-EF8AC4' in returned_value:
		print("Connected")
		marker_navigate.droneControl()

		while b'TELLO-EF8AC4' in returned_value:
			returned_value = subprocess.check_output("netsh wlan show interfaces")
			print("Waiting")
			time.sleep(5)
