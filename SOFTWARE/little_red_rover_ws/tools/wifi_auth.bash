echo "Attempting to contact robot..."
while true; do
	# Microcontroller defaults to 192.168.4.1 on its softAP
	if ping -c 1 192.168.4.1 &> /dev/null; then
		robot_ip="`curl -s -X GET 192.168.4.1/get-ip`" || echo "Something went wrong. Please file an issue https://github.com/usedhondacivic/little_red_rover/issues."
		if [[ "$robot_ip" == "0.0.0.0" ]]; then
			echo "Please connect your robot to wifi:"
			python3 /tools/idf/esp_prov/esp_prov.py --transport softAP --sec_ver 1
		else	
			echo "Found robot at: $robot_ip. You are connected!"
	  		break;
		fi
	else
	  echo "Robot not found. Please make sure you're connected to the robot's wifi network."
	  break;
	fi
done
