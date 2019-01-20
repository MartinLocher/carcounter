# ESP Car Counter in ULP

This code merges the Espressif ULP Pulse Counting example with LoraWan functionality and is also compileable on the Arduino IDE.

The intention of the project is is to count cars in an arbitrary parking lot, sending the counted number via LoraWan to a node red backend where it is processed in mysql and visualized on a dashboard in order to help employees to look for parking areas which free space instead of driving arround for nothing.

The lora nodes (based on the TTGO T-Beam ESP32 implementation) are positioned in the entry lane as well as in the exit lane of an parking area. A sonar sensor is attached to the node is detecting objects and raises a positive signal on an IO port of the ESP32.  

In order to save battery most of the time the lora node is in deepsleep and only the ULP portion is still active and counting the positive signalds.

Once the lora node wakes up it checks if there were car counted in its sleeptime and then transmits this numbers via LoraWan.
The beauty of ULP is also that as the transactions on the counter are atomic there is no need for serialization. The counter is counting while the main thread is sending data via LoraWan out of the box !

The application is at run time configurable by pushing down a byte stream from TTN. 

Currently the following configurations are possible:
1. byte sets the spreading factor
2. byte sets the sleep time in minutes
3. byte resets the car_count to a value

Thus a downlink like 0A 05 00 will set the spreading factor to 10, the sleep time to 5 minutes and resets the car counter to 0.

In order to compile the assembler files you need to follow this tutorial https://github.com/duff2013/arduino_ulp 
which is working like a charme and will propeller the ESP into another level. Thanks for that duff !
