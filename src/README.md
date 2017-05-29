## Platform
The project was copiled and tested on a Windows 10 laptop.

It was compiled in Visual Studio 2017 using cmake. Some of the syntax in the main.cpp, specifically for the uWS 
specific commands was changed to get the project to compile in the specified enviorments.

The instructions at the slack link below were used to get the project working:
https://carnd.slack.com/archives/C4Z4GFX0X/p1493794760012703

## Project implementation
The project follows the general flow of the steps to implemet PID control.

The parameters were tuned both manually and with the use of twiddle. 

# Manual Tuning
With trial and error following observation were paid regarding each parameter:
* P: A high gain was resulting a lot of zig zag motion in the car. This is was especially the case 
when the car was coming out of a turn onto a straight section of the track. The Kp value waa initially 
set at an order of 1e-1

* I: As there is no inherent drift in the system, the Ki value was not going to be significantly helpul.
Higher values were still tested and it was found that car went off the track very quickly. Therefor a value 
in the order of 1e-2 was used in the beginning. 

* D: The Kd gain was observed to be the most significant part of the controller. A higher value was helpfull in
keeping the car on the track. A value of 2 was initially used for Kd.

# Tuning with Twiddle
After manually testing how the car behaved with different ranges of PID gains, the twiddle algorithm was used to 
fine tune the parameters. Differnt techniques were used to test changes in the parameters by the algorithms. 

1. Recording the error for 1000-1500 in the beginning of the track and resetting the simulator with new parameters from
twiddle.

2. Letting the car go around the car while tracking error for 800-1000 frames and updating parameters using twiddle. 

Several iterations of the above two methods were carried out to settle on the final set of gains.
The current initialization of PID in the main file is set to the values that were observed to work
best while running the simulatior on my laptop. The values are set at:

Kp: 0.178899 Kd: 1.57257 Ki: 0.0022

With the above numbers the twiddle algorithms stops tuning the parameters after 2-3 iterations. The stopping 
condition is set at average squared cte going below 0.1.

Note: Changing the settings of the simulator was observed to produce different results. Therefore, all the testing 
and tuning was carried out at 640x480 screen resolution with graphics quality set to Fantastic. The above values 
work best for those setting on my machine and the car runs indefinitely around the track.

