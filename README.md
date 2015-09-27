# AMP-IP_Simulator

The AMP simulator was developed as part of my masters thesis work in Aalto Univeristy.
Thesis title: "Modeling and Simulation of Autonomous Intersection Management protocols".
The thesis document can be found above. 

Instructions for compiling and running AMP simulator:

1. Install Apache Ant tool, if you don't have it already on your system.

  On Ubuntu systems, Install Ant by opening a terminal and typing : sudo apt-get install ant

  For Installing Ant on other versions of Linux, Windows or Mac systems please refer to the Install instructions on   the official Ant website , found here: https://ant.apache.org/manual/install.html

2. On a terminal 'cd' to the folder containing this 'readme' file. It should also contain a 'src' folder and a 'build.xml' file.

3. To compile the AMP simulator, type 'ant' (without the quotes) and press Enter.

4. Then to run the simulator, type 'ant run' (without the quotes) and press Enter.

5. After the simulator window opens, on the title bar above , click on 'simulator' then 'start' to start the simulator.


Notes:
In order for Ant to work you need to have a Java Development kit (jdk) installed.
The simulator was tested using openjdk-7-jdk which can be installed on ubuntu systems by typing: sudo apt-get install openjdk-7-jdk.

To clean the compilation you can always execute the command  'ant clean'. 

For testing purposes, the log.txt contains the log messages of the simulation run.

For showing log messages from a certain vehicle , on a terminal 'cd' into the folder which contains the log.txt file and type 'awk '/^#vehicle <VEHICLE-ID>/' log.txt > <VEHICLE-ID>.txt' (without the outer quotes), after replacing <VEHICLE-ID> by the vehicle-ID of the vehicle you want to examine, then press Enter. You will find beside the log.txt file another file <VEHICLE-ID>.txt which includes the specific vehicle's logs.   

Instructions for using the simulator while it is running:
1. You can zoom in or out by using the mouse scroll 
2. You can watch the simulation in slow motion by holding the shift key on your keyboard.
For further instructions on controlling simulator views and controls please refere to Simulation control section (section 5) in AIM simulator user guide , found here: http://www.cs.utexas.edu/~aim/aim4sim/aim4-release-1.0.3/aim4-root/docs/user.html 

