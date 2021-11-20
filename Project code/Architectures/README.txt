FOLDERS
#Additional files
The original code in the folder "PythonAPI\carla\agents\navigation" called local planner was edited to account for
multi-lane collision. New code created is between the comments #___New code___ and #___End___.

#Baseline
Autonomous vehicles roaming.

#maps
Attempt at creating more environments using OSM.

#multi-lane-platoon
Test scenarios for two platoons merging.

#single-lane-platoon
Closed environment for vehicles to platoon in roads with 2 lanes which go in opposite directions.

#V2V system
The vehicle-to-vehicle communication architectures before it was scrapped.

INSTRUCTIONS

1. Put the architectures folder in the examples folder located in the PythonAPI folder 
   e.g."C:\WindowsNoEditor\PythonAPI\examples"
2. Replace the local planner file in "PythonAPI\carla\agents\navigation" with the one provided in the Additional files folder
3. Run the carla simulator
4. Open a directory of one of the architectures in the cmd
   e.g. cd "C:\WindowsNoEditor\PythonAPI\examples\multi-lane-platoon"
5. Load the world with the code "py -3.7 world.py", then load the vehicles with "py -3.7 vehicles.py"