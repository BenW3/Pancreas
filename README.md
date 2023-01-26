# Pancreas
The Pancreas is an autonomous agricutural robot. There was an acronym that went with Pancreas, it has since been lost to time. 

Put the following at the end of the .bashrc file on the Pancreas' onboard computer.

```
echo Running at boot

cd /media/pancreas/Pancreas\ SD

/bin/python3 /media/pancreas/Pancreas\ SD/fieldTest13/Pancreas\ Main\ Field\ Test13.py &

Then, set the terminal to open on startup.
```

If changes are made and the name of the folder and main python code is changed, be sure to change the name of the file and file folder run on startup.
Be sure to load the custom libraries and the Mux library onto the computer uploading to the arduinos. These are the StepperMotorClosedLoop2, PowerSensors, Sparkfun_I2C_Mux, and Compass1 folders.
