# 5036-rapid-react
frc team 5036's computer stuffy stuff for 2022


## TODO
1. ~~Drive with controller~~
2. ~~Autonomous drive PID (straight) - encoders~~
3. ~~Autonomous turning PID (gyro)~~
4. Intake logic
5. Conveyor logic
    conveyor and intake logic will take some time to get it automated and good
6. Camera display on Smart Dashboard
7. LEDs for driver feedback, programmer like a regular Spark Motor controller and PWM output  
    Flash lights when ball is picked up, this is to be detected by detecting a CURRENT spike (momentary increase) on the intake motors  
    We will have to see what baseline current is, then see what the spike is when we get a ball. Might also have to do some simple filtering where we ignore current spike on start up (1s ignore) because that also draws large current  
    A quick 3 flash when ball is picked up.  
    Also start flashing yellow or something when 30 seconds left
8. Drive train characterization
9. Ball dropping logic (if using motor)
