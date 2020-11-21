package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Lyesome on 2018-01-13.
 * This class is used to initialize all of the components that make up the robot.
 */

public class BotConfig {
    //Add components to robot build
    MecanumDrive Drive = new MecanumDrive();
    Collector Pickup = new Collector();
    Shooter RingThrower = new Shooter();
    Grabber WobbleGrabber = new Grabber();


    //VuMarkDecoder myVuMark = new VuMarkDecoder();

    Sensors mySensors = new Sensors();


    public BotConfig() { // constructor
    }

    //Method to initialize all the Hardware
    public void InitAuto(HardwareMap myNewHWMap){
        //Initialize Servos first to minimize movement

        //Then initialize sensors
        mySensors.init(myNewHWMap);


        //myVuMark.init(myNewHWMap);
        //Finally initialize motors

        //Drive.initAuto(myNewHWMap);
        //Drive.initGyro(myNewHWMap);
        //Pickup.init(myNewHWMap);
        RingThrower.init(myNewHWMap);
        //WobbleGrabber.init(myNewHWMap);
    }
    public void InitTele(HardwareMap myNewHWMap){
        //Initialize Servos first to minimize movement

        //initialize sensors
        //mySensors.initSensors(myNewHWMap);
        //Then initialize motors

        Drive.initTele(myNewHWMap);
        //Pickup.init(myNewHWMap);
        RingThrower.init(myNewHWMap);
        WobbleGrabber.init(myNewHWMap);
        //mySensors.init(myNewHWMap);

    }

    public void InitServos(HardwareMap myNewHWMap){
        //myGlyphLifter.initServos(myNewHWMap);

    }
    public void InitMotors(HardwareMap myNewHWMap){
        //myGlyphLifter.initMotors(myNewHWMap);

        Drive.initMotors(myNewHWMap);
    }
    public void InitSensors(HardwareMap myNewHWMap){
        Drive.initGyro(myNewHWMap);
        //myVuMark.init(myNewHWMap);
    }

}
