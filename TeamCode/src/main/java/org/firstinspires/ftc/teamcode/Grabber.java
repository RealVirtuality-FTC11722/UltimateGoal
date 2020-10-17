package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    public Servo servoClamp = null;
    public Servo servoLift = null;

    public Grabber(){ //constructor
    }

    public void init (HardwareMap hwMap){
        servoClamp = hwMap.get(Servo.class, "servoClamp" );
        servoLift = hwMap.get(Servo.class, "servoLift");
    }

    public void GrabberControls(boolean ClampButton, double LiftTrigger){
        servoLift.setPosition(LiftTrigger);
        if (ClampButton){
            servoClamp.setPosition(1.0);
        }
        //Ask drivers how they want to release.
    }
}
