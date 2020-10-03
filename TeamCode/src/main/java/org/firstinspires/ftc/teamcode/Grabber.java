package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    public Servo servoClamp = null;
    public Servo servoLift = null;

    public Grabber(){}

    public void init (HardwareMap hwMap){
        servoClamp= hwMap.get(Servo.class, "servoClamp" );
        servoLift= hwMap.get(Servo.class, "servoLift");
    }
}
