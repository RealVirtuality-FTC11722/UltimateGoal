package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Collector {
    public CRServo servoRoller = null;
    public Servo servoRelease = null;
    public DcMotor motorDrum = null;

    public Collector(){ //constructor
    }

    public void init(HardwareMap hwMap){
        servoRoller = hwMap.get(CRServo.class, "servoRoller");
        servoRelease = hwMap.get(Servo.class, "servoRelease");
        motorDrum = hwMap.get(DcMotor.class, "motorDrum");

        servoRoller.setDirection(CRServo.Direction.FORWARD);
        servoRelease.setDirection(Servo.Direction.FORWARD);
        motorDrum.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void CollectorControls(boolean OnButton, boolean OffButton){
        if (OnButton){
            motorDrum.setPower(0.4);
        }
        if (OffButton){
            motorDrum.setPower(0);
        }
    }

    public void ReleaseRoller(){
        servoRelease.setPosition(1);
    }

}


