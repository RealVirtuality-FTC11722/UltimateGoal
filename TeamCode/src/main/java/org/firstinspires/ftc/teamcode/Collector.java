package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Collector {
    public DcMotor motorRoller = null;
    public Servo servoRelease = null;
    public DcMotor motorDrum = null;

    public Collector(){ //constructor
    }

    public void init(HardwareMap hwMap){
        motorRoller = hwMap.get(DcMotor.class, "motorRoller");
        servoRelease = hwMap.get(Servo.class, "servoRelease");
        motorDrum = hwMap.get(DcMotor.class, "motorDrum");

        motorRoller.setDirection(CRServo.Direction.FORWARD);
        servoRelease.setDirection(Servo.Direction.FORWARD);
        motorDrum.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void CollectorControls(boolean OnButton, boolean OffButton){
        if (OnButton){
            motorDrum.setPower(0.4);
            motorRoller.setPower(1);
        }
        if (OffButton){
            motorDrum.setPower(0);
            motorRoller.setPower(0);
        }
    }

    public void ReleaseRoller(){
        servoRelease.setPosition(1);
    }

}


