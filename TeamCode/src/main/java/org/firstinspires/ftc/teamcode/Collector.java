package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collector {
    public CRServo servoPickup = null;
    public CRServo servoWheel = null;
    public DcMotor motorDrum = null;

    public Collector(){ //constructor
    }

    public void init(HardwareMap hwMap){
        servoPickup = hwMap.get(CRServo.class, "servoPickup");
        servoWheel = hwMap.get(CRServo.class, "servoWheel");
        motorDrum = hwMap.get(DcMotor.class, "motorDrum");

        servoPickup.setDirection(CRServo.Direction.FORWARD);
        servoWheel.setDirection(CRServo.Direction.FORWARD);
        motorDrum.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}


