package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public DcMotor motorFlyWheel =null;
    public Servo servoAimLift =null;
    public Servo servoPusher =null;
    public boolean togglePressed = false;
    public boolean toggleReleased = true;

    public Shooter(){
    }

    public void init(HardwareMap hwMap){
        motorFlyWheel = hwMap.get(DcMotor.class, "motorFlyWheel");
        //servoAimLift = hwMap.get(Servo.class, "servoAimLift");
        servoPusher = hwMap.get(Servo.class, "servoPusher");
    }

    public void ShooterControls(double shootTrigger, Boolean flyWheelOnButton, Boolean flyWheelOffButton, double LiftTrigger){

        if (flyWheelOnButton){
            motorFlyWheel.setPower(1.0);
        }
        if (flyWheelOffButton) {
            motorFlyWheel.setPower(0);
        }

        servoPusher.setPosition(shootTrigger);


    }

}
