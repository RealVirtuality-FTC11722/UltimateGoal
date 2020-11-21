package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    public Servo servoClamp = null;
    public DcMotor motorLift = null;
    public int UP_POS = 0;
    public int DOWN_POS = 100;
    public double GRAB_POS = 0.5;
    public double RELEASE_POS = 0;

    public Grabber(){ //constructor
    }

    public void init (HardwareMap hwMap){
        servoClamp = hwMap.get(Servo.class, "servoClamp" );
        motorLift = hwMap.get(DcMotor.class, "motorLift");
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void GrabberControls(boolean GrabButton, boolean ReleaseButton, boolean UpButton, boolean DownButton){
        if (UpButton) {
            Lift();
        }
        if (DownButton) {
            Lower();
        }
        //if (!motorLift.isBusy()){
        //    motorLift.setPower(0);
        //    motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //}

        if (GrabButton){
            servoClamp.setPosition(GRAB_POS);
        }
        if (ReleaseButton){
            servoClamp.setPosition(RELEASE_POS);
        }

        //Ask drivers how they want to release.
    }

    public void Lift(){
        motorLift.setTargetPosition(UP_POS);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(0.25);
    }
    public void Lower(){
        motorLift.setTargetPosition(DOWN_POS);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(0.25);
    }
    public void Grab(OpMode op){
        servoClamp.setPosition(GRAB_POS);
    }
    public void Release(OpMode op){
        servoClamp.setPosition(RELEASE_POS);
    }
}
