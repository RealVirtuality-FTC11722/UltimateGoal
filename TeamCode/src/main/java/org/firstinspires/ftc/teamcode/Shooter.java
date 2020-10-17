package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    public DcMotor motorFlyWheel = null;
    public Servo servoAimLift = null;
    public Servo servoPusher = null;
    public double liftChange = 0.05;

    public Shooter(){ //constructor
    }

    public void init(HardwareMap hwMap){
        motorFlyWheel = hwMap.get(DcMotor.class, "motorFlyWheel");
        servoAimLift = hwMap.get(Servo.class, "servoAimLift");
        servoPusher = hwMap.get(Servo.class, "servoPusher");
    }

    public void ShooterControls(double shootTrigger, Boolean flyWheelOnButton, Boolean flyWheelOffButton,
                                boolean AimUpButton, boolean AimDownButton){

        if (flyWheelOnButton){
            motorFlyWheel.setPower(1.0);
        }
        if (flyWheelOffButton) {
            motorFlyWheel.setPower(0.0);
        }
        servoPusher.setPosition(Range.clip(shootTrigger,0.08,1.0));
        if (AimUpButton) {
            servoAimLift.setPosition(Range.clip(servoAimLift.getPosition()+liftChange,0,1));
        }
        if (AimDownButton) {
            servoAimLift.setPosition(Range.clip(servoAimLift.getPosition()-liftChange,0,1));


        }
        //servoAimLift.setPosition(LiftTrigger);
    }

}
