package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;

public class Shooter {
    public DcMotor motorFlyWheel = null;
    public Servo servoAimLift = null;
    public Servo servoPusher = null;
    public double LIFT_CHANGE = 0.002;
    public double PUSHER_IN = 0.97;
    public double PUSHER_OUT = 0.54;
    public double AIMER_TOP = 0.5;
    public double AIMER_BOTTOM = 0.0;

    public Shooter(){ //constructor
    }

    public void init(HardwareMap hwMap){
        motorFlyWheel = hwMap.get(DcMotor.class, "motorFlyWheel");
        servoAimLift = hwMap.get(Servo.class, "servoAimLift");
        servoPusher = hwMap.get(Servo.class, "servoPusher");
        servoPusher.setPosition(PUSHER_IN);
        servoAimLift.setDirection(Servo.Direction.REVERSE);
    }

    public void ShooterControls(double shootTrigger, Boolean flyWheelOnButton, Boolean flyWheelOffButton,
                                boolean AimUpButton, boolean AimDownButton){

        if (flyWheelOnButton){
            motorFlyWheel.setPower(1.0);
        }
        if (flyWheelOffButton) {
            motorFlyWheel.setPower(0.0);
        }
        //servoPusher.setPosition();
        servoPusher.setPosition(Range.clip(1 - shootTrigger,PUSHER_OUT,PUSHER_IN));
        if (AimUpButton) {
            servoAimLift.setPosition(Range.clip(servoAimLift.getPosition()+LIFT_CHANGE,AIMER_BOTTOM,AIMER_TOP));
        }
        if (AimDownButton) {
            servoAimLift.setPosition(Range.clip(servoAimLift.getPosition()-LIFT_CHANGE,AIMER_BOTTOM,AIMER_TOP));


        }
    }

    public void Shoot(OpMode op) throws InterruptedException {
        servoPusher.setPosition(PUSHER_IN);
        sleep(1000);
        servoPusher.setPosition(PUSHER_OUT);
        sleep(500);
    }
}
