package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Lyesome on 2018-01-13.
 * This class is used to pass the gamepad controls to generalized methods that the other classes can use.
 * This allows all the controls to be defined in one location and makes it easier to change the controls if need.
 */

public class BotControls {

    public BotControls()  { // constructor
    }

    //GAMEPAD 1
    //Drive controls
    public static double  DriveYStick(LinearOpMode op)              {return op.gamepad1.left_stick_y; }
    public static double  DriveXStick(LinearOpMode op)              {return op.gamepad1.left_stick_x; }
    public static double  TurnStick(LinearOpMode op)                {return op.gamepad1.right_stick_x; }
    public static double  DriveThrottle(LinearOpMode op)            {return op.gamepad1.right_trigger;}


    //GAMEPAD 2
    //Collector controls
    public static boolean CollectorOnButton(LinearOpMode op)          {return  op.gamepad2.x;}
    public static boolean CollectorOffButton(LinearOpMode op)         {return  op.gamepad2.y;}

    //Shooter controls
    public static double  ShootTrigger(LinearOpMode op)              {return op.gamepad2.right_trigger;}
    public static boolean  FlyWheelOnButton(LinearOpMode op)         {return op.gamepad2.a;}
    public static boolean  FlyWheelOffButton(LinearOpMode op)        {return op.gamepad2.b;}
    public static boolean AimUpButton (LinearOpMode op)             {return op.gamepad2.dpad_up;}
    public static boolean AimDownButton (LinearOpMode op)            {return op.gamepad2.dpad_down;}

    //Grabber Controls
    public static boolean GrabButton (LinearOpMode op)                {return op.gamepad1.right_bumper;}
    public static boolean ReleaseButton (LinearOpMode op)              {return op.gamepad1.left_bumper;}
    public static boolean UpButton (LinearOpMode op)                {return op.gamepad1.y;}
    public static boolean DownButton (LinearOpMode op)              {return op.gamepad1.a;}
}

