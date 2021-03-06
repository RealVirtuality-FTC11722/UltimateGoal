/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Mode - Testing", group="Testing")
//@Disabled
public class DriverMode_Testing extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //create a new robot named ringGary
    //private BotConfig ringGary = new BotConfig();
    private final static int GAMEPAD_LOCKOUT = 500;
    Deadline gamepadRateLimit;
    boolean State;
    BlinkinLedDriver myLights;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
        myLights = new BlinkinLedDriver();
        myLights.init(hardwareMap, telemetry);
        //ringGary.InitTele(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("State:", State);
            telemetry.update();
            handleGamepad();
            if (myLights.displayKind == BlinkinLedDriver.DisplayKind.AUTO) {
                myLights.doAutoDisplay();
            } else {
                /*
                 * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.
                 */
            }
        }
    }

    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a) {
            myLights.setDisplayKind(BlinkinLedDriver.DisplayKind.MANUAL);
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            myLights.setDisplayKind(BlinkinLedDriver.DisplayKind.AUTO);
            gamepadRateLimit.reset();
        } else if ((myLights.displayKind == BlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.left_bumper)) {
            myLights.pattern = myLights.pattern.previous();
            myLights.displayPattern();
            gamepadRateLimit.reset();
        } else if ((myLights.displayKind == BlinkinLedDriver.DisplayKind.MANUAL) && (gamepad1.right_bumper)) {
            myLights.pattern = myLights.pattern.next();
            myLights.displayPattern();
            gamepadRateLimit.reset();
        }
    }}
