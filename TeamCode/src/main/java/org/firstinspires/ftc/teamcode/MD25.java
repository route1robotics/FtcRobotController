/*
Copyright 2020 FIRST Tech Challenge Team 16989

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class MD25 extends LinearOpMode {

    RobotHardware25 robot = new RobotHardware25(this);

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        this.resetRuntime();

        //variables
        double targetPower = 0;
        double strafePower = 0;
        double turnPower = 0;
        double ferrisCode = 0;
        boolean negFerrisCode = false;
        boolean resetFerris = false;
        double nuclearLaunchCodes = 0;
        boolean armServoThingyIDK = false;
        boolean mandibleButton = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Front Left", robot.frontLeftPosition());
            telemetry.addData("Front Right", robot.frontRightPosition());
            telemetry.addData("Back Left", robot.backLeftPosition());
            telemetry.addData("Back Right", robot.backRightPosition());
            telemetry.addData("Obelisk ID Code",robot.procureAprilTagList(1));
            telemetry.addData("Blue Zone ID Code",robot.procureAprilTagList(0));
            telemetry.addData("Red Zone ID Code",robot.procureAprilTagList(2));
            telemetry.addData("Colour Hue in Degrees",robot.getColorData());
            telemetry.addData("Current hue",robot.getColorHue());
            telemetry.addData("X Rotation",robot.imuRecorder(0));
            telemetry.addData("Y Rotation",robot.imuRecorder(1));
            telemetry.addData("Z Rotation",robot.imuRecorder(2));
            telemetry.addData("Mandible State",robot.grab(mandibleButton));
            telemetry.update();

            //DRIVER CONTROLLER-----------------------------------------------------
            targetPower = -this.gamepad1.left_stick_y;
            strafePower = (this.gamepad1.left_trigger - this.gamepad1.right_trigger) * 0.8;//this.gamepad1.left_stick_x;//
            turnPower = -this.gamepad1.right_stick_x * 0.8;
            robot.driveRobot(targetPower, strafePower, turnPower);

            //OPERATOR CONTROLLER---------------------------------------------------
            //Triggers / Bumpers
            ferrisCode = this.gamepad2.left_trigger;
            nuclearLaunchCodes = this.gamepad2.right_trigger;
            negFerrisCode = this.gamepad2.left_bumper;
            resetFerris = this.gamepad2.right_bumper;
            robot.ferris(ferrisCode, negFerrisCode, resetFerris); // l trigger, both bumpers
            robot.launch(nuclearLaunchCodes); // r trigger
            //Letter Buttons
            armServoThingyIDK = this.gamepad2.a;
            mandibleButton = this.gamepad2.b;
            robot.grab(mandibleButton); // b
        }
    }
}

//Hope you won