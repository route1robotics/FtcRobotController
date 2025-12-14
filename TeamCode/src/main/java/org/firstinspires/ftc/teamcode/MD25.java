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
        double nuclearLaunchCodes = 0;
        boolean mandibleOpen = false;
        boolean mandibleClose = false;
        boolean opDpadUp = false;
        boolean opDpadDown = false;
        String alliance = "Red";
        boolean targetSystem = false;
        boolean buttonX = false;
        boolean mandibleHalf = false;
        double ferrisRotate = 0;
        boolean ferrisQuarterTurn = false;



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.updateOdometry();

            telemetry.addData("Front Left",             robot.frontLeftPosition());
            telemetry.addData("Front Right",            robot.frontRightPosition());
            telemetry.addData("Back Left",              robot.backLeftPosition());
            telemetry.addData("Back Right",             robot.backRightPosition());
            telemetry.addData("Obelisk ID Code",        robot.procureAprilTagList(1));
            telemetry.addData("Blue Zone ID Code",      robot.procureAprilTagList(0));
            telemetry.addData("Red Zone ID Code",       robot.procureAprilTagList(2));
            telemetry.addData("X pos",                  robot.globalX);
            telemetry.addData("Y pos",                  robot.globalY);
            telemetry.addData("Heading",                robot.headingDeg);
            telemetry.addData("AprilTag info",          robot.getAllAprilTagInfo());
            telemetry.addData("launch power",           robot.GetLaunchPower());
            telemetry.update();

            //DRIVER CONTROLLER-----------------------------------------------------
            targetPower = -this.gamepad1.left_stick_y;
            strafePower = (this.gamepad1.left_trigger - this.gamepad1.right_trigger) * 0.8;//this.gamepad1.left_stick_x;//
            turnPower = -this.gamepad1.right_stick_x * 0.8;
            robot.driveRobot(targetPower, strafePower, turnPower);

            // targeting / team setting
            if (this.gamepad1.x) {
                alliance = "Blue";
            }
            if (this.gamepad1.b) {
                alliance = "Red";
            }
            targetSystem = this.gamepad1.a;
            if (alliance.equals("Blue")) {
                robot.lockAndLoad(targetSystem, 20, 6);
            }
            else {
                robot.lockAndLoad(targetSystem, 24, 4);
            }


            //OPERATOR CONTROLLER---------------------------------------------------

            //Triggers / Bumpers / Sticks
            nuclearLaunchCodes = this.gamepad2.right_trigger;
            ferrisRotate = this.gamepad2.left_stick_y;
            robot.ferris(ferrisRotate);
            robot.launch(nuclearLaunchCodes);
            ferrisQuarterTurn = this.gamepad2.left_bumper;
            //robot.ferrisQuarter(ferrisQuarterTurn);

            //D-Pad
            opDpadUp = this.gamepad2.dpad_up;
            opDpadDown = this.gamepad2.dpad_down;
            robot.LaunchPowerUp(opDpadUp);
            robot.LaunchPowerDown(opDpadDown);

            //Letter Buttons
            mandibleOpen = this.gamepad2.a;
            mandibleClose = this.gamepad2.b;
            mandibleHalf = this.gamepad2.y;
            robot.mandOpen(mandibleOpen);
            robot.mandClose(mandibleClose);
            robot.mandHalf(mandibleHalf);
            buttonX = this.gamepad2.x;

            robot.LaunchKicker(buttonX);
            robot.ReturnKicker(!buttonX);

        }
    }
}

//Hope you won