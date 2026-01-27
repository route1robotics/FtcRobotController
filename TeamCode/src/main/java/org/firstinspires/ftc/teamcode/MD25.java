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
        robot.initializeVisionPortal((byte) 1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.driveCoast();

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
        boolean prettycolor = false;



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Front Left",             robot.frontLeftPosition());
            telemetry.addData("Front Right",            robot.frontRightPosition());
            telemetry.addData("Back Left",              robot.backLeftPosition());
            telemetry.addData("Back Right",             robot.backRightPosition());
            telemetry.addData("Obelisk ID Code",        robot.getVisibleAprilTagID(21, 22, 23));
            telemetry.addData("motor power (bl)", robot.motorTelebl());
            telemetry.addData("motor power (br)", robot.motorTelebr());
            telemetry.addData("motor power (fl)", robot.motorTelefl());
            telemetry.addData("motor power (fr)", robot.motorTelefr());
            telemetry.addData("left color",             robot.getColorHueLeft());
            telemetry.addData("Launching near?",        robot.LaunchDistanceNear);
            telemetry.addData("AprilTag info",          robot.getAllAprilTagInfo());
            telemetry.addData("launch power",           robot.GetLaunchPower());
            telemetry.update();



            //DRIVER CONTROLLER-----------------------------------------------------
            targetPower = -this.gamepad1.left_stick_y;
            strafePower = (this.gamepad1.left_trigger - this.gamepad1.right_trigger) * 1;//this.gamepad1.left_stick_x;//
            turnPower = -this.gamepad1.right_stick_x * 1;
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


            prettycolor = this.gamepad1.left_stick_button;
            robot.gradient(prettycolor);
            robot.kickstand(this.gamepad1.dpad_down,this.gamepad1.dpad_up);

            //OPERATOR CONTROLLER---------------------------------------------------

            //Triggers / Bumpers / Sticks
            nuclearLaunchCodes = this.gamepad2.right_trigger;
            ferrisRotate = this.gamepad2.left_stick_y;
            robot.ferrisW(ferrisRotate);
            robot.launch(nuclearLaunchCodes);
            robot.ferrisQuarter(this.gamepad2.left_trigger >= 0.1);
            //robot.mandAndWheel(this.gamepad2.left_bumper);
            robot.quickLaunch(this.gamepad2.right_bumper);


            //D-Pad
            opDpadUp = this.gamepad2.dpad_up;
            opDpadDown = this.gamepad2.dpad_down;
            robot.LaunchPowerUp(opDpadUp);
            robot.LaunchPowerDown(opDpadDown);
            robot.setPowerDistance(this.gamepad2.dpad_left, this.gamepad2.dpad_right);

            //Letter Buttons
            mandibleOpen = this.gamepad2.a;
            mandibleClose = this.gamepad2.b;
            mandibleHalf = this.gamepad2.y;

            robot.thisLittleLightOfMine(
                    robot.proxy(),  // color mode (0 = mandible-based)
                    nuclearLaunchCodes,             // gamepad2 right trigger
                    robot.mandState                 // current mandible state
            );

            robot.mandOpen(mandibleOpen);
            robot.mandClose(mandibleClose);
            robot.mandHalf(mandibleHalf);
            buttonX = this.gamepad2.x;

            robot.LaunchKicker(buttonX);
            robot.ReturnKicker(!buttonX);



            robot.lockAndLoadAuto(5, this.gamepad2.right_stick_button, 24, 4);



        }
    }
}

//Hope you won