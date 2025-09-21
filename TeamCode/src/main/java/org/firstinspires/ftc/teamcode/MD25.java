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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Frontleft", robot.frontLeftPosition());
            telemetry.addData("Frontright", robot.frontRightPosition());
            telemetry.addData("Backright", robot.backRightPosition());
            telemetry.addData("Backleft", robot.backLeftPosition());
            //telemetry.addData("Claw Left Power",robot.ClawLeftPower());
            //telemetry.addData("Claw Right Power",robot.ClawRightPower());
            telemetry.update();


            //DRIVER CONTROLLER-----------------------------------------------------
            targetPower = -this.gamepad1.left_stick_y;
            strafePower = (this.gamepad1.left_trigger - this.gamepad1.right_trigger) * 0.8;//this.gamepad1.left_stick_x;//
            turnPower = -this.gamepad1.right_stick_x * 0.8;

            robot.driveRobot(targetPower, strafePower, turnPower);

        }
    }
}

//Hope you won