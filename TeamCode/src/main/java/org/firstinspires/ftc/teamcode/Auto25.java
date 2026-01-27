package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto25", group="Auto")


public class Auto25 extends LinearOpMode {
    RobotHardware25 robot = new RobotHardware25(this);
    public int element_zone = 1;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();


        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }


    public void runOpMode() {


        HardwareStart();
        robot.init();
        //robot.initializeVisionPortal((byte) 1);
        String curAuto = "launchNear";
        String curAlly = "red";
        boolean sorting = false;
        int toWait = 0;
        final double ferrisTurnTime = (double) 10.5 / 12;
        int cameraInit = 0;
        boolean LOCK = false;
        int idCode = 0;

        boolean prevLB = false;
        boolean prevRB = false;

        while (!opModeIsActive() && !isStopRequested()) {

            // --- Auto selection ---
            if (gamepad1.a &! LOCK) {
                curAuto = "drive";
                cameraInit = 1;
            }
            else if (gamepad1.b &! LOCK) {
                curAuto = "launchNear";
                cameraInit = 2;
            }
            else if (gamepad1.y &! LOCK) {
                curAuto = "launchFar";
                cameraInit = 1;
            }
            else if (gamepad1.x &! LOCK) {
                robot.initializeVisionPortal((byte) cameraInit);
                LOCK = true;
            }

            // --- Increment / decrement ONCE per press ---
            if (gamepad1.left_bumper && !prevLB) {
                toWait++;
            }
            else if (gamepad1.right_bumper && !prevRB) {
                if (toWait > 0) {
                    toWait --;
                }
            }

            // update previous states
            prevLB = gamepad1.left_bumper;
            prevRB = gamepad1.right_bumper;

            // --- Alliance selection ---
            if (gamepad1.dpad_left) {
                curAlly = "blue";
            } else if (gamepad1.dpad_right) {
                curAlly = "red";
            }

            // Sort?
            if (gamepad1.left_trigger > 0.1) {
                sorting = true;
            } else if (gamepad1.right_trigger > 0.1) {
                sorting = false;
            }

            //teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Auto (Gamepad1 B = Launch from goal, Gamepad1 A = Drive, Gamepad1 Y = Launch from back)", "");
            telemetry.addData("L trigger to sort, R trigger to not", "");
            telemetry.addData("Press Gamepad1 Right/Left Bumper to increment wait time in seconds", "");
            telemetry.addData("Select Alliance (Dpad Left = Blue, Dpad Right = Red", "");
            //telemetry.addData("Press X to LOCK auto", "");

            telemetry.addData("Current Auto Selected", curAuto.toUpperCase());
            telemetry.addData("Sorting?", sorting);
            telemetry.addData("Delay time", toWait);
            telemetry.addData("Current Alliance Selected", curAlly.toUpperCase());
            telemetry.addData("Camera chosen", cameraInit);
            telemetry.addData("Auto locked?", LOCK);

            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();




        if (curAuto.equals("launchNear")) {

            robot.initializeVisionPortal((byte) 1);

            robot.ferrisReset(true);
            robot.setLaunchPower(0.44);
            // tack on a wait time if needed
            robot.waitSafe(toWait);

            // continuously spin launcher
            robot.launch(1);
            // move back
            robot.DriveForward(0.5, 1.35);

            // get up to speed
            if (curAlly.equals("red")) {
                robot.lockAndLoadAuto(2, true, 24, -10);
            }
            else {
                robot.lockAndLoadAuto(2, true, 20, -20);
            }
            robot.waitSafe(3);
            if (!sorting) {
                // launch a ball
                robot.LaunchKicker(true);
                robot.waitSafe(1);
                robot.ReturnKicker(true);
                robot.waitSafe(0.5);
                // close mandibles
                robot.mandClose(true);
                // short pause
                robot.waitSafe(ferrisTurnTime);
                // rotate ferris a quarter turn
                robot.ferrisQuarter(true);
                robot.ferrisQuarter(false);
                // launch a ball
                robot.LaunchKicker(true);
                robot.waitSafe(1);
                robot.ReturnKicker(true);
                robot.waitSafe(1);
                // rotate ferris a quarter turn
                robot.ferrisQuarter(true);
                robot.ferrisQuarter(false);
                robot.LaunchKicker(true);
                robot.waitSafe(1);
                robot.ReturnKicker(true);
            }
            else {
                idCode = robot.getVisibleAprilTagID(21, 22, 23);
                robot.initializeVisionPortal((byte) 1);
                robot.waitSafe(4);
                if (curAlly.equals("red")) {
                    robot.lockAndLoadAuto(2, true, 24, -10);
                }
                else {
                    robot.lockAndLoadAuto(2, true, 20, -20);
                }
                robot.sortingLaunch(idCode);
            }
            robot.waitSafe(0.5);
            // stop launcher
            robot.launch(0);

        } else if (curAuto.equals("launchFar")) {

            robot.waitSafe(toWait);

            idCode = robot.getVisibleAprilTagID(21, 22, 23);

            if (idCode == 21) {
                robot.rgb_light.setPosition(0.34);
            }
            else if (idCode == 22) {
                robot.rgb_light.setPosition(0.45);
            }
            else if (idCode == 23) {
                robot.rgb_light.setPosition(0.67);
            }
            else {
                robot.rgb_light.setPosition(0.28);
            }

            robot.ferrisReset(true);
            robot.setLaunchPower(0.53);
            robot.DriveForward(-0.3, 0.15);
            if (curAlly.equals("red")) {
                robot.lockAndLoadAuto(5, true, 24, -10);
            }
            else {
                robot.lockAndLoadAuto(5, true, 20, -10);
            }
            // continuously spin launcher
            robot.launch(1);
            robot.waitSafe(3);
            if (!sorting) {
                robot.LaunchKicker(true);
                robot.waitSafe(1);
                robot.ReturnKicker(true);
                robot.waitSafe(0.5);
                // close mandibles
                robot.mandClose(true);
                // short pause
                robot.waitSafe(ferrisTurnTime);
                // rotate ferris a quarter turn
                if (curAlly.equals("red")) {
                    robot.lockAndLoadAuto(2, true, 24, -10);
                }
                else {
                    robot.lockAndLoadAuto(2, true, 20, -20);
                }
                robot.ferrisQuarter(true);
                robot.ferrisQuarter(false);
                robot.waitSafe(0.5);
                // launch a ball
                robot.LaunchKicker(true);
                robot.waitSafe(1);
                robot.ReturnKicker(true);
                robot.waitSafe(1);
                // rotate ferris a quarter turn
                if (curAlly.equals("red")) {
                    robot.lockAndLoadAuto(2, true, 24, -10);
                }
                else {
                    robot.lockAndLoadAuto(2, true, 20, -20);
                }
                robot.ferrisQuarter(true);
                robot.ferrisQuarter(false);
                robot.waitSafe(1.5);
                robot.LaunchKicker(true);
                robot.waitSafe(1);
                robot.ReturnKicker(true);
                robot.waitSafe(0.5);
                // rotate ferris a quarter turn
                robot.ferrisQuarter(true);
                robot.ferrisQuarter(false);
            }
            else {
                robot.sortingLaunch(idCode);
            }
            // stop launcher
            robot.launch(0);
            robot.setLaunchPower(0.44);
            if (curAlly.equals("red")) {
                robot.DriveStrafeLeft(0.3, 1);
            }
            else {
                robot.DriveStrafeRight(0.3, 1);
            }
        } else {
            robot.DriveForward(-0.3, 1);
        }
    }
}