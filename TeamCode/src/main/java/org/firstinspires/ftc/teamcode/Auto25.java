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
        String curAuto = "launch";
        String curAlly = "red";
        boolean sorting = false;
        int toWait = 0;
        final double ferrisTurnTime = (double) 10.5 / 12;

        boolean prevLB = false;
        boolean prevRB = false;

        while (!opModeIsActive() && !isStopRequested()) {

           // --- Auto selection ---
           if (gamepad1.a) {
               curAuto = "drive";
           }
           else if (gamepad1.b) {
               curAuto = "launchNear";
           }
           else if (gamepad1.y) {
               curAuto = "launchFar";
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
           telemetry.addData("Current Auto Selected", curAuto.toUpperCase());
           telemetry.addData("Sorting?", sorting);
           telemetry.addData("Delay time", toWait);
           telemetry.addData("Current Alliance Selected", curAlly.toUpperCase());

           telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

        if (curAuto.equals("launchNear")) {
            robot.ferrisReset(true);
            robot.setLaunchPower(0.65);
            // tack on a wait time if needed
            robot.waitSafe(toWait);
            // continuously spin launcher
            robot.launch(1);
            // move back
            robot.DriveForward(0.5, 1.25);
            // get up to speed
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
                robot.sortingLaunch(robot.procureAprilTagList(1));
            }
            robot.waitSafe(0.5);
            // stop launcher
            robot.launch(0);

        } else if (curAuto.equals("launchFar")) {
            robot.ferrisReset(true);
            robot.setLaunchPower(0.80);
            robot.DriveForward(-0.3, 0.15);
            if (curAlly.equals("red")) {
                robot.lockAndLoad(true, 24, 4);
            }
            else {
                robot.lockAndLoad(true, 20, 6);
            }
            if (curAlly.equals("red")) {
                robot.lockAndLoad(true, 24, 4);
            }
            else {
                robot.lockAndLoad(true, 20, 6);
            }
            robot.waitSafe(0.5);
            // continuously spin launcher
            robot.launch(1);
            if (curAlly.equals("red")) {
                robot.lockAndLoad(true, 24, 4);
            }
            else {
                robot.lockAndLoad(true, 20, 6);
            }
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
                    robot.lockAndLoad(true, 24, 4);
                } else {
                    robot.lockAndLoad(true, 20, 6);
                }
                if (curAlly.equals("red")) {
                    robot.lockAndLoad(true, 24, 4);
                } else {
                    robot.lockAndLoad(true, 20, 6);
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
                    robot.lockAndLoad(true, 24, 4);
                } else {
                    robot.lockAndLoad(true, 20, 6);
                }
                if (curAlly.equals("red")) {
                    robot.lockAndLoad(true, 24, 4);
                } else {
                    robot.lockAndLoad(true, 20, 6);
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
                robot.sortingLaunch(robot.procureAprilTagList(1));
            }
            // stop launcher
            robot.launch(0);
            robot.setLaunchPower(0.65);
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