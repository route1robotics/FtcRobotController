package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto24_Tri_Bot", group="Auto")

// His full name is Thingamajig Whatchamacallit Route1Robotics von Danville

public class Auto26_Thingamajig extends LinearOpMode {
    RobotHardware_TriBot robot = new RobotHardware_TriBot(this);
    public int element_zone = 1;

    //   private TeamElementSubsystem teamElementDetection = null;

    // boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        //teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }


    public void runOpMode()  {



        HardwareStart();
        robot.init();
        String curAlliance = "red";
        String curLocation = "front";

        while (!opModeIsActive() && !isStopRequested()) {
            robot.waitSafe(0.5);
            //element_zone = teamElementDetection.elementDetection(telemetry);
            //telemetry.addData("getMaxDistance", teamElementDetection.getMaxDistance());
/*
            if (togglePreview && gamepad2.a) {
                togglePreview = false;
                teamElementDetection.toggleAverageZone();
            } else if (!gamepad2.a) {
                togglePreview = true;
            }
*/
            if (gamepad1.x) {
                curAlliance = "blue";
            } else if (gamepad1.b) {
                curAlliance = "red";
            }
            if (gamepad1.dpad_left) {
                curLocation = "left";
            } else if (gamepad1.dpad_right) {
                curLocation = "right";
            } else if (gamepad1.dpad_up) {
                curLocation = "up";
            }
            //teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Select Start Location (Gamepad1 DpadUp = Front, Gamepad1 DpadDown = Back)", ""); //adjust based on rules
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.addData("Current Position Selected : ", curLocation.toUpperCase());

            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();
        // Alliance left
        if (curLocation == "left")
        {
            //robot.ClawClose();
            //robot.ViperStop();
            //robot.rotateviperstop();
            //robot.ViperEncoder(2500);
            robot.DriveForward(0.4, 1);
            robot.DriveTurnLeft(0.4, 114);
            robot.waitSafe(.2);
            robot.DriveForwardSafe(0.4, 1.25,3);
            robot.waitSafe(.2);

            if (robot.INTAKE_WHEELS_INSTALLED) {
                //robot.ClawRelease(); // at the basket
                robot.waitSafe(1.5);
               // robot.ClawStop();
            }
            else if (robot.SIMPLE_CLAW_INSTALLED) {
                //robot.ClawOpen(); // at the basket
                robot.waitSafe(1.5);
            }

            robot.DriveForward(-0.4, 1.2);
            robot.DriveTurnRight(0.4, 121);
            robot.waitSafe(.1);
            robot.DriveForward(0.4, 1.1);

            robot.DriveStrafeLeft(.4,0.2);
            robot.DriveForwardSafe(-0.7,2.3,4);
            robot.waitSafe(.5);

            robot.DriveForward(0.4, 1.5);
            robot.DriveStrafeLeft(.4,0.35);
            robot.DriveForwardSafe(-0.7,2.3,4);



            //robot.DriveForward(0.4,1.5);
            //  robot.DriveStrafeLeft(.4,0.3);
            // robot.DriveForwardSafe(-0.6,2.3,4);
            //  robot.waitSafe(.5);

/*
            robot.DriveForward(0.6,2.1);
            robot.DriveTurnRight(0.4, 40);
            robot.rotateViperEncoder(-400);
            robot.waitSafe(1);
            robot.DriveForwardSafe(0.5, 0.6,.7);
*/
        }
        else if (curLocation == "right")
        // Alliance Right
        {
            //robot.ClawClose();
            //robot.ViperStop();
            //robot.rotateviperstop();
            //robot.ViperEncoder(2400);
            //robot.rotateViperEncoder(-200);
            robot.waitSafe(1.5);
            robot.DriveForward(0.3, 1.3);
            //robot.rotateViperEncoder(-100);
            //robot.MegaViperEncoder(1200);
            robot.waitSafe(2);

            if (robot.INTAKE_WHEELS_INSTALLED) {
                //robot.ClawRelease(); // at the chamber
                robot.waitSafe(1.5);
                //robot.ClawStop();
            }
            else if (robot.SIMPLE_CLAW_INSTALLED) {
                //robot.ClawOpen(); // at the chamber
                robot.waitSafe(1.5);
            }
            // robot.DriveForward(0.4, 0.1);
            // robot.DriveForward(0.4, -0.1);
            //robot.ClawOpen();
            // robot.ViperEncoder(300);

            robot.DriveForward(-0.4, 1);
            robot.waitSafe(1);
            robot.DriveStrafeRight(0.4, 2);
            robot.DriveForwardSafe(-0.4, .4,2);
            // robot.DriveTurnRight(0.4, 90);
            // robot.DriveForward(0.4, 1.5);
            // robot.DriveTurnRight(0.4, 90);
            // robot.DriveForward(0.4, 1.6);

        }
        else if (curLocation == "up")
        //Alliance Up
        {
            // robot.ClawClose();
            //robot.ViperStop();
            //robot.rotateviperstop();
            //robot.ViperEncoder(2000);
            //robot.rotateViperEncoder(-400);
            robot.waitSafe(1.5);
            robot.DriveForward(0.4, 1.3);
            //robot.ViperEncoder(1500);
            robot.waitSafe(2);

            if (robot.INTAKE_WHEELS_INSTALLED) {
                //robot.ClawRelease(); // at the chamber
                robot.waitSafe(1.5);
                //robot.ClawStop();
            }
            else if (robot.SIMPLE_CLAW_INSTALLED) {
                //robot.ClawOpen(); // at the chamber
                robot.waitSafe(1.5);
            }

            //robot.ClawOpen();
            robot.DriveForward(-0.4, 1);
            robot.DriveStrafeLeft(0.4, 2);
            robot.DriveForward(0.4, 1.5);
            robot.DriveTurnRight(.4, 55);
            //robot.rotateViperEncoder(-2000);
            robot.DriveForward(0.4, 0.5);
        }
    }
}