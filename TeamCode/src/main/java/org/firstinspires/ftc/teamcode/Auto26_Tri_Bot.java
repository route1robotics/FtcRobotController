package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto26_Tri_Bot", group="Auto")

// His full name is Thingamajig Whatchamacallit Route1Robotics von Danville
// Don't worry about the file name it will be fixed

public class Auto26_Tri_Bot extends LinearOpMode {
    RobotHardware25 robot = new RobotHardware25(this);
    public int element_zone = 1;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();


        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }


    public void runOpMode()  {



        HardwareStart();
        robot.init();
        String curAlliance = "red";
        String curLocation = "front";

        while (!opModeIsActive() && !isStopRequested()) {

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

        robot.DriveForward(0.2, 1);
    }
}