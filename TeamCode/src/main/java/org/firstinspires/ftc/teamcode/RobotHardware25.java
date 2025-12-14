package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.Math;
import java.util.List;

public class RobotHardware25 {

    private int VIPER_MAX_ENCODER_VALUE = 2600;
    /* Declare OpMode members. */
    private int ROTATE_MAX_ENCODER_VALUE = 5500;
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    public boolean SIMPLE_CLAW_INSTALLED = true;
    public boolean INTAKE_WHEELS_INSTALLED = false;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private Blinker control_Hub = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor front_left = null;
    private DcMotor front_right = null;

    private ColorSensor colorSensor = null;
    private IMU imu = null;
    private DcMotor ferris = null;
    private DcMotor launcher = null;
    private Servo l_mandible = null;
    private Servo r_mandible = null;
    private Servo kicker = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    private int ENCODER_TILE = 8000;
    private int ENCODER_CIRCLE = 8000;
    private boolean enableSlopeEquations = true;
    private double X1 = 0.5;
    private double Y1 = 0.3;
    private double X2 = 0.9;
    private double Y2 = 0.7;
    private double eq1Slope = Y1/X1;
    private double eq2Slope = (Y2 - Y1)/(X2 - X1);
    private double eq2SlopeStrafe = Y2/(X2 - X1);
    private double eq2Inter = Y2 - eq2Slope*X2;  //Intercept
    private double eq2StrafeInter = Y2 - eq2SlopeStrafe*X2;  //Intercept
    private double eq3Slope = (1 - Y2)/(1 - X2);
    private double eq3Inter = 1 - eq3Slope;  //Intercept

    private int[] idCodes = {0,0,0};

    // Robot position relative to start (units = tiles)
    public double globalX = 0.0;
    public double globalY = 0.0;

    // Robot heading (degrees)
    public double headingDeg = 0.0;

    // Store last encoder positions
    double lastFL = 0, lastFR = 0, lastBL = 0, lastBR = 0;



    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware25(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        control_Hub = myOpMode.hardwareMap.get(Blinker.class, "Control Hub");
        back_left    = myOpMode.hardwareMap.get(DcMotor.class, "back left");   // 0
        back_right   = myOpMode.hardwareMap.get(DcMotor.class, "back right");  // 1
        front_left   = myOpMode.hardwareMap.get(DcMotor.class, "front left");  // 2
        front_right  = myOpMode.hardwareMap.get(DcMotor.class, "front right"); // 3
        //colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "color sensor"); // I2C 1
        imu = myOpMode.hardwareMap.get(IMU.class, "imu"); // I2C 0
        ferris = myOpMode.hardwareMap.get(DcMotor.class, "ferris"); // Expansion Hub, motor 0
        launcher = myOpMode.hardwareMap.get(DcMotor.class, "launcher"); // Expansion Hub, motor 1
        l_mandible = myOpMode.hardwareMap.get(Servo.class, "left claw"); // Expansion Hub, Servo 0
        r_mandible = myOpMode.hardwareMap.get(Servo.class, "right claw"); // Control Hub, Servo 5
        kicker =  myOpMode.hardwareMap.get(Servo.class, "kicker"); // Expansion Hub, Servo 1


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastFL = front_left.getCurrentPosition();
        lastFR = front_right.getCurrentPosition();
        lastBL = back_left.getCurrentPosition();
        lastBR = back_right.getCurrentPosition();

        initializeVisionPortal();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagProcessor myAprilTagProcessor;

    public void initializeVisionPortal(){

        //https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/vision_processor_init/vision-processor-init.html
        // Create a new AprilTag Processor Builder object.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        // Optional: set other custom features of the AprilTag Processor (4 are shown here).
        myAprilTagProcessorBuilder.setDrawTagID(true);       // Default: true, for all detections.
        myAprilTagProcessorBuilder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
        myAprilTagProcessorBuilder.setDrawAxes(true);        // Default: false.
        myAprilTagProcessorBuilder.setDrawCubeProjection(false);        // Default: false.

        // Create an AprilTagProcessor by calling build()
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        //https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_init/visionportal-init.html
        // Create a new VisionPortal Builder object.
        myVisionPortalBuilder = new VisionPortal.Builder();

        // Specify the camera to be used for this VisionPortal.
        myVisionPortalBuilder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam"));      // Other choices are: RC phone camera and "switchable camera name".

        // Add the AprilTag Processor to the VisionPortal Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);       // An added Processor is enabled by default.

        // Optional: set other custom features of the VisionPortal (4 are shown here).
        //myVisionPortalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableLiveView(true);// enableCameraMonitoring(true);      // Enable LiveView (RC preview).
        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

        // Create a VisionPortal by calling build()
        myVisionPortal = myVisionPortalBuilder.build();
    }

    public void getVisionPortalData() {
        List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();

        if (detections == null || detections.isEmpty()) {
            idCodes[0] = idCodes[1] = idCodes[2] = 0;
            return;
        }

        for (AprilTagDetection tag : detections) {

            if (tag == null) continue;
            if (tag.metadata == null) continue;
            if (tag.ftcPose == null) continue;

            int id = tag.id;

            if (id == 20) idCodes[0] = 20;
            else if (id == 21 || id == 22 || id == 23) idCodes[1] = id;
            else if (id == 24) idCodes[2] = 24;
        }
    }



    public int procureAprilTagList(int pos) {

        return idCodes[pos];
    }

    // displays a color (purple or green) given a hue range
    public float getColorData() {
        float[] hsvValues = new float[3];
        //Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

        return hsvValues[0];
    }

    // mainly for debugging; gives the hue value in degrees the sensor is currently seeing
    public float getColorHue() {
        float[] hsvValues = new float[3];
        //Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

        return hsvValues[0];
    }


    public float imuRecorder (int num) {
        float[] angles = new float[3];
        angles[0] = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        angles[1] = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        angles[2] = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        return angles[num];
    }


    public void driveCoast() {
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ferris.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    /**
     * Calculates the power adjustments based on controller input.
     * Then sends these power levels to the motors.
     *
     * @param drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left strafe power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is right
     */
    public void driveRobot(double drive, double strafe, double turn) {
        //DRIVER CONTROLLER-----------------------------------------------------
        double targetPower = drive;
        if (enableSlopeEquations)
        {
            if ((targetPower<X1)&&(targetPower>-X1))
                targetPower = eq1Slope*targetPower;
            else if ((targetPower<X2)&&(targetPower>=X1))
                targetPower = eq2Slope*targetPower+eq2Inter;
            else if (targetPower>=X2)
                targetPower = eq3Slope*targetPower+eq3Inter;
            else if (targetPower<-X2)
                targetPower = eq3Slope*targetPower-eq3Inter;
            else
                targetPower = eq2Slope*targetPower-eq2Inter;

            targetPower /= 1;
        }
        else
        {
            targetPower /= 2;
        }

        double strafePower = strafe;
        if (enableSlopeEquations)
        {
            if ((strafePower<X1)&&(strafePower>-X1))
                strafePower = eq1Slope*strafePower;
            else if ((strafePower<X2)&&(strafePower>=X1))
                strafePower = eq2Slope*strafePower+eq2Inter;
            else if (strafePower>=X2)
                strafePower = eq3Slope*strafePower+eq3Inter;
            else if (strafePower<-X2)
                strafePower = eq3Slope*strafePower-eq3Inter;
            else
                strafePower = eq2Slope*strafePower-eq2Inter;
                /*if ((strafePower<X1)&&(strafePower>-X1))
                   strafePower = 0;
                else if ((strafePower<X2)&&(strafePower>=X1))
                    strafePower = eq2SlopeStrafe*strafePower+eq2StrafeInter;
                else if (strafePower>=X2)
                    strafePower = eq3Slope*strafePower+eq3Inter;
                else if (strafePower<-X2)
                    strafePower = eq3Slope*strafePower-eq3Inter;
                else
                    strafePower = eq2SlopeStrafe*strafePower-eq2StrafeInter;*/
        }
        else
        {
            strafePower /=2;
        }

        double turnPower = turn;
        if (enableSlopeEquations)
        {
            if ((turnPower<X1)&&(turnPower>-X1))
                turnPower = eq1Slope*turnPower;
            else if ((turnPower<X2)&&(turnPower>=X1))
                turnPower = eq2Slope*turnPower+eq2Inter;
            else if (turnPower>=X2)
                turnPower = eq3Slope*turnPower+eq3Inter;
            else if (turnPower<-X2)
                turnPower = eq3Slope*turnPower-eq3Inter;
            else
                turnPower = eq2Slope*turnPower-eq2Inter;

            turnPower /= 1;
        }
        else
        {
            turnPower /= 1.5;
        }

        allDrive(targetPower,strafePower,turnPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param targetPower    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafePower    Left/Right driving power (-1.0 to 1.0) +ve is right
     * @param turnPower      Turn Left/Right driving power (-1.0 to 1.0) +ve is right
     */
    public void allDrive(double targetPower, double strafePower, double turnPower)
    {
        back_left.setPower(-targetPower - strafePower + turnPower);
        back_right.setPower(targetPower - strafePower + turnPower);
        front_left.setPower(-targetPower + strafePower + turnPower);
        front_right.setPower(targetPower + strafePower + turnPower);
    }

    double frontLeftPosition() {return front_left.getCurrentPosition();}
    double frontRightPosition() {return front_right.getCurrentPosition();}

    double backRightPosition() {return back_right.getCurrentPosition();}

    double backLeftPosition() {return back_left.getCurrentPosition();}

    void DriveForward(double power, double tiles)
    {
        int c = front_right.getCurrentPosition();
        int W = 0;
        double ticks = tiles*ENCODER_TILE;

        allDrive(power,0,0);
        while (W < ticks)
        {
            if(!myOpMode.opModeIsActive()){return;}
            W = Math.abs(front_right.getCurrentPosition() - c);
        }
        allDrive(0,0,0);
    }

    void DriveForwardSafe(double power, double tiles, double timeout)
    {
        int c = front_right.getCurrentPosition();
        int W = 0;
        double ticks = tiles*ENCODER_TILE;

        allDrive(power,0,0);
        myOpMode.resetRuntime();
        //while (myOpMode.getRuntime() < seconds){if(!myOpMode.opModeIsActive()){return;}}
        while (W < ticks)
        {
            if (myOpMode.getRuntime() > timeout){
                break;
            }
            if(!myOpMode.opModeIsActive()){return;}
            W = Math.abs(front_right.getCurrentPosition() - c);
        }
        allDrive(0,0,0);
    }

    void DriveStrafeLeft(double power, double tiles)
    {
        int c = back_left.getCurrentPosition();
        int W = 0;
        double ticks = tiles*ENCODER_TILE;
        allDrive(0,Math.abs(power),0.0);
        while (W < ticks)
        {
            if(!myOpMode.opModeIsActive()){return;}
            W = Math.abs(back_left.getCurrentPosition() - c);
        }
        allDrive(0,0,0);
    }

    void DriveStrafeRight(double power, double tiles)
    {
        int c = back_left.getCurrentPosition();
        int W = 0;
        double ticks = tiles*ENCODER_TILE;
        allDrive(0,-1*Math.abs(power),0.0);
        while (W < ticks)
        {
            if(!myOpMode.opModeIsActive()){return;}
            W = Math.abs(back_left.getCurrentPosition() - c);
        }
        allDrive(0,0,0);
    }

    void DriveTurnLeft(double power, double degrees)
    {
        int c = back_left.getCurrentPosition();
        int W = 0;
        allDrive(0,0,Math.abs(power));
        double ticks = degrees/360*ENCODER_CIRCLE;
        while (W < ticks)
        {
            if(!myOpMode.opModeIsActive()){return;}
            W = Math.abs(back_left.getCurrentPosition() - c);
        }
        allDrive(0,0,0);
    }

    void DriveTurnRight(double power, double degrees)
    {
        int c = back_left.getCurrentPosition();
        int W = 0;
        allDrive(0,0,-1*Math.abs(power));
        double ticks = degrees/360*ENCODER_CIRCLE;
        while (W < ticks)
        {
            if(!myOpMode.opModeIsActive()){return;}
            W = Math.abs(back_left.getCurrentPosition() - c);
        }
        allDrive(0,0,0);
    }

    public void frontrightgo() {
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setPower(1);
    }
    public void frontleftgo() {
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setPower(1);
    }
    public void backleftgo() {
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setPower(1);
    }
    public void backrightgo() {
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setPower(1);
    }


    // FERRIS WHEEL EXCLUSIVE VARIABLES #====================================================================
    boolean ferrisMoving = false;
    final double ferrisSpeed = 0.3; // Ferris wheel speed

    public void ferris(double stickValue) {

        // --- FORWARD ---
        if (stickValue >= 0.05) {
            ferris.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ferris.setPower(ferrisSpeed);
            return;
        }

        // --- BACKWARD ---
        if (stickValue <= -0.05) {
            ferris.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ferris.setPower(-ferrisSpeed);
            return;
        }

        // --- NO INPUT ---
        ferris.setPower(0);
    }

    public void ferrisReset(boolean resetButton){
        if (resetButton) {
            ferris.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void ferrisQuarter(boolean bumperValue) {
        final int TPR = 1500;
        int quartRot = TPR / 4;
        if (bumperValue) {
            quartRot += ferris.getCurrentPosition();
            ferris.setTargetPosition(quartRot);
            ferris.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ferris.setPower(0.3);
            while (ferris.isBusy()) {

            }
            //ferris.setPower(0);
        }
    }


    public void launch(double triggerValue) {
        if (triggerValue != 0) {
            launcher.setPower(-1*launchPower);
        }
        else {
            launcher.setPower(0);
        }
    }

//    int targetPosition=0;
//    int initialPosition=0;
//    double resetSpeed=0.1;
//    public void ferrisEnc(double triggerValue, boolean bumperPressed, boolean rightBumperPressed) {
//
//        // --- Automated quarter-turn forward ---
//        if (triggerValue > 0.1 && !ferrisMoving) {
//            ferrisMoving = true;
//            int ticksPerQuarterTurn = (int)(ferris.getMotorType().getTicksPerRev() * 0.25);
//            targetPosition = ferris.getCurrentPosition() + ticksPerQuarterTurn;
//
//            ferris.setTargetPosition(targetPosition);
//            ferris.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            ferris.setPower(ferrisSpeed);
//        }
//        // --- Automated quarter-turn backward ---
//        else if (bumperPressed && !rightBumperPressed && !ferrisMoving) {
//            ferrisMoving = true;
//            int ticksPerQuarterTurn = (int)(ferris.getMotorType().getTicksPerRev() * 0.25);
//            targetPosition = ferris.getCurrentPosition() - ticksPerQuarterTurn;
//
//            ferris.setTargetPosition(targetPosition);
//            ferris.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            ferris.setPower(-ferrisSpeed);
//        }
//        // --- Manual backward when both bumpers are held ---
//        else if (bumperPressed && rightBumperPressed) {
//            ferris.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            ferris.setPower(-resetSpeed); // Move backward slowly
//        }
//        // --- When neither both bumpers are held nor other movement commands, stop motor and set new initial position ---
//        else {
//            if (ferris.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//                ferris.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                ferris.setPower(0);
//                initialPosition = ferris.getCurrentPosition(); // Update new initial position
//            }
//            ferrisMoving = false;
//        }
//
//        // --- Stop automated movement when target reached ---
//        if (ferrisMoving && !ferris.isBusy()) {
//            ferris.setPower(0);
//            ferris.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            ferrisMoving = false;
//        }
//    }


    public void mandClose(boolean buttonPress) {

        // Detect rising edge: button just pressed
        if (buttonPress) {
            l_mandible.setPosition(0.0);
            r_mandible.setPosition(0.8);
        }

    }

    public void mandHalf(boolean buttonPress) {

        // Detect rising edge: button just pressed
        if (buttonPress) {
            l_mandible.setPosition(0.15);
            r_mandible.setPosition(0.65);
        }

    }

    public void mandOpen(boolean buttonPress) {

        if (buttonPress) {

            l_mandible.setPosition(0.25);
            r_mandible.setPosition(0.5);
        }
    }

    public void LaunchKicker(boolean buttonPress){
        if (buttonPress){
            kicker.setPosition(1);
        }
    }

    public void ReturnKicker(boolean buttonPress){
        if (buttonPress){
            kicker.setPosition(0.5);
        }
    }

    double launchPower = 0.7;
    boolean previousButtonStatePowerUp = false;
    boolean previousButtonStatePowerDown = false;

    public void setLaunchPower(double power) {
        launchPower = power;
    }

    public void LaunchPowerUp(boolean buttonPress) {
        // Detect rising edge: button just pressed
        if (buttonPress && !previousButtonStatePowerUp) {
            if (launchPower < 1) {
                launchPower += 0.05;
            }
        }
        previousButtonStatePowerUp = buttonPress;
    }

    public void LaunchPowerDown(boolean buttonPress) {
        // Detect rising edge: button just pressed
        if (buttonPress && !previousButtonStatePowerDown) {
            if (launchPower > 0) {
                launchPower -= 0.05;
            }
        }
        previousButtonStatePowerDown = buttonPress;
    }

    public double GetLaunchPower() {
        return launchPower;
    }

    public void waitSafe(double seconds) {
        myOpMode.resetRuntime();
        while (myOpMode.getRuntime() < seconds){if(!myOpMode.opModeIsActive()){return;}}
    }

    @SuppressLint("DefaultLocale")
    public String getAllAprilTagInfo() {
        List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();

        if (detections == null || detections.isEmpty()) {
            return "No AprilTags detected.";
        }

        StringBuilder sb = new StringBuilder();

        for (AprilTagDetection tag : detections) {

            if (tag == null) continue;
            if (tag.ftcPose == null) continue;

            double distanceTiles = tag.ftcPose.range / 24.0;
            double bearingDeg = tag.ftcPose.bearing;

            String direction =
                    (bearingDeg > 2) ? " (right)" :
                            (bearingDeg < -2) ? " (left)" : " (centered)";

            sb.append(String.format(
                    "Tag %d: %.2f tiles away, %.1f°%s\n",
                    tag.id, distanceTiles, bearingDeg, direction
            ));
        }

        if (sb.length() == 0)
            return "Tags detected but no usable pose data.";

        return sb.toString();
    }


    public void lockAndLoad(boolean buttonPress, int target, int offset) {
        if (!buttonPress) return;
        if (!myOpMode.opModeIsActive()) return;

        while (myOpMode.opModeIsActive()) {

            List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();

            if (detections == null || detections.isEmpty()) {
                allDrive(0, 0, 0);
                break;
            }

            AprilTagDetection targetTag = null;

            for (AprilTagDetection tag : detections) {
                if (tag != null && tag.id == target && tag.ftcPose != null) {
                    targetTag = tag;
                    break;
                }
            }

            // No tag visible anymore
            if (targetTag == null) {
                allDrive(0, 0, 0);
                break;
            }

            double bearing = targetTag.ftcPose.bearing;

            // Close enough
            double toleranceL = -1+offset;
            double toleranceR = -1-offset;
            if (Math.abs(bearing) <= toleranceR && toleranceL <= Math.abs(bearing)) {
                allDrive(0, 0, 0);
                break;
            }

            // Smooth proportional turning
            double minPower = 0.12;
            double maxPower = 0.25;

            double turnPower = (Math.abs(bearing) / 15.0) * maxPower;
            turnPower = Math.min(Math.max(turnPower, minPower), maxPower);

            if (bearing < 0) turnPower = -turnPower;

            allDrive(0, 0, turnPower);
        }

        allDrive(0, 0, 0);
    }

    public void updateOdometry() {

        // --- Read current motor encoders ---
        double fl = front_left.getCurrentPosition();
        double fr = front_right.getCurrentPosition();
        double bl = back_left.getCurrentPosition();
        double br = back_right.getCurrentPosition();

        // --- Compute delta since last update ---
        double dFL = fl - lastFL;
        double dFR = fr - lastFR;
        double dBL = bl - lastBL;
        double dBR = br - lastBR;

        // Save for next call
        lastFL = fl;
        lastFR = fr;
        lastBL = bl;
        lastBR = br;

        // --- Convert ticks to robot movement in inches ---
        // Your value: 1 tile = 8000 ticks
        double ticksPerTile = ENCODER_TILE;

        double forward = (-(dFL + dFR + dBL + dBR) / 4.0) / ticksPerTile;   // +Y forward
        double strafe  = ((-dFL + dFR + dBL - dBR) / 4.0) / ticksPerTile;   // +X right

        // --- Get heading from IMU ---
        headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double headingRad = Math.toRadians(headingDeg);

        // --- Convert robot-relative movement into field-relative movement ---
        double fieldDX = forward * Math.sin(headingRad) + strafe * Math.cos(headingRad);
        double fieldDY = forward * Math.cos(headingRad) - strafe * Math.sin(headingRad);

        // --- Update global position ---
        globalX += fieldDX;
        globalY += fieldDY;
    }



}