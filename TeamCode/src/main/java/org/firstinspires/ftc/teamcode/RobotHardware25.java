package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

    private RevColorSensorV3 l_color = null;
    private RevColorSensorV3 r_color = null;
    private IMU imu = null;
    private DcMotor ferris = null;
    private DcMotor launcher = null;
    private DcMotor arm_one = null;
    private DcMotor arm_two = null;
    private Servo l_mandible = null;
    private Servo r_mandible = null;
    private Servo kicker = null;
    public Servo rgb_light = null;
    private DistanceSensor proxy = null;

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

    public int[] idCodes = {0,0,0};
    public boolean LaunchDistanceNear = false; //true = NEAR, false = FAR

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
        back_left   = myOpMode.hardwareMap.get(DcMotor.class, "back left");             // 0
        back_right  = myOpMode.hardwareMap.get(DcMotor.class, "back right");            // 1
        front_left  = myOpMode.hardwareMap.get(DcMotor.class, "front left");            // 2
        front_right = myOpMode.hardwareMap.get(DcMotor.class, "front right");           // 3
        l_color     = myOpMode.hardwareMap.get(RevColorSensorV3.class, "left color");   // I2C 1
        r_color     = myOpMode.hardwareMap.get(RevColorSensorV3.class, "right color");  // I2C 1
        imu         = myOpMode.hardwareMap.get(IMU.class, "imu");                       // I2C 0
        ferris      = myOpMode.hardwareMap.get(DcMotor.class, "ferris");                // Expansion Hub, motor 0
        launcher    = myOpMode.hardwareMap.get(DcMotor.class, "launcher");              // Expansion Hub, motor 1
        l_mandible  = myOpMode.hardwareMap.get(Servo.class, "left claw");               // Expansion Hub, Servo 0
        r_mandible  = myOpMode.hardwareMap.get(Servo.class, "right claw");              // Control Hub, Servo 5
        kicker      = myOpMode.hardwareMap.get(Servo.class, "kicker");                  // Expansion Hub, Servo 1
        rgb_light   = myOpMode.hardwareMap.get(Servo.class, "light");                   // Control Hub Servo Port 0
        arm_one     = myOpMode.hardwareMap.get(DcMotor.class, "arm1");
        arm_two     = myOpMode.hardwareMap.get(DcMotor.class, "arm2");


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ferris.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializeVisionPortal(); =============================================================================================================================

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagProcessor myAprilTagProcessor;

    public void initializeVisionPortal(byte cameraNum){

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
        if (cameraNum == 1) {
            myVisionPortalBuilder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam")); // Other choices are: RC phone camera and "switchable camera name".
        }
        else {
            myVisionPortalBuilder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam2"));
        }
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

    // mainly for debugging; gives the hue value in degrees the sensor is currently seeing
    public float getColorHueRight() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(r_color.red(), r_color.green(), r_color.blue(), hsvValues);

        return hsvValues[0];
    }
    public float getColorHueLeft() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(l_color.red(), l_color.green(), l_color.blue(), hsvValues);

        return hsvValues[0];
    }

    public int eliteBallKnowledge() {
        float[] hsvValuesR = new float[3];
        float[] hsvValuesL = new float[3];
        int ballColorR = 0;
        Color.RGBToHSV(r_color.red(), r_color.green(), r_color.blue(), hsvValuesR);
        int ballColorL = 0;
        Color.RGBToHSV(l_color.red(), l_color.green(), l_color.blue(), hsvValuesL);
        int theColor = 0;
        if (hsvValuesR[0] > 80 && hsvValuesR[0] < 160) { // green
            ballColorR = 1;
        }
        else if (hsvValuesR[0] > 280 || hsvValuesR[0] < 40) { // purple
            ballColorR = 2;
        }
        else {
            ballColorR = 0;
        }

        if (hsvValuesL[0] > 100 && hsvValuesL[0] < 140) { // green
            ballColorL = 1;
        }
        else if (hsvValuesL[0] > 290 || hsvValuesL[0] < 20) { // purple
            ballColorL = 2;
        }
        else {
            ballColorL = 0;
        }

        if (ballColorR == 0 && ballColorL == 1) {
            theColor = 1;
        }
        else if (ballColorR == 1 && ballColorL == 0) {
            theColor = 2;
        }
        else if (ballColorR == 1 && ballColorL == 1) {
            theColor = 3;
        }
        else if (ballColorR == 0 && ballColorL == 2) {
            theColor = 4;
        }
        else if (ballColorR == 2 && ballColorL == 0) {
            theColor = 5;
        }
        else if (ballColorR == 2 && ballColorL == 2) {
            theColor = 6;
        }
        else {
            theColor = 0;
        }

        return theColor;
    }

    private static int counter = 0;
    public boolean proxy() {
        double distance = l_color.getDistance(DistanceUnit.CM);

        if (distance <= 5.5) {
            counter += 1;
        }
        else {
            counter = 0;
        }
        if (counter >= 5) {
            return true;
        }
        else {
            return false;
        }
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
    final double ferrisSpeed = 0.214; // Ferris wheel speed

    public void ferrisW(double stickValue) {

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
        final int TPR = 1060;
        int quartRot = TPR / 4;
        if (bumperValue) {
            quartRot += ferris.getCurrentPosition();
            ferris.setTargetPosition(quartRot);
            ferris.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ferris.setPower(0.2);
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

    int mandState = 0;

    public void mandClose(boolean buttonPress) {

        // Detect rising edge: button just pressed
        if (buttonPress) {
            l_mandible.setPosition(0.0);
            r_mandible.setPosition(0.8);
            mandState = 0;
        }

    }

    public void mandHalf(boolean buttonPress) {

        // Detect rising edge: button just pressed
        if (buttonPress) {
            l_mandible.setPosition(0.15);
            r_mandible.setPosition(0.65);
            mandState = 1;
        }

    }

    public void mandOpen(boolean buttonPress) {

        if (buttonPress) {

            l_mandible.setPosition(0.25);
            r_mandible.setPosition(0.5);
            mandState = 2;
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

    double launchPower = 0.45;
    boolean previousButtonStatePowerUp = false;
    boolean previousButtonStatePowerDown = false;

    public void setLaunchPower(double power) {
        launchPower = power;
    }

    public void LaunchPowerUp(boolean buttonPress) {
        // Detect rising edge: button just pressed
        if (buttonPress && !previousButtonStatePowerUp) {
            if (LaunchDistanceNear) {
                if (launchPower < 0.5) {
                    launchPower += 0.01;
                    nearPower = launchPower;
                }
            }
            else {
                if (launchPower < 0.58) {
                    launchPower += 0.01;
                    farPower = launchPower;
                }
            }
        }
        previousButtonStatePowerUp = buttonPress;
    }

    public void LaunchPowerDown(boolean buttonPress) {
        // Detect rising edge: button just pressed
        if (buttonPress && !previousButtonStatePowerDown) {
            if (LaunchDistanceNear) {
                if (launchPower > 0.4) {
                    launchPower -= 0.01;
                    nearPower = launchPower;
                }
            }
            else {
                if (launchPower > 0.48) {
                    launchPower -= 0.01;
                    farPower = launchPower;
                }
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


    public int getVisibleAprilTagID(int... validIDs) {

        List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) {
            return 0;
        }

        for (AprilTagDetection tag : detections) {
            if (tag == null || tag.ftcPose == null) {
                continue;
            }

            for (int id : validIDs) {
                if (tag.id == id) {
                    return tag.id;
                }
            }
        }

        return 0;
    }



    public void lockAndLoad(boolean buttonPress, int target, int offset) {
        boolean lockAndLoadEnable = false;
        if (buttonPress) {
            lockAndLoadEnable = true;
        }

        if (lockAndLoadEnable) {
            List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
            if (detections == null || detections.isEmpty()) {
                allDrive(0, 0, 0);
                lockAndLoadEnable = false;
                return;
            }

            AprilTagDetection targetTag = null;
            for (AprilTagDetection tag : detections) {
                if (tag != null && tag.id == target && tag.ftcPose != null) {
                    targetTag = tag;
                    break;
                }
            }

            if (targetTag == null) {
                allDrive(0, 0, 0);
                lockAndLoadEnable = false;
                return;
            }

            double bearing = targetTag.ftcPose.bearing;
            double tolerance = 5;

            if (Math.abs(bearing)+offset <= tolerance) {
                allDrive(0, 0, 0);   // Locked
                lockAndLoadEnable = false;
                return;
            }

            // Proportional turning
            double minPower = 0.12;
            double maxPower = 0.25;

            double turnPower = (Math.abs(bearing) / 15.0) * maxPower;
            turnPower = Math.min(Math.max(turnPower, minPower), maxPower);

            if (bearing < 0) turnPower = -turnPower;

            allDrive(0, 0, turnPower);
            lockAndLoadEnable = false;
        }
    }


    public void sortingLaunch(int idCode) {
        // GPP
        if (idCode == 21) {

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);

            waitSafe(1);

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);


            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);
        }
        // PPG
        else if (idCode == 23) {

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);
        }
        // PGP
        else {

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);

            mandHalf(true);
            mandHalf(false);
            ferrisQuarter(true);
            ferrisQuarter(false);
            mandClose(true);
            mandClose(false);
            waitSafe(1);

            LaunchKicker(true);
            waitSafe(1);
            ReturnKicker(true);
        }
    }

    // ---COLOR CHECKING LIGHT---

//    public void thisLittleLightOfMine(int color, double triggerValue, int mandibleState) {
//
//        // --- Mandible base color ---
//        double mandibleColor = 0;
//        if (mandibleState == 0) {
//            mandibleColor = 0.28;
//        }
//        else if (mandibleState == 1) {
//            mandibleColor = 0.35;
//        }
//        else {
//            switch (color){
//                case 0:
//                    mandibleColor = 1;
//                    break;
//                case 1:
//                case 2:
//                    mandibleColor = 0.44;
//                    break;
//                case 3:
//                    mandibleColor = 0.5;
//                    break;
//                case 4:
//                case 5:
//                    mandibleColor = 0.67;
//                    break;
//                case 6:
//                    mandibleColor = 0.72;
//                    break;
//            };
//        }
//
//        // --- Auto-aim bearing detection ---
//        double bearing = Double.NaN;
//        double tolerance = 1;   // Same base tolerance used by lockAndLoad()
//
//        List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
//        if (detections != null) {
//            for (AprilTagDetection tag : detections) {
//                if (tag == null || tag.ftcPose == null) continue;
//
//                // Only care about tags 20 and 24
//                if (tag.id == 20 || tag.id == 24) {
//                    bearing = tag.ftcPose.bearing;
//                    break;
//                }
//            }
//        }
//
//        // --- Aim override (LOCK INDICATOR) ---
//        if (triggerValue >= 0.1 && !Double.isNaN(bearing)) {
//
//            if (bearing <= tolerance && bearing >= -tolerance) {
//                // Centered on tag
//                rgb_light.setPosition(0.45);
//            }
//            else if (bearing > tolerance) {
//                // Tag is to the right
//                rgb_light.setPosition(0.38);
//            }
//            else {
//                // Tag is to the left
//                rgb_light.setPosition(0.33);
//            }
//            return;
//        }
//        else {
//            rgb_light.setPosition(mandibleColor);
//        }
//    }

    // PROXIMITY CHECKING LIGHT
    public void thisLittleLightOfMine(boolean proximity, double triggerValue, int mandibleState) {

        // --- Mandible base color ---
        double mandibleColor = 0;
        if (mandibleState == 0) {
            mandibleColor = 0.28;
        }
        else if (mandibleState == 1) {
            mandibleColor = 0.35;
        }
        else {
            if (proximity) {
                mandibleColor = 1;
            }
            else {
                mandibleColor = 0.45;
            }
        }

        // --- Auto-aim bearing detection ---
        double bearing = Double.NaN;
        double tolerance = 0.5;   // Same base tolerance used by lockAndLoad()

        List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
        if (detections != null) {
            for (AprilTagDetection tag : detections) {
                if (tag == null || tag.ftcPose == null) continue;

                // Only care about tags 20 and 24
                if (tag.id == 20 || tag.id == 24) {
                    bearing = tag.ftcPose.bearing;
                    break;
                }
            }
        }

        // --- Aim override (LOCK INDICATOR) ---
        if (triggerValue >= 0.1 && !Double.isNaN(bearing)) {

            if (bearing <= tolerance && bearing >= -tolerance) {
                // Centered on tag
                rgb_light.setPosition(0.45);
            }
            else if (bearing > tolerance) {
                // Tag is to the right
                rgb_light.setPosition(0.38);
            }
            else {
                // Tag is to the left
                rgb_light.setPosition(0.33);
            }
            return;
        }
        else {
            rgb_light.setPosition(mandibleColor);
        }
    }


    public void gradient (boolean button) {
        if (button) {
           for (double pos = 0.278; pos <= 0.722; pos += 0.05) {
               rgb_light.setPosition(pos);
           }
        }
    }

    public void kickstand (boolean down,boolean up) {
        if (down) {
            arm_one.setPower(-0.5);
            arm_two.setPower(-0.5);
        }
        else if (up) {
            arm_one.setPower(0.5);
            arm_two.setPower(0.5);
        }
        else {
            arm_one.setPower(0);
            arm_two.setPower(0);
        }
    }

    public double motorTelebl() {
        return back_left.getPower();
    }
    public double motorTelebr() {
        return back_right.getPower();
    }
    public double motorTelefl() {
        return front_left.getPower();
    }
    public double motorTelefr() {
        return front_right.getPower();
    }


    private double farPower = 0.53;
    private double nearPower = 0.45;

    public void setPowerDistance(boolean LbuttonPress, boolean RbuttonPress) {
        if (LbuttonPress) {
            launchPower = nearPower;
            LaunchDistanceNear = true; // NEAR
        }
        else if (RbuttonPress) {
            launchPower = farPower;
            LaunchDistanceNear = false; // FAR
        }
    }

    public  void sortLight(int idCode) {
        if (idCode == 21) {
            rgb_light.setPosition(0.33);
        }
        else if (idCode == 22) {
            rgb_light.setPosition(0.5);
        }
        else if (idCode == 23) {
            rgb_light.setPosition(0.67);
        }
        else {
            rgb_light.setPosition(0.28);
        }
    }


    public void lockAndLoadAuto(double seconds, boolean buttonPress, int target, int offset) {
        myOpMode.resetRuntime();
        if(!buttonPress) {
            return;
        }
        while (myOpMode.getRuntime() < seconds){
            lockAndLoad(buttonPress, target, offset);
            if(!myOpMode.opModeIsActive()) {
                return;
            }
        }
        allDrive(0, 0, 0);   // Locked

    }

//    public void mandAndWheel(boolean buttonpress) {
//        if (buttonpress) {
//            mandClose(true);
//            mandClose(false);
//
//            myOpMode.resetRuntime();
//            while (myOpMode.getRuntime() < 0.25){
//                if(!myOpMode.opModeIsActive()) {
//                    continue;
//                }
//            }
//
//            ferrisQuarter(true);
//            ferrisQuarter(false);
//
//            myOpMode.resetRuntime();
//            while (myOpMode.getRuntime() < 0.25){
//                if(!myOpMode.opModeIsActive()) {
//                    continue;
//                }
//            }
//
//            mandOpen(true);
//            mandOpen(false);
//        }
//    }

    public void quickLaunch (boolean buttonpress) {
        if (buttonpress) {
            LaunchKicker(true);
            waitSafe(0.5);

            ReturnKicker(true);
            waitSafe(0.5);

            ferrisQuarter(true);
            ferrisQuarter(false);
        }
    }



}