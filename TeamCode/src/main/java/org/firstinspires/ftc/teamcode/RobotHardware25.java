package org.firstinspires.ftc.teamcode;

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

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    private int ENCODER_TILE = 26000;
    private int ENCODER_CIRCLE = 24000;
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
        back_left = myOpMode.hardwareMap.get(DcMotor.class, "back left"); //0
        back_right = myOpMode.hardwareMap.get(DcMotor.class, "back right"); //1
        front_left = myOpMode.hardwareMap.get(DcMotor.class, "front left"); //2
        front_right = myOpMode.hardwareMap.get(DcMotor.class, "front right"); //3
        //colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "color sensor"); // I2C 1
        imu = myOpMode.hardwareMap.get(IMU.class, "imu"); // I2C 0
        ferris = myOpMode.hardwareMap.get(DcMotor.class, "ferris"); // Expansion Hub, motor 0
        launcher = myOpMode.hardwareMap.get(DcMotor.class, "launcher"); // EH, motor 1
        l_mandible = myOpMode.hardwareMap.get(Servo.class, "left claw"); // EH Servo 0
        r_mandible = myOpMode.hardwareMap.get(Servo.class, "right claw"); // CH Servo 5


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

    public void getVisionPortalData(){
        //https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_id_code/apriltag-id-code.html
        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
        int myAprilTagIdCode = 0;                           // ID code of current detection, in for() loop

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();

        // Cycle through through the list and process each AprilTag.
        for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {

            if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                myAprilTagIdCode = myAprilTagDetection.id;
                double myTagPoseRange = myAprilTagDetection.ftcPose.range;
                double myTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                double myTagPoseElevation = myAprilTagDetection.ftcPose.elevation;
                // Now take action based on this tag's ID code, or store info for later action.
                if (myAprilTagIdCode == 21) {
                    idCodes[1] = 21;
                }
                else if (myAprilTagIdCode == 22) {
                    idCodes[1] = 22;
                }
                else if (myAprilTagIdCode == 23) {
                    idCodes[1] = 23;
                }
                else {
                    idCodes[1] = 0;
                }
                if (myAprilTagIdCode == 20) {
                    idCodes[0] = 20;
                }
                else {
                    idCodes[0] = 0;
                }
                if (myAprilTagIdCode == 24) {
                    idCodes[2] = 24;
                }
                else {
                    idCodes[2] = 0;
                }
            }
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
    final double ferrisSpeed = 0.2; // Ferris wheel speed

    public void ferris(double triggerValue, boolean bumperPressed, boolean rightBumperPressed) {

        // --- Automated quarter-turn forward ---
        if (triggerValue > 0.1 && !ferrisMoving) {
            ferris.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ferris.setPower(ferrisSpeed);
            ferrisMoving = true;
        }
        // --- Automated quarter-turn backward ---
        else if (bumperPressed && !ferrisMoving) {
            ferris.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ferris.setPower(-ferrisSpeed);
            ferrisMoving = true;
        }

        // --- Stop automated movement when target reached ---
        if (ferrisMoving && !ferris.isBusy()) {
            ferris.setPower(0);
            ferris.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ferrisMoving = false;
        }
    }

    public void launch(double triggerValue) {
        if (triggerValue != 0) {
            launcher.setPower(-0.8); // ADJUST. On full battery, 1 is too much and is only one well tested.
        }
        else {
            launcher.setPower(0);
        }
    }

    boolean grabbing = false;
    boolean previousButtonStateMandible = false; // Tracks last button state

    public String grab(boolean buttonPress) {
        String state = grabbing ? "open" : "closed";

        // Detect rising edge: button just pressed
        if (buttonPress && !previousButtonStateMandible) {
            grabbing = !grabbing; // Toggle state

            // Move servo depending on new state
            if (grabbing) {
                l_mandible.setPosition(0.4);
                r_mandible.setPosition(0.5);
                state = "open";
            } else {
                l_mandible.setPosition(0.0);
                r_mandible.setPosition(0.8);
                state = "closed";
            }
        }

        previousButtonStateMandible = buttonPress;
        return state;
    }



}










































//           ⣿⣿⣿⣿⣿⣿⡿⠛⣛⣛⣛⣛⣛⣛⣛⣛⣛⣛⡛⠛⠿⠿⢿⣿⣿⣿⣿⣿⣿
//           ⣿⣿⣿⣿⡿⢃⣴⣿⠿⣻⢽⣲⠿⠭⠭⣽⣿⣓⣛⣛⣓⣲⣶⣢⣍⠻⢿⣿⣿
//           ⣿⣿⣿⡿⢁⣾⣿⣵⡫⣪⣷⠿⠿⢿⣷⣹⣿⣿⣿⢲⣾⣿⣾⡽⣿⣷⠈⣿⣿
//           ⣿⣿⠟⠁⣚⣿⣿⠟⡟⠡⠀⠀⠀⠶⣌⠻⣿⣿⠿⠛⠉⠉⠉⢻⣿⣿⠧⡙⢿
//           ⡿⢡⢲⠟⣡⡴⢤⣉⣛⠛⣋⣥⣿⣷⣦⣾⣿⣿⡆⢰⣾⣿⠿⠟⣛⡛⢪⣎⠈
//           ⣧⢸⣸⠐⣛⡁⢦⣍⡛⠿⢿⣛⣿⡍⢩⠽⠿⣿⣿⡦⠉⠻⣷⣶⠇⢻⣟⠟⢀
//           ⣿⣆⠣⢕⣿⣷⡈⠙⠓⠰⣶⣤⣍⠑⠘⠾⠿⠿⣉⣡⡾⠿⠗⡉⡀⠘⣶⢃⣾
//           ⣿⣿⣷⡈⢿⣿⣿⣌⠳⢠⣄⣈⠉⠘⠿⠿⠆⠶⠶⠀⠶⠶⠸⠃⠁⠀⣿⢸⣿
//           ⣿⣿⣿⣷⡌⢻⣿⣿⣧⣌⠻⢿⢃⣷⣶⣤⢀⣀⣀⢀⣀⠀⡀⠀⠀⢸⣿⢸⣿
//           ⣿⣿⣿⣿⣿⣦⡙⠪⣟⠭⣳⢦⣬⣉⣛⠛⠘⠿⠇⠸⠋⠘⣁⣁⣴⣿⣿⢸⣿
//           ⣿⣿⣿⣿⣿⣿⣿⣷⣦⣉⠒⠭⣖⣩⡟⠛⣻⣿⣿⣿⣿⣿⣟⣫⣾⢏⣿⠘⣿
//           ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣤⣍⡛⠿⠿⣶⣶⣿⣿⣿⣿⣿⣾⣿⠟⣰⣿
//           ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣶⣤⣭⣍⣉⣛⣋⣭⣥⣾⣿⣿