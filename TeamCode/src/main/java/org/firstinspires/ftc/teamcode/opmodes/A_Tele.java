package org.firstinspires.ftc.teamcode.opmodes;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.alliance;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.blue;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.cameraConstants.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.mtzButtonBehavior;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

//@Disabled
@TeleOp
public class A_Tele extends CommandOpMode {
    /*********
     * Modified Left Bumper on 2 to not strafe or distance, but to change flywheel speed
     */
    public final double[] DISTANCE_CAL = {0,55, 65.8, 75,116,205};
    public final double[] TOP_FLYWHEEL_CAL = {.65,.68,.65,.65,.8,.85};
    public final double[] BOTTOM_FLYWHEEL_CAL = {.75,.75,.85,.85,.85,.85};
    InterpLUT topFlywheelLUT;
    InterpLUT bottomFlywheelLUT;
    final double DESIRED_DISTANCE = 66.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5/2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5/2;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3/2;   //  Clip the turn speed to this max value (adjust for your robot)
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int RED_TARGET_TAG_ID = 24;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int BLUE_TARGET_TAG_ID = 20;     // Choose the tag you want to approach or set to -1 for ANY tag.

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    Follower teleFollower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    boolean runFlywheel = false;
    boolean runIntake = false;
    double intakeDesired = .75;
    double topFlywheelDesired = .65;
    double bottomFlywheelDesired = 0.85;
    double triggerToIntake = 0.1;
    double triggerToHold = 0.45;
    double triggerToFire = 0.9;
    double flywheelAdjust = 1.0;
    /*******
     * Add Controller Variables & Objects
     ********/
// Assign Variables & Objects for Control Pads
    double triggerValue = 0;
    double aimError = 0;
    double chassisSpeedRatio=0.75;

    /*************************
     * Motor & Servo Variables
     *************************/
    private DcMotor intake;
    private DcMotor bottomFlywheel;
    private DcMotor topFlywheel;
    private Servo trigger;
    private Servo index1;

    //CRServo index1 = new CRServo(hardwareMap, "s6");



    InterpLUT triggerServoLUT = new InterpLUT();

    mtzButtonBehavior topFlywheelFaster = new mtzButtonBehavior();
    mtzButtonBehavior bottomFlywheelFaster = new mtzButtonBehavior();
    mtzButtonBehavior topFlywheelSlower = new mtzButtonBehavior();
    mtzButtonBehavior bottomFlywheelSlower = new mtzButtonBehavior();
    mtzButtonBehavior aimBumper = new mtzButtonBehavior();
    mtzButtonBehavior triggerHold = new mtzButtonBehavior();
    mtzButtonBehavior triggerIntake = new mtzButtonBehavior();
    mtzButtonBehavior intakeOn = new mtzButtonBehavior();
    mtzButtonBehavior intakeOff = new mtzButtonBehavior();
    mtzButtonBehavior flywheelOn = new mtzButtonBehavior();
    mtzButtonBehavior flywheelOff = new mtzButtonBehavior();
    mtzButtonBehavior redAllianceButton = new mtzButtonBehavior();
    mtzButtonBehavior blueAllianceButton = new mtzButtonBehavior();
    double chassisSpeedSlow;
    double chassisSpeedFast;
    double triggerFire = 0;
    double reverseIntake = 0;

    @Override
    public void initialize() {
        teleFollower = Constants.createFollower(hardwareMap);

        //Red Starting Point

        teleFollower.setStartingPose(new Pose(82, 144-16, Math.toRadians(0)));
        super.reset();

        teleFollower.startTeleopDrive();

        bottomFlywheel = hardwareMap.dcMotor.get("m5");
        topFlywheel = hardwareMap.dcMotor.get("m6");
        bottomFlywheel.setDirection(DcMotor.Direction.FORWARD);
        topFlywheel.setDirection(DcMotor.Direction.REVERSE);
        bottomFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake = hardwareMap.dcMotor.get("m7");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setDirection(DcMotor.Direction.REVERSE);

        trigger = hardwareMap.servo.get("s12");
        trigger.setDirection(Servo.Direction.REVERSE);
        //trigger.setPosition(triggerToHold);

        //index1.setZeroPowerBehavior(CRServo.ZeroPowerBehavior.FLOAT);
        index1 = hardwareMap.servo.get("s6");

        // Initialize the Apriltag Detection process
        initAprilTag();
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

    }

    @Override
    public void run() {
        super.run();

        /* Robot-Centric Drive
        teleFollower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        */

        // Field-Centric Drive

        //teleFollower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);

        InterpLUT topFlywheelLUT = new InterpLUT();
        InterpLUT bottomFlywheelLUT = new InterpLUT();

        topFlywheelLUT .add(DISTANCE_CAL[0], TOP_FLYWHEEL_CAL[0]);
        topFlywheelLUT.add(DISTANCE_CAL[1], TOP_FLYWHEEL_CAL[1]);
        topFlywheelLUT.add(DISTANCE_CAL[2], TOP_FLYWHEEL_CAL[2]);
        topFlywheelLUT.add(DISTANCE_CAL[3], TOP_FLYWHEEL_CAL[3]);
        topFlywheelLUT.add(DISTANCE_CAL[4], TOP_FLYWHEEL_CAL[4]);
        topFlywheelLUT.createLUT();

        bottomFlywheelLUT .add(DISTANCE_CAL[0], BOTTOM_FLYWHEEL_CAL[0]);
        bottomFlywheelLUT.add(DISTANCE_CAL[1], BOTTOM_FLYWHEEL_CAL[1]);
        bottomFlywheelLUT.add(DISTANCE_CAL[2], BOTTOM_FLYWHEEL_CAL[2]);
        bottomFlywheelLUT.add(DISTANCE_CAL[3], BOTTOM_FLYWHEEL_CAL[3]);
        bottomFlywheelLUT.add(DISTANCE_CAL[4], BOTTOM_FLYWHEEL_CAL[4]);
        bottomFlywheelLUT.createLUT();





        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        while (opModeIsActive())
        {




            /**********
             *
             * Chassis Control
             */
            aimError=Math.toRadians(45)-teleFollower.getPose().getHeading();


            /***
             * Chassis Speed Control
             */
            driveConstants.maxPower(1.0);
            chassisSpeedFast=gamepad1.right_trigger;
            chassisSpeedSlow=gamepad1.left_trigger;

            if(chassisSpeedFast>0.5){
                chassisSpeedRatio=1.0;
            } else if(chassisSpeedSlow>0.5){
                chassisSpeedRatio=0.2;
            } else {
                chassisSpeedRatio=0.75;
            }


            //triggerValue = triggerServoLUT.get(gamepad1.right_trigger);
            bottomFlywheelFaster.update(gamepad2.dpad_up);
            topFlywheelFaster.update(gamepad2.dpad_right);
            topFlywheelSlower.update(gamepad2.dpad_left);
            bottomFlywheelSlower.update(gamepad2.dpad_down);
            aimBumper.update(gamepad2.left_bumper);
            intakeOff.update(gamepad2.y);
            intakeOn.update(gamepad2.a);
            flywheelOn.update(gamepad2.b);
            flywheelOff.update(gamepad2.x);
            triggerFire = gamepad2.right_trigger;
            reverseIntake = gamepad2.left_trigger;

            redAllianceButton.update(gamepad1.dpad_left);
            blueAllianceButton.update(gamepad1.dpad_right);




            if(topFlywheelFaster.clickedDown){topFlywheelDesired+=0.025;}
            if(topFlywheelSlower.clickedDown){topFlywheelDesired-=0.025;}
            if(bottomFlywheelFaster.clickedDown){flywheelAdjust+=0.025;}
            if(bottomFlywheelSlower.clickedDown){flywheelAdjust-=0.025;}
            if(flywheelOn.clickedDown){runFlywheel=true;}
            if(flywheelOff.clickedDown){runFlywheel=false;}
            if (runFlywheel) {
                bottomFlywheel.setPower(bottomFlywheelDesired*flywheelAdjust);
                topFlywheel.setPower(topFlywheelDesired*flywheelAdjust);
                bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            else {
                bottomFlywheel.setPower(0);
                topFlywheel.setPower(0);
            }


            if(intakeOn.clickedDown){runIntake=true;runFlywheel=false;}
            if(intakeOff.clickedDown){runIntake=false;runFlywheel=true;}

            if (runIntake) {
                intake.setPower(intakeDesired);
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                index1.setPosition(1);
                trigger.setPosition(triggerToIntake);
            }
            else {
                intake.setPower(-reverseIntake);
                index1.setPosition(.5);
                trigger.setPosition(triggerToHold);
            }



            if (triggerIntake.isDown){
                trigger.setPosition(triggerToIntake);
            } else if (triggerFire>0.5) {
                trigger.setPosition(triggerToFire);
            } else if (triggerHold.isDown){
                trigger.setPosition(triggerToHold);
            }



            if (redAllianceButton.clickedDown){
                alliance=red;
            }
            if (blueAllianceButton.clickedDown){
                alliance=blue;
            }




            if (alliance==red){
                DESIRED_TAG_ID = RED_TARGET_TAG_ID;
            } else {
                DESIRED_TAG_ID = BLUE_TARGET_TAG_ID;
            }

            /**
             * Set the intake roller to spin if the trigger servo is beyond the hold position on it's way to intake
             */

        /*if(trigger.getPosition()<=triggerToHold) {
            index1.setPosition(0.0);
        }
        else  {
            index1.setPosition(0.5);
        }

         */

            telemetryData.addData("Alliance", alliance==red?"Red":"Blue");
            telemetryData.addData("X", teleFollower.getPose().getX());
            telemetryData.addData("Y", teleFollower.getPose().getY());
            telemetryData.addData("Heading", teleFollower.getPose().getHeading());
            telemetryData.addData("Top Flywheel Power", topFlywheel.getPower());
            //telemetryData.addData("Top Flywheel Ratio", topFlywheelDesired);
            telemetryData.addData("Bottom Flywheel Power", bottomFlywheel.getPower());
            telemetryData.addData("Flywheel Adjust", flywheelAdjust);
            telemetryData.addData("Trigger Value", triggerValue);
            telemetryData.addData("Trigger Position", trigger.getPosition());
            telemetryData.addData("Aim Error", aimError);















            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (aimBumper.isDown && targetFound) {


                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);




                /***
                double headingError = desiredTag.ftcPose.bearing;

                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                drive = -gamepad1.left_stick_y * chassisSpeedRatio;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x * chassisSpeedRatio;  // Reduce strafe rate to 50%.


                topFlywheelDesired = topFlywheelLUT.get(desiredTag.ftcPose.range);
                bottomFlywheelDesired = bottomFlywheelLUT.get(desiredTag.ftcPose.range);

                 ****/



                telemetry.addData("Auto Drive & Turn", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else if (aimBumper.isDown) {



                //turn = Range.clip(aimError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                turn   = -gamepad1.right_stick_x  * chassisSpeedRatio *.66;  // Reduce turn rate to 33%.

                drive = -gamepad1.left_stick_y  * chassisSpeedRatio;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  * chassisSpeedRatio;  // Reduce strafe rate to 50%.


                //Need to find distance to target based on follower
                topFlywheelDesired = topFlywheelLUT.get(72);
                bottomFlywheelDesired = bottomFlywheelLUT.get(72);

                //topFlywheelDesired = topFlywheelLUT.get(desiredTag.ftcPose.range);
                //bottomFlywheelDesired = bottomFlywheelLUT.get(desiredTag.ftcPose.range);



                telemetry.addData("Target Not Found", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else if (gamepad2.right_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                //drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                //strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


                topFlywheelDesired = topFlywheelLUT.get(desiredTag.ftcPose.range);
                bottomFlywheelDesired = bottomFlywheelLUT.get(desiredTag.ftcPose.range);

                telemetry.addData("Auto Turn", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y   * chassisSpeedRatio;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x   * chassisSpeedRatio;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x  * chassisSpeedRatio *.66;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            //teleFollower.setTeleOpDrive(-gamepad1.left_stick_y*chassisSpeedRatio, -gamepad1.left_stick_x*chassisSpeedRatio, -gamepad1.right_stick_x*chassisSpeedRatio, false);
            teleFollower.setTeleOpDrive(drive, strafe, turn, false);
            teleFollower.update();
            // Apply desired axes motions to the drivetrain.
            sleep(10);
        }


    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        /**********************
         * Setting these values requires a definition of the axes of the camera and robot:
         * Camera axes:
         * Origin location: Center of the lens
         * Axes orientation: +x right, +y down, +z forward (from camera’s perspective)
         *
         * Robot axes: (this is typical, but you can define this however you want)
         * Origin location: Center of the robot at field height
         * Axes orientation: +x right, +y forward, +z upward
         *
         * Position:
         * If all values are zero (no translation), that implies the camera is at the center of the robot.
         * Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12 inches above the ground -
         * you would need to set the position to (-5, 7, 12).
         *
         * Orientation:
         * If all values are zero (no rotation), that implies the camera is pointing straight up.
         * In most cases, you’ll need to set the pitch to -90 degrees (rotation about the x-axis), meaning the camera is horizontal.
         * Use a yaw of 0 if the camera is pointing forwards, +90 degrees if it’s pointing straight left, -90 degrees for straight right, etc.
         * You can also set the roll to +/-90 degrees if it’s vertical, or 180 degrees if it’s upside-down.
         */
        Position cameraPosition = new Position(DistanceUnit.MM, cameraX, cameraY, cameraZ, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, cameraYaw, cameraPitch, cameraRoll, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}