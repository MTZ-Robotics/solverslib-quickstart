package org.firstinspires.ftc.teamcode.archive;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.blue;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.red;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.alliance;

import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.blueTargetX;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.blueTargetY;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.launchClose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.launchFar;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.launchOutOfTheWay;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.loadingZoneEnd;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.loadingZoneStart;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.parkClose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.parkFar;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.parkMid;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.stack1End;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.stack1Start;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.startClose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.startFar;

import static org.firstinspires.ftc.teamcode.pedroPathing.Positions.tileLength;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

/****
 *
 * This runs in circles to the left
 * during wait for follower to be not busy at the start of stage 2,
 * start to launch path chain
 *
 *
 *
 */
@Disabled
@Autonomous
public class AutoTest6 extends OpMode {
    /****************** Modify These Variables ************************/
    //int alliance = blue;
    public int startingPosition = 1;
    public double topFlywheelDesired = 0.75;
    public double bottomFlywheelDesired = 0.85;
    public double intakeDesired = .75;
    public int fireLoopCountMax = 3;
    public double chassisSpeedMax = 20;

    public double debugDelay = 3;
    public double flywheelStartUpTime = 3.0;
    public double timeToFireTrigger = 1.0;
    public double timeToResetTrigger = 2.5;
    public double timeToDelayStart = 0;
    double triggerToIntake = 0.1;
    double triggerToHold = 0.4;
    double triggerToFire = 0.9;

    /************** End of Highly Modifiable Variables **************/


    /***
     *
     * The coordinates in these poses are just to get them defined.
     *
     * To modify coordinates of where the bot goes, adjust the coordinates in the Positions.java file
     *
     ***/
    private Pose startPose = new Pose(48, 9, Math.toRadians(90));
    private Pose launchPose = new Pose(50, 12, Math.toRadians(110));
    private Pose loadPose1Start = new Pose(tileLength, tileLength/2, Math.toRadians(180));
    private Pose loadPose1End = new Pose(9, tileLength/2, Math.toRadians(180));
    private Pose loadPose2Start = new Pose(40, 1.5*tileLength, Math.toRadians(180));
    private Pose loadPose2End = new Pose(20, 1.5*tileLength, Math.toRadians(180));
    private Pose parkPose = new Pose(2.5*tileLength, 2*tileLength, Math.toRadians(0));

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    //public static Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer, allianceSelectTimer, pathSelectTimer, delaySelectTimer, debugTimer;

    public enum triggerState {readyToFire, firing, resetting}

    ;
    public triggerState currentTriggerState;

    private int pathState;
    public enum pathOptions {noPath, startFarParkMid, startCloseOutOfTheWayParkClose, startFarLoadTwiceIntakeLoadingZoneRepeatParkFar, startFarLoadOnceIntake1ParkMid, startFarLoadTwiceIntakeLoadingZoneIntake1ParkFar, startCloseLoadOnceIntake3ParkClose}
    public String [] chosenPathName = {"noPath", "startFar.ParkMid", "startClose.OutOfTheWay.ParkClose", "startFar.IntakeLoadingZone.Repeat.ParkFar", "startFar.Intake1.ParkMid", "startFar.IntakeLoadingZone.Intake1.ParkFar", "startClose.Intake3.ParkClose"};
    public pathOptions chosenPath = pathOptions.noPath;
    private DcMotor bottomFlywheel;
    private DcMotor topFlywheel;
    private DcMotor intake;
    private Servo trigger;
    private int fireLoopCount = 1;


    private Path scorePreload;
    private PathChain startToLaunch, goPark, launchToReload1, reload1, reload1ToLaunch,launchToReload2,reload2,reload2ToLaunch;

    public void buildPaths() {
        //Start Poses
        if(chosenPath.toString().contains("startClose")){
            startPose = startClose;
        } else {
            startPose = startFar;
        }
        //Park Poses
        if(chosenPath.toString().contains("parkClose")){
            parkPose = parkClose;
        } else if (chosenPath.toString().contains("parkFar")) {
            parkPose = parkFar;
        } else {
            parkPose = parkMid;
        }
        //Launch Poses
        if(chosenPath.toString().contains("OutOfTheWay")){
            launchPose = launchOutOfTheWay;
        } else if (chosenPath.toString().contains("startClose")){
            launchPose = launchClose;
        } else {
            launchPose = launchFar;
        }
        //Load Poses

        if(chosenPath.toString().contains("IntakeLoadingZone")){
            loadPose1Start = loadingZoneStart;
            loadPose1End = loadingZoneEnd;
        }
        if(chosenPath.toString().contains("Repeat")) {
            loadPose2Start = loadPose1Start;
            loadPose2End = loadPose1End;
        }
        if(chosenPath.toString().contains("IntakeLoadingZoneIntake1")){
            loadPose1Start = loadingZoneStart;
            loadPose1End = loadingZoneEnd;
            loadPose2Start = stack1Start;
            loadPose2End = stack1End;
        }

        if(alliance==red) {
            startPose = startPose.mirror();
            parkPose = parkPose.mirror();
            loadPose1Start = loadPose1Start.mirror();
            loadPose1End = loadPose1End.mirror();
            loadPose2Start = loadPose2Start.mirror();
            loadPose2End = loadPose2End.mirror();
            launchPose = launchPose.mirror();
        }
        //Define Paths
        startToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        launchToReload1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, loadPose1Start))
                .setLinearHeadingInterpolation(launchPose.getHeading(), loadPose1Start.getHeading())
                .build();
        reload1 = follower.pathBuilder()
                .addPath(new BezierLine(loadPose1Start, loadPose1End))
                .setLinearHeadingInterpolation(loadPose1Start.getHeading(), loadPose1End.getHeading())
                .build();
        reload1ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(loadPose1End, launchPose))
                .setLinearHeadingInterpolation(loadPose1End.getHeading(), launchPose.getHeading())
                .build();
        launchToReload2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, loadPose2Start))
                .setLinearHeadingInterpolation(launchPose.getHeading(), loadPose2Start.getHeading())
                .build();
        reload2 = follower.pathBuilder()
                .addPath(new BezierLine(loadPose2Start, loadPose2End))
                .setLinearHeadingInterpolation(loadPose2Start.getHeading(), loadPose2End.getHeading())
                .build();
        reload2ToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(loadPose2End, launchPose))
                .setLinearHeadingInterpolation(loadPose2End.getHeading(), launchPose.getHeading())
                .build();
        goPark = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, parkPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (opmodeTimer.getElapsedTimeSeconds() < timeToDelayStart) {
                    break;
                }
                bottomFlywheel.setPower(bottomFlywheelDesired);
                topFlywheel.setPower(topFlywheelDesired);
                bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actionTimer.resetTimer();
                setPathState(1);
                debugTimer.resetTimer();
                break;
            case 1:
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    follower.followPath(startToLaunch);
                    setPathState(2);
                    debugTimer.resetTimer();
                }
                break;
            case 2:
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {

                    /***
                     * Fire Trigger 3x
                     */

                    //follower.followPath(grabPickup1,true);
                    //if 1st time through firing loop, set the action timer
                    switch (currentTriggerState) {
                        case readyToFire:
                            if (fireLoopCount==1 && actionTimer.getElapsedTimeSeconds()<=flywheelStartUpTime){ //give flywheel time to spin up
                                break;
                            }
                            trigger.setPosition(triggerToFire);
                            actionTimer.resetTimer();
                            currentTriggerState = triggerState.firing;
                            break;
                        case firing:
                            if(actionTimer.getElapsedTimeSeconds()>=timeToFireTrigger) {
                                trigger.setPosition(triggerToHold);
                                currentTriggerState= triggerState.resetting;
                                actionTimer.resetTimer();
                            }
                            break;
                        case resetting:
                                //If last time through loop
                                if (fireLoopCount >= fireLoopCountMax) {
                                    setPathState(3);
                                    debugTimer.resetTimer();
                                    break;
                                }
                                if(actionTimer.getElapsedTimeSeconds()>=timeToResetTrigger) {
                                    fireLoopCount++;
                                    currentTriggerState= triggerState.readyToFire;
                                }
                                break;
                    }
                }
                debugTimer.resetTimer();
                break;
            case 3:
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    bottomFlywheel.setPower(0);
                    topFlywheel.setPower(0);
                    if(chosenPath.toString().contains("Load")) {
                        intake.setPower(intakeDesired);
                        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        trigger.setPosition(triggerToIntake);
                        follower.followPath(launchToReload1, true);
                        //Need to find the right number for park state
                        setPathState(4);
                    } else {
                        //Need to find the right number for park state
                        setPathState(11);
                    }
                }
                debugTimer.resetTimer();
                break;
            case 4:
                if (!follower.isBusy()) {
                        follower.followPath(reload1, true);
                        setPathState(5);
                }
                debugTimer.resetTimer();
                break;
            case 5:

                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    intake.setPower(0);
                    trigger.setPosition(triggerToHold);
                    bottomFlywheel.setPower(bottomFlywheelDesired);
                    topFlywheel.setPower(topFlywheelDesired);
                    bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    actionTimer.resetTimer();
                    follower.followPath(reload1ToLaunch, true);
                    //Need to find the right number for park state
                    setPathState(6);
                }
                debugTimer.resetTimer();
                break;
            case 6:
                //Fire the loaded artifacts
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    /***
                     * Fire Trigger 3x
                     */
                    //if 1st time through firing loop, check the action timer
                    switch (currentTriggerState) {
                        case readyToFire:
                            if (fireLoopCount==1 && actionTimer.getElapsedTimeSeconds()<=3){ //give flywheel time to spin up
                                break;
                            }
                            trigger.setPosition(triggerToFire);
                            actionTimer.resetTimer();
                            currentTriggerState = triggerState.firing;
                            break;
                        case firing:
                            if(actionTimer.getElapsedTimeSeconds()>=timeToFireTrigger) {
                                trigger.setPosition(triggerToHold);
                                currentTriggerState= triggerState.resetting;
                                actionTimer.resetTimer();
                            }
                            break;
                        case resetting:
                            //If last time through loop
                            if (fireLoopCount >= fireLoopCountMax) {
                                setPathState(7);
                                debugTimer.resetTimer();
                                break;
                            }
                            if(actionTimer.getElapsedTimeSeconds()>=timeToResetTrigger) {
                                fireLoopCount++;
                                currentTriggerState= triggerState.readyToFire;
                            }
                            break;
                    }
                }
                debugTimer.resetTimer();
                break;
            case 7:
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    bottomFlywheel.setPower(0);
                    topFlywheel.setPower(0);
                    if(chosenPath.toString().contains("LoadTwice")) {
                        intake.setPower(intakeDesired);
                        trigger.setPosition(triggerToIntake);
                        follower.followPath(launchToReload2, true);
                        //Need to find the right number for park state
                        setPathState(8);
                    } else {
                        //Need to find the right number for park state
                        setPathState(11);
                    }
                }
                debugTimer.resetTimer();
                break;
            case 8:
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    follower.followPath(reload1, true);
                    //Need to find the right number for park state
                    setPathState(9);

                }
                debugTimer.resetTimer();
                break;
            case 9:
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    intake.setPower(0);
                    trigger.setPosition(triggerToHold);
                    bottomFlywheel.setPower(bottomFlywheelDesired);
                    topFlywheel.setPower(topFlywheelDesired);
                    follower.followPath(reload1ToLaunch, true);
                    //Need to find the right number for park state
                    setPathState(10);
                }
                debugTimer.resetTimer();
                break;

            case 10:
                //Fire the loaded artifacts
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    /***
                     * Fire Trigger 3x
                     */
                    //if 1st time through firing loop, check the action timer
                    switch (currentTriggerState) {
                        case readyToFire:
                            if (fireLoopCount==1 && actionTimer.getElapsedTimeSeconds()<=3){ //give flywheel time to spin up
                                break;
                            }
                            trigger.setPosition(triggerToFire);
                            actionTimer.resetTimer();
                            currentTriggerState = triggerState.firing;
                            break;
                        case firing:
                            if(actionTimer.getElapsedTimeSeconds()>=timeToFireTrigger) {
                                trigger.setPosition(triggerToHold);
                                currentTriggerState= triggerState.resetting;
                                actionTimer.resetTimer();
                            }
                            break;
                        case resetting:
                            //If last time through loop
                            if (fireLoopCount >= fireLoopCountMax) {
                                setPathState(11);
                                debugTimer.resetTimer();
                                break;
                            }
                            if(actionTimer.getElapsedTimeSeconds()>=timeToResetTrigger) {
                                fireLoopCount++;
                                currentTriggerState= triggerState.readyToFire;
                            }
                            break;
                    }
                }
                debugTimer.resetTimer();
                break;
            case 11:
                //Go Park
                if (!follower.isBusy() && debugTimer.getElapsedTimeSeconds()>=debugDelay) {
                    intake.setPower(0);
                    bottomFlywheel.setPower(0);
                    topFlywheel.setPower(0);
                    follower.followPath(goPark, true);
                    setPathState(12);
                }
                break;
        }

    }
    public void setPathState ( int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }


    /**
         * This runs the OpMode, updating the Follower as well as printing out the debug statements to
         * the Telemetry, as well as the Panels.
         *
         */

        @Override
        public void loop () {

            // These loop the movements of the robot, these must be called continuously in order to work

            driveConstants.maxPower(chassisSpeedMax);
            follower.update();
            autonomousPathUpdate();


            //drawCurrentAndHistory();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Aim Heading", Math.atan((follower.getPose().getY()-blueTargetY)/(follower.getPose().getX()-blueTargetX)));
            telemetry.addData("Aim Distance", Math.sqrt(((follower.getPose().getY()-blueTargetY)*(follower.getPose().getY()-blueTargetY))+((follower.getPose().getX()-blueTargetX)*(follower.getPose().getX()-blueTargetX))));

            telemetry.update();
        }
        @Override
        public void init () {
            pathTimer = new Timer();
            actionTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            debugTimer = new Timer();;
            pathSelectTimer = new Timer();
            pathSelectTimer.resetTimer();
            allianceSelectTimer = new Timer();
            allianceSelectTimer.resetTimer();
            delaySelectTimer = new Timer();
            delaySelectTimer.resetTimer();


            follower = Constants.createFollower(hardwareMap);

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            bottomFlywheel = hardwareMap.dcMotor.get("m5");
            topFlywheel = hardwareMap.dcMotor.get("m6");
            bottomFlywheel.setDirection(DcMotor.Direction.FORWARD);
            topFlywheel.setDirection(DcMotor.Direction.REVERSE);
            bottomFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            topFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake = hardwareMap.dcMotor.get("m7");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            trigger = hardwareMap.servo.get("s12");
            trigger.setDirection(Servo.Direction.REVERSE);
            trigger.setPosition(triggerToHold);
            currentTriggerState = triggerState.readyToFire;
        }
        @Override
        public void init_loop () {
            /*** Alliance Selection ***/
            if(gamepad1.dpad_left && allianceSelectTimer.getElapsedTimeSeconds()>.5) {
                allianceSelectTimer.resetTimer();
                if (alliance == blue) {
                    alliance = red;
                } else {
                    alliance = blue;
                }
            }
            /*** Path Selection ***/
            if(gamepad1.dpad_right && pathSelectTimer.getElapsedTimeSeconds()>.25) {
                pathSelectTimer.resetTimer();
                chosenPath = pathOptions.values()[(chosenPath.ordinal() + 1) % AutoTest_2.pathOptions.values().length];


                if (chosenPath == pathOptions.noPath) {
                    chosenPath = pathOptions.startFarParkMid;
                }
            }
            /*** Start Delay Adjustment ***/
            if(gamepad1.dpad_up && delaySelectTimer.getElapsedTimeSeconds()>.25) {
                delaySelectTimer.resetTimer();
                timeToDelayStart = timeToDelayStart+1;
            }
            if(gamepad1.dpad_down && delaySelectTimer.getElapsedTimeSeconds()>.25) {
                delaySelectTimer.resetTimer();
                timeToDelayStart = timeToDelayStart-1;
            }


            /*** Reset Path Positions With Alliance and Path Chosen ***/

            telemetryData.addData("Select Alliance with the Left Button.", alliance==red?"Red":"Blue");
            telemetryData.addData("Select Path with the Right Button.", chosenPathName[chosenPath.ordinal()]);
            telemetryData.addData("Modify start delay with Up & Down Buttons", timeToDelayStart);

            telemetryData.update();
            follower.update();
            drawCurrent();

        }
        @Override
        public void start () {
            /*** Reset timer and PathState when Start is pressed ***/
            opmodeTimer.resetTimer();
            setPathState(0);
            buildPaths();
        }
}



