package org.firstinspires.ftc.teamcode.MTZ;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.blue;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.red;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

@Autonomous
public class AutoTestRed2 extends OpMode {
    /****************** Modify These Variables ************************/
    public int alliance = red;
    public int startingPosition = 1;
    public double topFlywheelRatio = .45;
    public double bottomFlywheelDesired = 0.8;
    public int fireLoopCountMax = 3;
    public double chassisSpeedMax = 0.1;
    public double timeToFireTrigger = 1.0;
    public double timeToResetTrigger = 2.5;

    double triggerToIntake = 0.1;
    double triggerToHold = 0.5;
    double triggerToFire = 0.9;

    private double redTargetX=140;
    private double redTargetY=140;
    private double blueTargetX=4;
    private double blueTargetY=140;
    private final Pose redScorePose = new Pose(144-50, 16, Math.atan((redTargetY-16)/(redTargetX-(144-50))));
    //private final Pose redScorePose = new Pose(144-50, 16, Math.toRadians(60));
    private final Pose redStartPose1 = new Pose(144-55, 14, Math.toRadians(90));
    private final Pose redStartPose2 = new Pose(121, 126, Math.toRadians(45));
    private final Pose redInterPose = new Pose(144-55, 14, Math.toRadians(90));
    private final Pose redEndPose = new Pose(144-50, 36, Math.toRadians(0));

    //private final Pose blueScorePose = new Pose(48, 108, Math.atan(blueTargetY-108/blueTargetX-48));
    private final Pose blueScorePose = new Pose(43, 144-36, Math.toRadians(135));

    private final Pose blueStartPose1 = new Pose(21.5, 144-14.5, Math.toRadians(135));
    private final Pose blueStartPose2 = new Pose(23, 126, Math.toRadians(135));
    private final Pose blueInterPose = new Pose(48, 108, Math.toRadians(135));
    private final Pose blueEndPose = new Pose(58, 144-24, Math.toRadians(180));


    /************** End of Highly Modifiable Variables **************/


    private Pose startPose = new Pose(23, 0, Math.toRadians(135));
    private Pose interPose = new Pose(48, -16, Math.toRadians(135));
    private Pose endPose = new Pose(58, 24, Math.toRadians(90));
    private Pose scorePose = new Pose(48, 108, Math.toRadians(135));



    @IgnoreConfigurable
    static PoseHistory poseHistory;


    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    public static Follower autoRedFollower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    public enum triggerState {readyToFire, firing, resetting}

    ;
    public triggerState currentTriggerState;

    private int pathState;
    private DcMotor bottomFlywheel;
    private DcMotor topFlywheel;
    private Servo trigger;
    private int fireLoopCount = 1;


    private Path scorePreload;
    private PathChain backUpToShoot, goPark;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        if (alliance == blue) {
            if (startingPosition == 2) {
                startPose = blueStartPose2;
            } else {
                startPose = blueStartPose1;
            }
            interPose = blueInterPose;
            endPose = blueEndPose;
            scorePose = blueScorePose;
        }
        if (alliance == red) {
            if (startingPosition == 2) {
                startPose = redStartPose2;
            } else {
                startPose = redStartPose1;
            }
            interPose = redInterPose;
            endPose = redEndPose;
            scorePose = redScorePose;
        }

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        backUpToShoot = autoRedFollower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .build();
         goPark = autoRedFollower.pathBuilder()
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
               .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                bottomFlywheel.setPower(bottomFlywheelDesired);
                topFlywheel.setPower(topFlywheelRatio * bottomFlywheelDesired);
                actionTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                autoRedFollower.followPath(scorePreload);
                setPathState(2);
                break;
            case 2:

                /* You could check for
                - Follower State: "if(!autoRedFollower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(autoRedFollower.getPose().getX() > 36) {}"
                */

                if (!autoRedFollower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    /***
                     * Fire Trigger 3x
                     */

                    //autoRedFollower.followPath(grabPickup1,true);
                    //if 1st time through firing loop, set the action timer
                    switch (currentTriggerState) {
                        case readyToFire:
                            if (fireLoopCount==1 && actionTimer.getElapsedTimeSeconds()<=5){
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
                                    break;
                                }
                                if(actionTimer.getElapsedTimeSeconds()>=timeToResetTrigger) {
                                    fireLoopCount++;
                                    currentTriggerState= triggerState.readyToFire;
                                }
                                break;

                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!autoRedFollower.isBusy()) {

                    bottomFlywheel.setPower(0);
                    topFlywheel.setPower(0);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    autoRedFollower.followPath(goPark, true);
                    setPathState(4);
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
            autoRedFollower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", autoRedFollower.getPose().getX());
            telemetry.addData("y", autoRedFollower.getPose().getY());
            telemetry.addData("heading", autoRedFollower.getPose().getHeading());
            telemetry.update();
        }

        @Override
        public void init () {
            pathTimer = new Timer();
            actionTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();


            autoRedFollower = Constants.createFollower(hardwareMap);
            buildPaths();


            if (alliance == blue) {
                if (startingPosition == 2) {
                    autoRedFollower.setStartingPose(blueStartPose2);
                } else {
                    autoRedFollower.setStartingPose(blueStartPose1);
                }
            } else if (alliance == red) {
                if (startingPosition == 2) {
                    autoRedFollower.setStartingPose(redStartPose2);
                } else {
                    autoRedFollower.setStartingPose(redStartPose1);
                }
            }


            poseHistory = autoRedFollower.getPoseHistory();

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            bottomFlywheel = hardwareMap.dcMotor.get("m5");
            topFlywheel = hardwareMap.dcMotor.get("m6");
            bottomFlywheel.setDirection(DcMotor.Direction.FORWARD);
            topFlywheel.setDirection(DcMotor.Direction.REVERSE);
            bottomFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            topFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            trigger = hardwareMap.servo.get("s12");
            trigger.setDirection(Servo.Direction.REVERSE);
            trigger.setPosition(triggerToHold);
            currentTriggerState = triggerState.readyToFire;
        }

        @Override
        public void init_loop () {
            telemetryData.addData("This will run in a roughly triangular shape, starting on the bottom-middle point.", getRuntime());
            telemetryData.addData("So, make sure you have enough space to the left, front, and right to run the OpMode.", getRuntime());
            telemetryData.update();
            //autoRedFollower.update();
            //drawCurrent();

        }

        /** Creates the PathChain for the "triangle".*/
        @Override
        public void start () {

            opmodeTimer.resetTimer();
            setPathState(0);


        }

        class Drawing {
            public static final double ROBOT_RADIUS = 9; // woah
            private final FieldManager panelsField = PanelsField.INSTANCE.getField();

            private final Style robotLook = new Style(
                    "", "#3F51B5", 0.0
            );
            private final Style historyLook = new Style(
                    "", "#4CAF50", 0.0
            );

            /**
             * This prepares Panels Field for using Pedro Offsets
             */
            public void init() {
                panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            }

            /**
             * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
             * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
             *
             * @param autoRedFollower Pedro Follower instance.
             */
        /*public void drawDebug(Follower autoRedFollower) {
            if (autoRedFollower.getCurrentPath() != null) {
                drawPath(autoRedFollower.getCurrentPath(), robotLook);
                Pose closestPoint = autoRedFollower.getPointFromPath(autoRedFollower.getCurrentPath().getClosestPointTValue());
                drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), autoRedFollower.getCurrentPath().getHeadingGoal(autoRedFollower.getCurrentPath().getClosestPointTValue())), robotLook);
            }
            drawPoseHistory(autoRedFollower.getPoseHistory(), historyLook);
            drawRobot(autoRedFollower.getPose(), historyLook);

            sendPacket();
        }
         */

            /**
             * This draws a robot at a specified Pose with a specified
             * look. The heading is represented as a line.
             *
             * @param pose  the Pose to draw the robot at
             * @param style the parameters used to draw the robot with
             */
        /*
        public void drawRobot(Pose pose, Style style) {
            if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
                return;
            }

            panelsField.setStyle(style);
            panelsField.moveCursor(pose.getX(), pose.getY());
            panelsField.circle(ROBOT_RADIUS);

            Vector v = pose.getHeadingAsUnitVector();
            v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
            double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
            double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

            panelsField.setStyle(style);
            panelsField.moveCursor(x1, y1);
            panelsField.line(x2, y2);
        }

         */

            /**
             * This draws a robot at a specified Pose. The heading is represented as a line.
             *
             * @param pose the Pose to draw the robot at
             */
        /*
        public void drawRobot(Pose pose) {
            drawRobot(pose, robotLook);
        }

         */

            /**
             * This draws a Path with a specified look.
             *
             * @param path  the Path to draw
             * @param style the parameters used to draw the Path with
             */
        /*
        public void drawPath(Path path, Style style) {
            double[][] points = path.getPanelsDrawingPoints();

            for (int i = 0; i < points[0].length; i++) {
                for (int j = 0; j < points.length; j++) {
                    if (Double.isNaN(points[j][i])) {
                        points[j][i] = 0;
                    }
                }
            }

            panelsField.setStyle(style);
            panelsField.moveCursor(points[0][0], points[0][1]);
            panelsField.line(points[1][0], points[1][1]);
        }

         */

            /**
             * This draws all the Paths in a PathChain with a
             * specified look.
             *
             * @param pathChain the PathChain to draw
             * @param style     the parameters used to draw the PathChain with
             */
        /*
        public void drawPath(PathChain pathChain, Style style) {
            for (int i = 0; i < pathChain.size(); i++) {
                drawPath(pathChain.getPath(i), style);
            }
        }

         */

            /**
             * This draws the pose history of the robot.
             *
             * @param poseTracker the PoseHistory to get the pose history from
             * @param style       the parameters used to draw the pose history with
             */
        /*
        public void drawPoseHistory(PoseHistory poseTracker, Style style) {
            panelsField.setStyle(style);

            int size = poseTracker.getXPositionsArray().length;
            for (int i = 0; i < size - 1; i++) {

                panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
                panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
            }
        }

         */

            /**
             * This draws the pose history of the robot.
             *
             * @param poseTracker the PoseHistory to get the pose history from
             */
        /*
        void drawPoseHistory(PoseHistory poseTracker) {
            drawPoseHistory(poseTracker, historyLook);
        }

         */


            /**
             * This tries to send the current packet to FTControl Panels.
             */
            public void sendPacket() {
                panelsField.update();
            }
        }
   /* public void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }

    public void drawCurrent() {
        Drawing.drawRobot(autoRedFollower.getPose());
    }*/

}



