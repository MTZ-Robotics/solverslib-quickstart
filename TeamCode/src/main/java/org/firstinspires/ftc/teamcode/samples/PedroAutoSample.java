package org.firstinspires.ftc.teamcode.samples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class PedroAutoSample extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    double pickupX = 60;
    double pickup1Y = 84;
    double pickup2Y = 60;
    double pickup3Y = 36;
    double pickupLength = 38;

    // Poses
    private final Pose startPose = new Pose(60, 9, Math.toRadians(-90));
    private final Pose scorePose = new Pose(60, 128, Math.toRadians(-35));
    private final Pose pickup1Pose = new Pose(pickupX, pickup1Y, Math.toRadians(180));
    private final Pose pickup1FinishPose = new Pose(pickupX+pickupLength, pickup1Y, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(pickupX, pickup2Y, Math.toRadians(180));
    private final Pose pickup2FinishPose = new Pose(pickupX+pickupLength, pickup2Y, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(pickupX, pickup3Y, Math.toRadians(180));
    private final Pose pickup3FinishPose = new Pose(pickupX+pickupLength, pickup3Y, Math.toRadians(180));
    private final Pose parkPose = new Pose(12, 12, Math.toRadians(180));

    // Path chains
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3;
    private PathChain scorePickup1, scorePickup2, scorePickup3, park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose,pickup1FinishPose))
                .addPath(new BezierLine(pickup1FinishPose,pickup1Pose))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose,pickup2FinishPose))
                .addPath(new BezierLine(pickup2FinishPose,pickup2Pose))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose,pickup3FinishPose))
                .addPath(new BezierLine(pickup3FinishPose,pickup3Pose))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(60, 12), // Control point
                        parkPose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // Mechanism commands - replace these with your actual subsystem commands
    private InstantCommand fireTrigger1() {
        return new InstantCommand(() -> {
            // Example: outtakeSubsystem.openClaw();
        });
    }

    private InstantCommand IntakeOn() {
        return new InstantCommand(() -> {
            // Example: intakeSubsystem.IntakeOn();
        });
    }
    private InstantCommand IntakeOff() {
        return new InstantCommand(() -> {
            // Example: intakeSubsystem.IntakeOff();
        });
    }

    private InstantCommand flywheelOn() {
        return new InstantCommand(() -> {
            // Example: shooter.flywheelOn();
        });
    }

    private InstantCommand flywheelOff() {
        return new InstantCommand(() -> {
            // Example: shooter.flywheelOff();
        });
    }

    @Override
    public void initialize() {
        super.reset();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Create the autonomous command sequence
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                // Score preload
                flywheelOn(),
                new FollowPathCommand(follower, scorePreload),
                fireTrigger1(),
                //new WaitCommand(1000), // Wait 1 second

                // First pickup cycle
                IntakeOn(),
                new FollowPathCommand(follower, grabPickup1).setGlobalMaxPower(0.5), // Sets globalMaxPower to 50% for all future paths
                                                                                     // (unless a custom maxPower is given)
                IntakeOff(),
                new FollowPathCommand(follower, scorePickup1),
                fireTrigger1(),

                // Second pickup cycle
                IntakeOn(),
                new FollowPathCommand(follower, grabPickup2),
                IntakeOff(),
                new FollowPathCommand(follower, scorePickup2, 1.0), // Overrides maxPower to 100% for this path only
                fireTrigger1(),

                // Third pickup cycle

                IntakeOn(),
                new FollowPathCommand(follower, grabPickup3),
                IntakeOff(),
                new FollowPathCommand(follower, scorePickup3),
                fireTrigger1(),

                // Park
                new FollowPathCommand(follower, park, false), // park with holdEnd false
                flywheelOff()
        );

        // Schedule the autonomous sequence
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}