package org.firstinspires.ftc.teamcode.MTZ;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class MTZPedroTeleOp extends CommandOpMode {
    Follower teleFollower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    boolean runFlywheel = false;
    double topFlywheelRatio = .75;
    double bottomFlywheelDesired = 0.9;
    double triggerToIntake = 0.1;
    double triggerToHold = 0.45;
    double triggerToFire = 0.9;

    /*************************
     * Motor & Servo Variables
     *************************/
    private DcMotor bottomFlywheel;
    private DcMotor topFlywheel;
    private Servo trigger;
    private Servo intake1;

    //CRServo intake1 = new CRServo(hardwareMap, "s6");


    /*******
     * Add Controller Variables & Objects
     ********/
// Assign Variables & Objects for Control Pads
    double triggerValue = 0;
    double aimError = 0;
    double chassisSpeedRatio=0.75;
    InterpLUT triggerServoLUT = new InterpLUT();

    mtzButtonBehavior topFlywheelFaster = new mtzButtonBehavior();
    mtzButtonBehavior bottomFlywheelFaster = new mtzButtonBehavior();
    mtzButtonBehavior topFlywheelSlower = new mtzButtonBehavior();
    mtzButtonBehavior bottomFlywheelSlower = new mtzButtonBehavior();
    mtzButtonBehavior aimBumper = new mtzButtonBehavior();
    mtzButtonBehavior triggerHold = new mtzButtonBehavior();
    mtzButtonBehavior triggerIntake = new mtzButtonBehavior();
    mtzButtonBehavior flywheelOn = new mtzButtonBehavior();
    mtzButtonBehavior flywheelOff = new mtzButtonBehavior();
    double chassisSpeedSlow;
    double chassisSpeedFast;
    double triggerFire = 0;

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

        trigger = hardwareMap.servo.get("s12");
        trigger.setDirection(Servo.Direction.REVERSE);
        //trigger.setPosition(triggerToHold);

        //intake1.setZeroPowerBehavior(CRServo.ZeroPowerBehavior.FLOAT);
        intake1 = hardwareMap.servo.get("s6");

    }

    @Override
    public void run() {
        super.run();

        /* Robot-Centric Drive
        teleFollower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        */

        // Field-Centric Drive

        //teleFollower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);

        /**********
         *
         * Chassis Control
         */
        aimError=Math.toRadians(45);         //-teleFollower.getPose().getHeading();


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

        if(aimBumper.isDown){
            teleFollower.setTeleOpDrive(-gamepad1.left_stick_y*chassisSpeedRatio, -gamepad1.left_stick_x*chassisSpeedRatio, aimError/Math.PI, false);
        } else {
            teleFollower.setTeleOpDrive(-gamepad1.left_stick_y*chassisSpeedRatio, -gamepad1.left_stick_x*chassisSpeedRatio, -gamepad1.right_stick_x*chassisSpeedRatio, false);
        }
        teleFollower.update();

        //triggerValue = triggerServoLUT.get(gamepad1.right_trigger);
        bottomFlywheelFaster.update(gamepad2.dpad_up);
        topFlywheelFaster.update(gamepad2.dpad_left);
        topFlywheelSlower.update(gamepad2.dpad_right);
        bottomFlywheelSlower.update(gamepad2.dpad_down);
        aimBumper.update(gamepad1.left_bumper);
        triggerHold.update(gamepad2.y);
        triggerIntake.update(gamepad2.a);
        flywheelOn.update(gamepad2.b);
        flywheelOff.update(gamepad2.x);
        triggerFire = gamepad2.right_trigger;




        if(topFlywheelFaster.clickedDown){topFlywheelRatio+=.05;}
        if(topFlywheelSlower.clickedDown){topFlywheelRatio-=0.05;}
        if(bottomFlywheelFaster.clickedDown){bottomFlywheelDesired+=0.05;}
        if(bottomFlywheelSlower.clickedDown){bottomFlywheelDesired-=0.05;}
        if(flywheelOn.clickedDown){runFlywheel=true;}
        if(flywheelOff.clickedDown){runFlywheel=false;}
        if (runFlywheel) {
            bottomFlywheel.setPower(bottomFlywheelDesired);
            topFlywheel.setPower(topFlywheelRatio * bottomFlywheelDesired);
            bottomFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        else {
            bottomFlywheel.setPower(0);
            topFlywheel.setPower(0);
        }



        if (triggerIntake.isDown){
            trigger.setPosition(triggerToIntake);
        } else if (triggerFire>0.5) {
            trigger.setPosition(triggerToFire);
        } else {
            trigger.setPosition(triggerToHold);
        }

        /**
         * Set the intake roller to spin if the trigger servo is beyond the hold position on it's way to intake
         */

        /*if(trigger.getPosition()<=triggerToHold) {
            intake1.setPosition(0.0);
        }
        else  {
            intake1.setPosition(0.5);
        }

         */

        telemetryData.addData("X", teleFollower.getPose().getX());
        telemetryData.addData("Y", teleFollower.getPose().getY());
        telemetryData.addData("Heading", teleFollower.getPose().getHeading());
        telemetryData.addData("Top Flywheel Power", topFlywheel.getPower());
        telemetryData.addData("Top Flywheel Ratio", topFlywheelRatio);
        telemetryData.addData("Bottom Flywheel Power", bottomFlywheel.getPower());
        telemetryData.addData("Trigger Value", triggerValue);
        telemetryData.addData("Trigger Position", trigger.getPosition());
        telemetryData.addData("Aim Error", aimError);

        telemetryData.update();
    }
}