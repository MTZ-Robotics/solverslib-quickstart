package org.firstinspires.ftc.teamcode.MTZ;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.InterpLUT;

@TeleOp(name="Tele Shooter Test", group ="Test")
//@Disabled
/****  Test Shooter with speed and trigger controls */

public class TeleShooterTest extends LinearOpMode {
    double topFlywheelRatio = .5;
    double bottomFlywheelDesired = .9;

    /*************************
     * Motor & Servo Variables
     *************************/
    private DcMotor bottomFlywheel;
    private DcMotor topFlywheel;
    private Servo trigger;
    /*******
     * Add Controller Variables & Objects
     ********/
// Assign Variables & Objects for Control Pads
    double triggerValue;
    InterpLUT triggerServoLUT = new InterpLUT();

    mtzButtonBehavior topFlywheelFaster = new mtzButtonBehavior();
    mtzButtonBehavior bottomFlywheelFaster = new mtzButtonBehavior();
    mtzButtonBehavior topFlywheelSlower = new mtzButtonBehavior();
    mtzButtonBehavior bottomFlywheelSlower = new mtzButtonBehavior();
// End of Assignment Mapping
    @Override
    //This is the default opMode call for generically running the opMode in this class directly from the phone without calling it from a super class
    public void runOpMode() throws InterruptedException{

        //Trigger Servo Min Value
        triggerServoLUT.add(0,0);
        triggerServoLUT.add(0.1,0);
        //Trigger Servo Mid Value
        triggerServoLUT.add(.5,.5);
        //Trigger Servo Max Value
        triggerServoLUT.add(.9,1);
        triggerServoLUT.add(1,1);
        triggerServoLUT.createLUT();


        bottomFlywheel = hardwareMap.dcMotor.get("m5");
        topFlywheel = hardwareMap.dcMotor.get("m6");
        bottomFlywheel.setDirection(DcMotor.Direction.REVERSE);
        topFlywheel.setDirection(DcMotor.Direction.FORWARD);
        bottomFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        trigger = hardwareMap.servo.get("s1");
        trigger.setDirection(Servo.Direction.REVERSE);

        /**********************************
         * Do Not set positions on initialize since that counts as controlling the robot
         * and initialize would not be able to happen until the timer starts for driver controlled period
         **********************************/
        /***********************************************
         * Tell driver station that initialization is complete
         **********************************************/
        telemetry.log().clear();
        telemetry.update();
        telemetry.log().add("Initialized. ");
        waitForStart();
        while (opModeIsActive()) {
            /**************************************************************
             *
             * TeleOp Loops From Here to the End of while loop
             *
             * Loops often to see if controls are still the same
             *
             ****************************************************************/

            triggerValue = triggerServoLUT.get(gamepad1.right_trigger);
            bottomFlywheelFaster.update(gamepad1.dpad_up);
            topFlywheelFaster.update(gamepad1.dpad_left);
            topFlywheelSlower.update(gamepad1.dpad_right);
            bottomFlywheelSlower.update(gamepad1.dpad_down);

            if(topFlywheelFaster.clickedDown){topFlywheelRatio=topFlywheelRatio*1.05;}
            if(topFlywheelSlower.clickedDown){topFlywheelRatio=topFlywheelRatio*0.95;}
            if(bottomFlywheelFaster.clickedDown){bottomFlywheelDesired=bottomFlywheelDesired*1.05;}
            if(bottomFlywheelSlower.clickedDown){bottomFlywheelDesired=bottomFlywheelDesired*0.95;}
            bottomFlywheel.setPower(bottomFlywheelDesired);
            topFlywheel.setPower(topFlywheelRatio*bottomFlywheelDesired);


            bottomFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            topFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            trigger.setPosition(triggerValue);

            displayTelemetry();
        }
    }
//Telemetry Methods
    public void displayTelemetry() {
        //telemetry.clearAll();
            telemetry.log().add("Bottom: ", bottomFlywheel.getPower());
            telemetry.log().add("Top: ", topFlywheel.getPower());
            telemetry.log().add("Trigger: ", trigger.getPosition());
            telemetry.update();
    }

    //End of Telemetry Methods
    //End of Class
}
