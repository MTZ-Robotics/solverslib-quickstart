package org.firstinspires.ftc.teamcode.subsystems;// Replace with your team's package name

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class DualFlywheelLauncherSubsystem extends SubsystemBase {
    private final DcMotorEx bottomFlywheelMotor;
    private final DcMotorEx topFlywheelMotor;

    private final Servo m_pusherServo;

    // Define positions for the servo (adjust these values based on your physical robot)
    private static final double PUSHER_RETRACTED_POSITION = 0.45;
    private static final double PUSHER_PUSHED_POSITION = 0.9;

    public DualFlywheelLauncherSubsystem(HardwareMap hardwareMap) {
        bottomFlywheelMotor = hardwareMap.get(DcMotorEx.class, "m5");
        topFlywheelMotor = hardwareMap.get(DcMotorEx.class, "m5");

        m_pusherServo = hardwareMap.get(Servo.class, "s12");

        // May need to set motor direction, zero power behavior, etc.
        bottomFlywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        topFlywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_pusherServo.setDirection(Servo.Direction.REVERSE);
        m_pusherServo.setPosition(PUSHER_RETRACTED_POSITION);
    }

    // Method to spin the flywheel at a specific power (0.0 to 1.0)
    public void setFlywheelPower(double power, double ratio) {
        bottomFlywheelMotor.setPower(power);
        topFlywheelMotor.setPower(clamp(power*ratio,0,1));
    }

    // Method to stop the flywheel
    public void stopFlywheel() {
        bottomFlywheelMotor.setPower(0.0);
        topFlywheelMotor.setPower(0.0);
    }

    // Method to push the game piece (instant action)
    public void extendPusher() {
        m_pusherServo.setPosition(PUSHER_PUSHED_POSITION);
    }

    // Method to retract the pusher
    public void retractPusher() {
        m_pusherServo.setPosition(PUSHER_RETRACTED_POSITION);
    }

    @Override
    public void periodic() {
        // This runs constantly. Useful for telemetry or PID loops if needed.
    }
}

