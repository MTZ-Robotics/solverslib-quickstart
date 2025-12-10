package org.firstinspires.ftc.teamcode.subsystems;// Replace with your team's package name

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Launcher1Subsystem extends SubsystemBase {
    private final DcMotorEx m_flywheelMotor;
    private final Servo m_pusherServo;

    // Define positions for the servo (adjust these values based on your physical robot)
    private static final double PUSHER_RETRACTED_POSITION = 0.45;
    private static final double PUSHER_PUSHED_POSITION = 0.9;

    public Launcher1Subsystem(HardwareMap hardwareMap) {
        m_flywheelMotor = hardwareMap.get(DcMotorEx.class, "m5");
        m_pusherServo = hardwareMap.get(Servo.class, "s12");

        m_pusherServo.setDirection(Servo.Direction.REVERSE);

        // May need to set motor direction, zero power behavior, etc.
        m_flywheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
        m_pusherServo.setPosition(PUSHER_RETRACTED_POSITION);
    }

    // Method to spin the flywheel at a specific power (0.0 to 1.0)
    public void setFlywheelPower(double power) {
        m_flywheelMotor.setPower(power);
    }

    // Method to stop the flywheel
    public void stopFlywheel() {
        m_flywheelMotor.setPower(0.0);
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

