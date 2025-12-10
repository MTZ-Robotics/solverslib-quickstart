package org.firstinspires.ftc.teamcode.subsystems;// Replace with your team's package name

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake1Subsystem extends SubsystemBase {
    private final DcMotorEx iIntakeMotor;

    public Intake1Subsystem(HardwareMap hardwareMap) {
        iIntakeMotor = hardwareMap.get(DcMotorEx.class, "m7");
        // May need to set motor direction, zero power behavior, etc.
        iIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        iIntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setIntakePower(double power) {
        iIntakeMotor.setPower(power);
    }
    public void stopIntake() {
        iIntakeMotor.setPower(0.0);
    }


    @Override
    public void periodic() {
        // This runs constantly. Useful for telemetry or PID loops if needed.
    }
}

