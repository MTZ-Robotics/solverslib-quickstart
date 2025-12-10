package org.firstinspires.ftc.teamcode.commands;// Replace with your team's package name

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake1Subsystem;

public class Intake1OnCommand extends CommandBase {
    private final Intake1Subsystem iIntake;
    private final double m_targetPower;

    public Intake1OnCommand(Intake1Subsystem intake, double targetPower) {
        iIntake = intake;
        m_targetPower = targetPower;
        // Declare that this command requires the LauncherSubsystem
        addRequirements(iIntake);
    }

    @Override
    public void initialize() {
        // Start the motor spinning when the command is scheduled
        iIntake.setIntakePower(m_targetPower);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command is canceled or ends
        iIntake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        // This command runs indefinitely until interrupted by another command or button release
        return false;
    }
}

