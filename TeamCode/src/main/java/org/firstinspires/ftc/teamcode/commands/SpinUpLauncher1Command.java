package org.firstinspires.ftc.teamcode.commands;// Replace with your team's package name

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Launcher1Subsystem;

public class SpinUpLauncher1Command extends CommandBase {
    private final Launcher1Subsystem m_launcher;
    private final double m_targetPower;

    public SpinUpLauncher1Command(Launcher1Subsystem launcher, double targetPower) {
        m_launcher = launcher;
        m_targetPower = targetPower;
        // Declare that this command requires the LauncherSubsystem
        addRequirements(m_launcher);
    }

    @Override
    public void initialize() {
        // Start the motor spinning when the command is scheduled
        m_launcher.setFlywheelPower(m_targetPower);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command is canceled or ends
        m_launcher.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        // This command runs indefinitely until interrupted by another command or button release
        return false;
    }
}

