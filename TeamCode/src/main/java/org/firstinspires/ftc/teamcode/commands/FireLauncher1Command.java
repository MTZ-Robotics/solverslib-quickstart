package org.firstinspires.ftc.teamcode.commands;// Replace with your team's package name

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Launcher1Subsystem;

public class FireLauncher1Command extends SequentialCommandGroup {
    public FireLauncher1Command(Launcher1Subsystem launcher) {
        // Sequence of commands: push the servo, wait briefly, retract the servo
        addCommands(
                // Push forward instantly
                new InstantCommand(launcher::extendPusher, launcher),

                // Wait for a short duration (adjust this duration as needed in milliseconds)
                new WaitCommand(300),

                // Retract instantly
                new InstantCommand(launcher::retractPusher, launcher)
        );

        // Add requirements for the entire group
        addRequirements(launcher);
    }
}

