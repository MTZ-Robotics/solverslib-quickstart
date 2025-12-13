package org.firstinspires.ftc.teamcode.archive;// Replace with your team's package name

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.FireLauncher1Command;
import org.firstinspires.ftc.teamcode.commands.SpinUpLauncher1Command;
import org.firstinspires.ftc.teamcode.subsystems.Launcher1Subsystem;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button;
@Disabled
@TeleOp(name = "Launcher TeleOp")
public class TeleOpCommandOpMode extends CommandOpMode {
    // Declare the subsystems and gamepads
    private Launcher1Subsystem m_launcherSubsystem;
    private GamepadEx m_operatorGamepad;

    @Override
    public void initialize() {
        // Initialize hardware and subsystems
        m_launcherSubsystem = new Launcher1Subsystem(hardwareMap);
        m_operatorGamepad = new GamepadEx(gamepad2); // Assuming operator uses gamepad 2

        // Bind commands to buttons

        // When the right bumper is held down, spin up the launcher (e.g., 85% power)
        m_operatorGamepad.getGamepadButton(Button.RIGHT_BUMPER)
                .whileHeld(new SpinUpLauncher1Command(m_launcherSubsystem, 0.85));

        // When the left bumper is pressed once, fire the launcher mechanism
        m_operatorGamepad.getGamepadButton(Button.LEFT_BUMPER)
                .whenPressed(new FireLauncher1Command(m_launcherSubsystem));

        /*
        // Option 2: Alternatively, you can create new GamepadButton objects explicitly
        GamepadButton rightBumperButton = new GamepadButton(m_operatorGamepad, Button.RIGHT_BUMPER);
        GamepadButton leftBumperButton = new GamepadButton(m_operatorGamepad, Button.LEFT_BUMPER);

        rightBumperButton.whileHeld(new SpinUpLauncherCommand(m_launcherSubsystem, 0.85));
        leftBumperButton.whenPressed(new FireLauncherCommand(m_launcherSubsystem));
        */
    }

    // CommandOpMode handles the loop automatically by calling the scheduler
}
