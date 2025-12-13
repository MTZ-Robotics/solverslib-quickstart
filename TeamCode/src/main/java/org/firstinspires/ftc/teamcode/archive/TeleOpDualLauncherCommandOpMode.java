package org.firstinspires.ftc.teamcode.archive;// Replace with your team's package name

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button;

import org.firstinspires.ftc.teamcode.commands.FireDualFlywheelLauncherCommand;
import org.firstinspires.ftc.teamcode.commands.Intake1OnCommand;
import org.firstinspires.ftc.teamcode.commands.SpinUpDualFlywheelLauncherCommand;
import org.firstinspires.ftc.teamcode.subsystems.DualFlywheelLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake1Subsystem;
@Disabled
@TeleOp(name = "Dual Launcher Command TeleOp")
public class TeleOpDualLauncherCommandOpMode extends CommandOpMode {
    // Declare the subsystems and gamepads
    private DualFlywheelLauncherSubsystem m_launcherSubsystem;
    private Intake1Subsystem m_intakeSubsystem;
    private GamepadEx m_operatorGamepad;

    @Override
    public void initialize() {
        // Initialize hardware and subsystems
        m_launcherSubsystem = new DualFlywheelLauncherSubsystem(hardwareMap);
        m_intakeSubsystem = new Intake1Subsystem(hardwareMap);
        m_operatorGamepad = new GamepadEx(gamepad2); // Assuming operator uses gamepad 2

        // Bind commands to buttons

        // When the right bumper is held down, spin up the launcher (e.g., 85% power)
        m_operatorGamepad.getGamepadButton(Button.RIGHT_BUMPER)
                .whileHeld(new SpinUpDualFlywheelLauncherCommand(m_launcherSubsystem, 0.85, 1.25));

        // When the left bumper is pressed once, fire the launcher mechanism
        m_operatorGamepad.getGamepadButton(Button.LEFT_BUMPER)
                .whenPressed(new FireDualFlywheelLauncherCommand(m_launcherSubsystem));

        /*
        // Option 2: Alternatively, you can create new GamepadButton objects explicitly
        GamepadButton rightBumperButton = new GamepadButton(m_operatorGamepad, Button.RIGHT_BUMPER);
        GamepadButton leftBumperButton = new GamepadButton(m_operatorGamepad, Button.LEFT_BUMPER);

        rightBumperButton.whileHeld(new SpinUpLauncherCommand(m_launcherSubsystem, 0.85));
        leftBumperButton.whenPressed(new FireLauncherCommand(m_launcherSubsystem));
        */
        m_operatorGamepad.getGamepadButton(Button.DPAD_DOWN)
                .whenPressed(new Intake1OnCommand(m_intakeSubsystem, 1.0));
        m_operatorGamepad.getGamepadButton(Button.DPAD_UP)
                .whenPressed(new Intake1OnCommand(m_intakeSubsystem,0.0));
    }

    // CommandOpMode handles the loop automatically by calling the scheduler
}
