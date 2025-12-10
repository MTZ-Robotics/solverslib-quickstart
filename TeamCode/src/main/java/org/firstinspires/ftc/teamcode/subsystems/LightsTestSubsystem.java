package org.firstinspires.ftc.teamcode.subsystems;



import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LightsTestSubsystem extends SubsystemBase {
    public RevBlinkinLedDriver blinkinLedDriver;

    public LightsTestSubsystem() {
        // Empty constructor required by SolversLib CommandOpMode
    }

    //@Override
    public void init(HardwareMap hardwareMap) {
        // Initialize hardware
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        // Set an initial pattern or color if needed
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
    }

    // Other subsystems and commands would go here
}
