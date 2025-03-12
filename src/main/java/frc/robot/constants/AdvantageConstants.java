package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AdvantageConstants {
    public static final AdvantageMode kCurrentMode = RobotBase.isReal() ? AdvantageMode.Real : AdvantageMode.Sim;

    public enum AdvantageMode {
        Real, Sim, Replay,
    }

    public static final Command kDeadline = kCurrentMode == AdvantageMode.Sim
        ? new WaitCommand(1.0)
        : new WaitCommand(1e99);
}
