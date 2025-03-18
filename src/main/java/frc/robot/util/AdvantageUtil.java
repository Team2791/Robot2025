package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.AdvantageConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public final class AdvantageUtil {
    private AdvantageUtil() { }

    public static <T> T matchReal(
        T real,
        T sim,
        T replay
    ) {
        return switch (AdvantageConstants.kCurrentMode) {
            case Real -> real;
            case Sim -> sim;
            case Replay -> replay;
        };
    }

    public static <T> T matchReal(
        Supplier<T> real,
        Supplier<T> sim,
        Supplier<T> replay
    ) {
        return AdvantageUtil.<Supplier<T>>matchReal(real, sim, replay).get();
    }

    public static void logActiveCommand(Subsystem self) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Command command = scheduler.requiring(self);

        if (command != null) {
            Logger.recordOutput(self.getName() + "/ActiveCommand", command.getName());
        } else {
            Logger.recordOutput(self.getName() + "/ActiveCommand", "None");
        }
    }
}
