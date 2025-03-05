package frc.robot.util;

import frc.robot.constants.AdvantageConstants;

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
}
