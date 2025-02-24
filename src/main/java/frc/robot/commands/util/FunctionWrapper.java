package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

public class FunctionWrapper extends Command {
    final Runnable function;
    final BooleanSupplier finished;
    final Runnable callback;


    public FunctionWrapper(Runnable function, Subsystem... requirements) {
        this(function, () -> true, requirements);
    }

    public FunctionWrapper(Runnable function, BooleanSupplier finished, Subsystem... requirements) {
        this(function, finished, () -> { }, requirements);
    }

    public FunctionWrapper(Runnable function, Runnable callback, Subsystem... requirements) {
        this(function, () -> true, callback, requirements);
    }

    public FunctionWrapper(Runnable function, BooleanSupplier finished, Runnable callback, Subsystem... requirements) {
        this.function = function;
        this.finished = finished;
        this.callback = callback;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        function.run();
    }

    @Override
    public void end(boolean interrupted) {
        callback.run();
    }

    @Override
    public boolean isFinished() {
        return finished.getAsBoolean();
    }
}
