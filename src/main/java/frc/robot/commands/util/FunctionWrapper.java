package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;

/** 99% of commands are just functions that already exist, change my mind. */
public class FunctionWrapper extends Command {
    final Runnable begin;
    final BooleanSupplier isFinished;
    final Runnable end;

    /**
     * Functional Command Wrapper. Will always finish instantly
     *
     * @param begin     what to run when the command starts
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable begin, Subsystem... requirements) {
        this(begin, () -> true, requirements);
    }

    /**
     * Functional Command Wrapper
     *
     * @param begin     what to run when the command starts
     * @param isFinished     whether the command is finished
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable begin, BooleanSupplier isFinished, Subsystem... requirements) {
        this(begin, isFinished, () -> {}, requirements);
    }

    /**
     * Functional Command Wrapper. Will always finish instantly
     *
     * @param end     what to run when the command ends
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable begin, Runnable end, Subsystem... requirements) {
        this(begin, () -> true, end, requirements);
    }

    /**
     * Functional Command Wrapper
     *
     * @param begin     what to run when the command starts
     * @param isFinished     whether the command is finished
     * @param end     what to run when the command ends
     * @param requirements subsystem requirements
     */
    public FunctionWrapper(Runnable begin, BooleanSupplier isFinished, Runnable end, Subsystem... requirements) {
        this.begin = begin;
        this.isFinished = isFinished;
        this.end = end;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        begin.run();
    }

    @Override
    public void end(boolean interrupted) {
        end.run();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
