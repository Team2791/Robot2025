package frc.robot.commands.lift;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DispenserConstants;
import frc.robot.subsystems.lift.Lift;

public class SlowBack extends Command {
    final Lift lift;
    final Timer timer = new Timer(); // todo: replace with encoder readings

    public SlowBack(Lift lift) {
        this.lift = lift;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();

        lift.dispense(DispenserConstants.Power.kSlowBack);
    }

    @Override
    public void execute() {
        if (lift.getDispenser().broken) timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        lift.dispense(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 4.5;
    }
}
