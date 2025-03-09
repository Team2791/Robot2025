package frc.robot.commands.lift;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DispenserConstants;
import frc.robot.subsystems.dispenser.Dispenser;

public class SlowBack extends Command {
    final Dispenser dispenser;
    final Timer timer = new Timer(); // todo: replace with encoder readings

    /**
     * Run the coral back through the dispenser slowly
     *
     * @param dispenser the dispenser subsystem
     */
    public SlowBack(Dispenser dispenser) {
        this.dispenser = dispenser;
        addRequirements(dispenser);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        dispenser.dispense(DispenserConstants.Power.kSlowBack);
    }

    @Override
    public void execute() {
        if (dispenser.getDispenser().broken) timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        dispenser.dispense(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 4.5;
    }
}
