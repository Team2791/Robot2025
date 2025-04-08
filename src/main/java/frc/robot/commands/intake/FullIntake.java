package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.dispenser.DispenseIn;
import frc.robot.commands.dispenser.SlowBack;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IntakeConstants;
import frc.robot.event.EventRegistry;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Alerter;

import java.util.function.Consumer;


public class FullIntake extends SequentialCommandGroup {
    private static boolean useNearby = true;

    /**
     * Full intake command.
     * First, run the elevator to L0.
     * Then, run the intake until the beam has broken and move it to the dispenser.
     * At the same time as above, run the dispenser to accept the coral, and pull coral back slowly
     *
     * @param dispenser the dispenser subsystem
     * @param elevator  the elevator subsystem
     * @param intake    the intake subsystem
     */
    public FullIntake(Dispenser dispenser, Elevator elevator, Intake intake) {
        addCommands(
            new Elevate(elevator, 0),
            new ParallelRaceGroup(
                new SequentialCommandGroup(new TakeIn(intake), new ToDispenser(intake, elevator), new WaitCommand(2.0)),
                new SequentialCommandGroup(
                    new DispenseIn(dispenser, elevator),
                    new SlowBack(dispenser)
                ).handleInterrupt(() -> new SequentialCommandGroup(
                    new Dislodge(intake, dispenser),
                    new FullIntake(dispenser, elevator, intake)
                ).schedule())
            ),
            new FunctionWrapper(Alerter.getInstance()::rumble)
        );
    }

    /**
     * Register the full intake command to run when the robot is near the coral dispenser.
     *
     * @param dispenser the dispenser subsystem
     * @param elevator  the elevator subsystem
     * @param intake    the intake subsystem
     */
    public static void registerNearby(Dispenser dispenser, Elevator elevator, Intake intake) {
        FullIntake instance = new FullIntake(dispenser, elevator, intake);
        CommandScheduler scheduler = CommandScheduler.getInstance();

        EventRegistry.stationRange.register(new Consumer<>() {
            boolean wasScheduled = false;

            @Override
            public void accept(Double distance) {
                boolean nearby = distance <= IntakeConstants.Range.kRunIntake;
                boolean auto = DriverStation.isAutonomous();

                if (auto || !useNearby) {
                    return;
                }

                if (nearby && !wasScheduled) {
                    scheduler.schedule(instance);
                    wasScheduled = true;
                }

                if (!nearby && wasScheduled) {
                    scheduler.cancel(instance);
                    wasScheduled = false;
                }
            }
        });
    }

    /**
     * Unregister the full intake command from running when the robot is near the coral dispenser.
     */
    public static void disableNearby() {
        useNearby = false;
    }
}
