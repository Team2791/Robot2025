package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.FullIntake;
import frc.robot.commands.intake.Dislodge;
import frc.robot.commands.lift.DispenseOut;
import frc.robot.commands.lift.Elevate;
import frc.robot.commands.lift.ManualElevate;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IOConstants;
import frc.robot.logging.Alerter;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.dispenser.Scoral;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.NavX;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Lift;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.photon.Camera;
import frc.robot.subsystems.photon.Photon;
import frc.robot.util.AdvantageUtil;
import frc.robotreplay.*;
import frc.robotsim.*;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class RobotContainer {
    // controllers
    final CommandXboxController driverctl;
    final CommandXboxController operctl;

    // subsystems
    final Drivetrain drivetrain = new Drivetrain(
        AdvantageUtil.matchReal(NavX::new, () -> WorldSimulator.getInstance().makeGyro(), GyroReplay::new),
        AdvantageUtil.matchReal(
            SwerveModule::new,
            (id) -> WorldSimulator.getInstance().makeModule(id),
            ModuleReplay::new
        )
    );
    final Intake intake = new Intake(
        AdvantageUtil.matchReal(Roller::new, RollerSim::new, RollerReplay::new)
    );
    final Dispenser dispenser = new Dispenser(
        AdvantageUtil.matchReal(Scoral::new, DispenserSim::new, DispenserReplay::new)
    );
    final Elevator elevator = new Elevator(
        AdvantageUtil.matchReal(Lift::new, ElevatorSim::new, ElevatorReplay::new)
    );
    final Photon photon = new Photon(
        drivetrain,
        AdvantageUtil.matchReal(Camera::new, CameraSim::new, CameraReplay::new)
    );

    // autos
    final SendableChooser<Command> autoChooser;

    public RobotContainer() throws IOException, ParseException {
        this.driverctl = new CommandXboxController(IOConstants.Controller.kDriver);
        this.operctl = new CommandXboxController(IOConstants.Controller.kOperator);
        this.autoChooser = AutoBuilder.buildAutoChooser();

        configureBindings();

        SmartDashboard.putData(autoChooser);
        Alerter.getInstance().provideControllers(driverctl, operctl);
    }

    private void configureBindings() {
        // automatically start the intake if near the coral station

        // disable for now. TODO: comps: enable this
        // FullIntake.registerNearby(intake, lift);
        // Elevate.registerRetract(lift);

        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
        driverctl.start().onTrue(new FunctionWrapper(drivetrain::resetGyro));

        driverctl.x().onTrue(new Elevate(elevator, 3));
        driverctl.y().onTrue(new Elevate(elevator, 4));
        driverctl.a().onTrue(new Elevate(elevator, 1));
        driverctl.b().onTrue(new Elevate(elevator, 2));

        // driverctl.rightBumper().toggleOnTrue(new ReefAlign(drivetrain, 1));
        // driverctl.leftBumper().toggleOnTrue(new ReefAlign(drivetrain, -1));

        driverctl.rightTrigger().onTrue(new DispenseOut(dispenser, elevator));
        driverctl.leftTrigger().toggleOnTrue(new FullIntake(dispenser, elevator, intake));

        driverctl.rightStick().onTrue(new Elevate(elevator, 0));

        operctl.axisLessThan(1, -0.1)
            .whileTrue(new SequentialCommandGroup(
                new FunctionWrapper(Elevate::disableRetract),
                new ManualElevate(elevator, true)
            ));

        operctl.axisGreaterThan(1, 0.1)
            .whileTrue(new SequentialCommandGroup(
                new FunctionWrapper(Elevate::disableRetract),
                new ManualElevate(elevator, false)
            ));

        operctl.a().onTrue(new FunctionWrapper(FullIntake::disableNearby));
        operctl.b().onTrue(new FunctionWrapper(Elevate::disableRetract));
        operctl.x().whileTrue(new Dislodge(intake, dispenser));
    }

    public Command getAutonomousCommand() { return autoChooser.getSelected(); }
}
