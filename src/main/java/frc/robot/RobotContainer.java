package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.FullIntake;
import frc.robot.commands.lift.DispenseOut;
import frc.robot.commands.lift.Elevate;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IOConstants;
import frc.robot.logging.Alerter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.NavX;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Roller;
import frc.robot.subsystems.lift.Dispenser;
import frc.robot.subsystems.lift.Elevator;
import frc.robot.subsystems.lift.Lift;
import frc.robot.util.AdvantageUtil;
import frc.robotreplay.drivetrain.GyroReplay;
import frc.robotreplay.drivetrain.ModuleReplay;
import frc.robotreplay.intake.RollerReplay;
import frc.robotreplay.lift.DispenserReplay;
import frc.robotreplay.lift.ElevatorReplay;
import frc.robotsim.intake.RollerSim;
import frc.robotsim.lift.DispenserSim;
import frc.robotsim.lift.ElevatorSim;
import frc.robotsim.maple.MapleSim;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class RobotContainer {
    // controllers
    final CommandXboxController driverctl;
    final CommandXboxController operctl;

    // subsystems
    final Drivetrain drivetrain = new Drivetrain(
        AdvantageUtil.matchReal(NavX::new, MapleSim.getInstance()::makeGyro, GyroReplay::new),
        AdvantageUtil.matchReal(SwerveModule::new, MapleSim.getInstance()::makeModule, ModuleReplay::new)
    );
    final Lift lift = new Lift(
        AdvantageUtil.matchReal(Dispenser::new, DispenserSim::new, DispenserReplay::new),
        AdvantageUtil.matchReal(Elevator::new, ElevatorSim::new, ElevatorReplay::new)
    );
    final Intake intake = new Intake(
        AdvantageUtil.matchReal(Roller::new, RollerSim::new, RollerReplay::new)
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
        FullIntake.registerNearby(intake, lift);
        // Elevate.registerRetract(lift);

        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
        driverctl.start().onTrue(new FunctionWrapper(drivetrain::resetGyro));
        driverctl.x().onTrue(new Elevate(lift, 1));
        driverctl.y().onTrue(new Elevate(lift, 2));
        driverctl.a().onTrue(new Elevate(lift, 3));
        driverctl.b().onTrue(new Elevate(lift, 4));
        driverctl.rightBumper().onTrue(new Elevate(lift, 0));
        driverctl.leftBumper().toggleOnTrue(new DispenseOut(lift));

        operctl.a().onTrue(new FunctionWrapper(FullIntake::disableNearby));
        operctl.b().onTrue(new FunctionWrapper(Elevate::disableRetract));
    }

    public Command getAutonomousCommand() { return autoChooser.getSelected(); }
}
