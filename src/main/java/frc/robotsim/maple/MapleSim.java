package frc.robotsim.maple;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.AdvantageConstants;
import frc.robot.constants.AdvantageConstants.AdvantageMode;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.PhysicalConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robotsim.drivetrain.GyroSim;
import frc.robotsim.drivetrain.ModuleSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

public class MapleSim {
    static MapleSim instance;
    final SwerveDriveSimulation drivetrain;

    private MapleSim() {
        if (AdvantageConstants.kCurrentMode != AdvantageMode.Sim) {
            this.drivetrain = null;
            return;
        }

        DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofNav2X())
            .withTrackLengthTrackWidth(
                Meters.of(PhysicalConstants.Drivetrain.kTrackWidth),
                Meters.of(PhysicalConstants.Drivetrain.kWheelBase)
            )
            .withSwerveModule(
                COTS.ofMAXSwerve(
                    DCMotor.getNEO(1),
                    DCMotor.getNeo550(1),
                    ModuleConstants.Wheel.kFrictionCoefficient,
                    switch ((int) ModuleConstants.DriveMotor.kPinionTeeth) {
                        case 12 -> 1;
                        case 13 -> 2;
                        case 14 -> 3;
                        default -> throw new IllegalArgumentException("Invalid pinion teeth");
                    }
                )
            )
            .withBumperSize(
                Meters.of(PhysicalConstants.Drivetrain.kBumperLength),
                Meters.of(PhysicalConstants.Drivetrain.kBumperWidth)
            )
            .withRobotMass(Kilogram.of(PhysicalConstants.kMass));

        drivetrain = new SwerveDriveSimulation(config, new Pose2d(7.5, 5, new Rotation2d()));

        SimulatedArena.getInstance().addDriveTrainSimulation(drivetrain);
        Emitter.on(new Drivetrain.PoseResetEvent(), this::resetPose);
    }

    public static MapleSim getInstance() {
        if (instance == null) {
            instance = new MapleSim();
        }
        return instance;
    }

    public ModuleSim makeModule(int id) {
        if (AdvantageConstants.kCurrentMode != AdvantageMode.Sim) {
            return null;
        }

        return new ModuleSim(drivetrain.getModules()[id]);
    }

    public GyroSim makeGyro() {
        if (AdvantageConstants.kCurrentMode != AdvantageMode.Sim) {
            return null;
        }

        return new GyroSim(drivetrain.getGyroSimulation());
    }

    public void resetPose(Pose2d pose) {
        if (AdvantageConstants.kCurrentMode != AdvantageMode.Sim) {
            return;
        }

        drivetrain.setSimulationWorldPose(pose);
    }
}
