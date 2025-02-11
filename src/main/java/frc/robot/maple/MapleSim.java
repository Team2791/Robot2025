package frc.robot.maple;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.constants.AdvantageConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.PhysicalConstants;
import frc.robot.constants.AdvantageConstants.AdvantageMode;
import frc.robotsim.drivetrain.GyroSim;
import frc.robotsim.drivetrain.SwerveSim;

public class MapleSim {
	final SwerveDriveSimulation drivetrain;

	static MapleSim instance;

	private MapleSim() {
		if (AdvantageConstants.Modes.kCurrent != AdvantageMode.Sim) {
			this.drivetrain = null;
			return;
		}

		DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
			.withGyro(COTS.ofNav2X())
			.withTrackLengthTrackWidth(
				Meters.of(DriveConstants.Dimensions.kTrackWidth),
				Meters.of(DriveConstants.Dimensions.kWheelBase)
			)
			.withSwerveModule(
				COTS.ofMAXSwerve(
					DCMotor.getNEO(1),
					DCMotor.getNeo550(1),
					COTS.WHEELS.SLS_PRINTED_WHEELS.cof, // TODO: get proper est from design ppl
					switch ((int) ModuleConstants.DriveMotor.kPinionTeeth) {
						case 12 -> 1;
						case 13 -> 2;
						case 14 -> 3;
						default -> throw new IllegalArgumentException("Invalid pinion teeth");
					}
				)
			)
			.withBumperSize(
				Meters.of(DriveConstants.Dimensions.kBumperLength),
				Meters.of(DriveConstants.Dimensions.kBumperWidth)
			)
			.withRobotMass(Kilogram.of(PhysicalConstants.kMass));

		drivetrain = new SwerveDriveSimulation(config, new Pose2d(7.5, 5, new Rotation2d()));
		SimulatedArena.getInstance().addDriveTrainSimulation(drivetrain);
	}

	public static MapleSim getInstance() {
		if (instance == null) {
			instance = new MapleSim();
		}
		return instance;
	}

	public SwerveSim makeModule(int driveId, int turnId, double angularOffset) {
		return new SwerveSim(driveId, turnId, angularOffset, drivetrain.getModules()[switch (driveId) {
			case IOConstants.Drivetrain.Drive.kFrontLeft -> 0;
			case IOConstants.Drivetrain.Drive.kFrontRight -> 1;
			case IOConstants.Drivetrain.Drive.kRearLeft -> 2;
			case IOConstants.Drivetrain.Drive.kRearRight -> 3;
			default -> -1;
		}]);
	}

	public GyroSim makeGyro() {
		return new GyroSim(drivetrain.getGyroSimulation());
	}

	public Pose2d getPose() { return drivetrain.getSimulatedDriveTrainPose(); }
}
