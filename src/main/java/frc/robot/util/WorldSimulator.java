package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.*;
import frc.robot.constants.AdvantageConstants.AdvantageMode;
import frc.robot.event.Emitter;
import frc.robot.subsystems.drivetrain.gyro.GyroSim;
import frc.robot.subsystems.drivetrain.module.ModuleSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

public class WorldSimulator {
    static WorldSimulator instance;

    final SwerveDriveSimulation drivetrain;
    final VisionSystemSim vision;

    private WorldSimulator() {
        assertSim();

        DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofNav2X())
            .withTrackLengthTrackWidth(
                Meters.of(RobotConstants.DriveBase.kTrackWidth),
                Meters.of(RobotConstants.DriveBase.kWheelBase)
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
                Meters.of(RobotConstants.DriveBase.kBumperLength),
                Meters.of(RobotConstants.DriveBase.kBumperWidth)
            )
            .withRobotMass(Kilogram.of(RobotConstants.kMass));

        drivetrain = new SwerveDriveSimulation(config, GameConstants.kInitialPose);
        vision = new VisionSystemSim("caspian");

        vision.addAprilTags(VisionConstants.AprilTag.kLayout);

        SimulatedArena.getInstance().addDriveTrainSimulation(drivetrain);

        Emitter.poseReset.register(this::resetPose);
        Emitter.periodic.register(() -> this.vision.update(this.drivetrain.getSimulatedDriveTrainPose()));

    }

    public static WorldSimulator getInstance() {
        assertSim();

        if (instance == null) {
            instance = new WorldSimulator();
        }
        return instance;
    }

    static void assertSim() {
        assert AdvantageConstants.kCurrentMode == AdvantageMode.Sim : "Used simulation globals in real or replay mode";
    }

    public ModuleSim makeModule(int id) {
        assertSim();
        return new ModuleSim(drivetrain.getModules()[id]);
    }

    public GyroSim makeGyro() {
        assertSim();
        return new GyroSim(drivetrain.getGyroSimulation());
    }

    public void resetPose(Pose2d pose) {
        assertSim();
        drivetrain.setSimulationWorldPose(pose);
        vision.resetRobotPose(pose);
    }

    public void addCamera(PhotonCameraSim camera, Transform3d transform) {
        assertSim();
        vision.addCamera(camera, transform);
    }
}
