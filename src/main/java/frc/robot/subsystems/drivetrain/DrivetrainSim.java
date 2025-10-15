package frc.robot.subsystems.drivetrain;

import frc.robot.util.IterUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import swervelib.SwerveModule;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class DrivetrainSim extends DrivetrainYAGSL {
    /**
     * Constructs a simulation Drivetrain using the YAGSL library.
     */
    public DrivetrainSim() {
        super();

        super.swerve.setHeadingCorrection(false);
        super.swerve.setCosineCompensator(false);
    }

    // Override just this, super method casts to hardware
    public void update() {
        super.swerve.updateOdometry();

        SwerveDriveSimulation driveSim = super.swerve.getMapleSimDrive().orElseThrow();
        GyroSimulation gyro = driveSim.getGyroSimulation();

        // [fl, fr, bl, br]
        List<SwerveModuleSimulation> simModules = List.of(driveSim.getModules());
        List<SwerveModule> libModules = List.of(super.swerve.getModules());

        DrivetrainData.ModuleState[] states = IterUtil.zip(simModules.stream(), libModules.stream())
                .map(modules -> {
                    SwerveModuleSimulation sim = modules.getKey();
                    SwerveModule module = modules.getValue();

                    return new DrivetrainData.ModuleState(
                            new DrivetrainData.MotorState(
                                    true,
                                    module.getPosition().distanceMeters,
                                    module.getState().speedMetersPerSecond,
                                    sim.getDriveMotorAppliedVoltage().in(Volts),
                                    sim.getDriveMotorSupplyCurrent().in(Amps)),
                            new DrivetrainData.MotorState(
                                    true,
                                    module.getPosition().angle.getRadians(),
                                    sim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond),
                                    sim.getSteerMotorAppliedVoltage().in(Volts),
                                    sim.getSteerMotorSupplyCurrent().in(Amps)));
                })
                .toArray(DrivetrainData.ModuleState[]::new);

        this.data.frontLeft = states[0];
        this.data.frontRight = states[1];
        this.data.backLeft = states[2];
        this.data.backRight = states[3];

        this.data.gyro = new DrivetrainData.GyroState(
                true,
                gyro.getGyroReading().getRadians(),
                gyro.getMeasuredAngularVelocity().in(RadiansPerSecond));

        this.data.pose = this.swerve.getPose();
        this.data.velocity = this.swerve.getRobotVelocity();
    }
}
