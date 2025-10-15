package frc.robot.subsystems.drivetrain;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.ControlConstants;
import frc.robot.util.MotorState;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.util.List;

import static edu.wpi.first.units.Units.*;

public class DrivetrainYAGSL extends DrivetrainIO {
    protected final SwerveDrive swerve;

    /**
     * Constructs a Drivetrain using the YAGSL library.
     */
    public DrivetrainYAGSL()  {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                .createSwerveDrive(ControlConstants.Drivetrain.kMaxSpeed);

            this.swerve.swerveController.addSlewRateLimiters(
                new SlewRateLimiter(ControlConstants.RateLimits.kOrthogonal),
                new SlewRateLimiter(ControlConstants.RateLimits.kOrthogonal),
                new SlewRateLimiter(ControlConstants.RateLimits.kRotation));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public Field2d getField2d() {
        return this.swerve.field;
    }

    public void addVisionMeasurement(Pose2d measurement, double timestamp, Matrix<N3, N1> stddevs) {
        this.swerve.addVisionMeasurement(measurement, timestamp, stddevs);
    }

    public void resetPose(Pose2d pose) {
        this.swerve.resetOdometry(pose);
    }

    public void drive(ChassisSpeeds speeds, DriveMode mode) {
        switch (mode) {
            case kFieldRelative -> this.swerve.driveFieldOriented(speeds);
            case kRobotRelative -> this.swerve.drive(speeds);
            case kTeleoperated -> {
                // in this case, drive the robot open-loop
                Rotation2d heading = this.data.pose.getRotation();
                ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading);
                this.swerve.drive(fieldRelative, true, new Translation2d());
            }
        }
    }

    public void update() {
        this.swerve.updateOdometry();

        // [fl, fr, bl, br]
        List<SwerveModule> modules = List.of(this.swerve.getModules());
        DrivetrainData.ModuleState[] states = modules.stream()
                .map(m -> {
                    SwerveMotor drive = m.getDriveMotor();
                    SwerveMotor angle = m.getAngleMotor();
                    SparkMax driveSpark = (SparkMax) drive.getMotor();
                    SparkMax angleSpark = (SparkMax) angle.getMotor();

                    return new DrivetrainData.ModuleState(
                            new MotorState(
                                    driveSpark.getLastError() == REVLibError.kOk,
                                    drive.getPosition(),
                                    drive.getVelocity(),
                                    drive.getVoltage(),
                                    driveSpark.getOutputCurrent()),
                            new MotorState(
                                    angleSpark.getLastError() == REVLibError.kOk,
                                    angle.getPosition(),
                                    angle.getVelocity(),
                                    angle.getVoltage(),
                                    angleSpark.getOutputCurrent()));
                })
                .toArray(DrivetrainData.ModuleState[]::new);

        this.data.frontLeft = states[0];
        this.data.frontRight = states[1];
        this.data.backLeft = states[2];
        this.data.backRight = states[3];

        AHRS gyro = (AHRS) this.swerve.getGyro().getIMU();

        this.data.gyro = new DrivetrainData.GyroState(
                gyro.isConnected(),
                Degrees.of(gyro.getAngle()).in(Radians),
                DegreesPerSecond.of(gyro.getRate()).in(RadiansPerSecond));

        this.data.pose = this.swerve.getPose();
        this.data.velocity = this.swerve.getRobotVelocity();
    }
}
