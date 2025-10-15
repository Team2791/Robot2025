package frc.robot.subsystems.drivetrain;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IOConstants;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.AllianceUtil;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
    private final DrivetrainIO io;

    public Drivetrain(DrivetrainIO io) {
        this.io = io;

        AutoLogOutputManager.addObject(this);
        HAL.report(
                FRCNetComm.tResourceType.kResourceType_RobotDrive,
                FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);
    }

    /**
     * Reset the robot's pose
     * @param pose the new pose
     */
    public void resetPose(Pose2d pose) {
        this.io.resetPose(pose);
    }

    /**
     * Get the robot's pose
     * @return the robot's pose
     */
    public Pose2d getPose() {
        return this.io.data.pose;
    }

    /**
     * Swerve drive control
     * @param speeds The desired speeds for the robot to move at.
     * @param mode   The drive mode to use.
     */
    public void drive(ChassisSpeeds speeds, DrivetrainIO.DriveMode mode) {
        io.drive(speeds, mode);
    }

    /**
     * Controller-based swerve drive control
     * @param controller The controller to get input from.
     */
    public void drive(CommandXboxController controller) {
        // [-1..1] inputs w/ deadband
        double xspeed = MathUtil.applyDeadband(controller.getLeftX(), IOConstants.Controller.kDeadband);
        double yspeed = MathUtil.applyDeadband(controller.getLeftY(), IOConstants.Controller.kDeadband);
        double rot = MathUtil.applyDeadband(controller.getRightX(), IOConstants.Controller.kDeadband);

        // build into a vector with max mag 1 to enforce max speeds correctly
        Vector2 velocity = new Vector2(xspeed, yspeed);
        double magnitude = velocity.getMagnitude();
        if (magnitude > 1.0) {
            velocity.setMagnitude(1.0);
            magnitude = 1.0;
        }

        // square the magnitude
        Vector2 velocity2 = velocity.multiply(magnitude);

        // square rotation keeping sign
        double rot2 = Math.copySign(Math.pow(rot, 2), rot);

        /*
         * Time to explain some wpilib strangeness
         *
         * xspeed, given from the controller, *should* be interpreted as the left-right speed of the robot
         * yspeed, given from the controller, *should* be interpreted as the forward-backward speed of the robot
         * however, the WPI coordinate system is such that +Xw is forward, and +Yw is left (using w for WPI)
         * and the controller coordinate system is such that +Xc is right, and +Yc is down (using c for controller)
         * so, we need to mutate x and y, so that +Xc becomes -Yw and +Yc becomes -Xw
         * also, WPIs rotation is ccw-positive and the controller is cw-positive, so we need to negate the rotation
         */
        double x = -velocity2.y;
        double y = -velocity2.x;
        double rotation = -rot2;

        this.drive(new ChassisSpeeds(x, y, rotation), DrivetrainIO.DriveMode.kTeleoperated);
    }

    /** Reset the gyro */
    public void resetGyro() {
        Rotation2d reset = AllianceUtil.facingDriver();
        this.io.resetHeading(reset);
    }

    /** Get Field widget */
    public Field2d getField() {
        return this.io.getField2d();
    }

    /** input data */
    public DrivetrainIO.DrivetrainData getData() {
        return this.io.data;
    }

    @Override
    public void periodic() {
        this.io.update();

        Logger.processInputs("Drivetrain", this.io.data);
        AdvantageUtil.logActiveCommand(this);

        SmartDashboard.putData("Field", this.getField());
    }
}
