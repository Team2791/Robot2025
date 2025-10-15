package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.ArmConstants;
import frc.constants.IOConstants;

public class ExtendArm extends SubsystemBase implements ExtendArmInterface {
    private final MotorController extMotor;
    private final Encoder extEncoder;

    private double extTarget = 0.0;

    public ExtendArm(MotorController motorController, Encoder encoder) {
        this.extMotor = motorController;
        this.extEncoder = encoder;

        extMotor.setIdleMode(IdleMode.kBrake);
        extEncoder.setPosition(0.0);
        extEncoder.setPositionConversionFactor(ArmConstants.Extension.kPositionFactor);

        var tab = Shuffleboard.getTab("ExtendArm");
        tab.addNumber("Extension Percent", this::getExtension);
        tab.addNumber("Extension Encoder", extEncoder::getPosition);
        tab.addNumber("Extension Target", () -> extTarget);
    }

    public double getExtension() {
        return extEncoder.getPosition() * ArmConstants.Extension.kPositionFactor;
    }

    public void setExtTarget(double value) {
        extTarget = normalizeExt(value);
    }

    private static double normalizeExt(double value) {
        return Math.max(ArmConstants.Extension.kMin, Math.min(value, ArmConstants.Extension.kMax));
    }

    @Override
    public boolean atExtTarget() {
        return Math.abs(getExtension() - extTarget) < ArmConstants.kValueTolerance;
    }

    @Override
    public void extend() {
        setExtTarget(ArmConstants.Extension.kMax);
    }

    @Override
    public void retract() {
        setExtTarget(ArmConstants.Extension.kMin);
    }

    @Override
    public void holdExtension() {
        setExtTarget(getExtension());
    }

    public void extendSetpoint() {
        if (atExtTarget()) {
            extMotor.set(0.0);
        } else if (extTarget < getExtension() && getExtension() > ArmConstants.Extension.kMin + 10) {
            extMotor.set(-ArmConstants.Extension.kSpeed);
        } else if (extTarget > getExtension() && getExtension() < ArmConstants.Extension.kMax - 10) {
            extMotor.set(ArmConstants.Extension.kSpeed);
        }
    }

    @Override
    public void periodic() {
        extendSetpoint();
    }
}