package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private final Translation2d moduleOffset;
    private Rotation2d angle;

    private SwerveModuleState targetState;

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkPIDController drivePidController;

    private final CANSparkMax steeringMotor;
    // This one matters more and runs on
    private final CANcoder steeringEncoder;
    private final SparkPIDController steeringPidController;

    public SwerveModule(int driveMotorId, int steeringMotorId, int steeringEncoderId, double steeringEncoderOffset,
            Translation2d moduleOffset) {
        this.moduleOffset = moduleOffset;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        steeringMotor = new CANSparkMax(steeringMotorId, MotorType.kBrushless);

        // Turns out this is best practice, in case a motor is swapped out
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        steeringMotor.restoreFactoryDefaults();
        steeringMotor.setIdleMode(IdleMode.kBrake);

        // TODO Setup gear ratios

        // Setup encoders
        driveEncoder = driveMotor.getEncoder();
        steeringEncoder = new CANcoder(steeringEncoderId);

        drivePidController = driveMotor.getPIDController();
        drivePidController.setP(0);
        drivePidController.setI(0);
        drivePidController.setD(0);

        steeringPidController = steeringMotor.getPIDController();
        steeringPidController.setP(0);
        steeringPidController.setI(0);
        steeringPidController.setD(0);
        steeringPidController.setPositionPIDWrappingEnabled(true);
        steeringPidController.setPositionPIDWrappingMinInput(0);
        steeringPidController.setPositionPIDWrappingMaxInput(1);
    }

    @Override
    public void periodic() {
        // Get the current rotations of the steering encoder
        final double rotations = steeringEncoder.getAbsolutePosition().refresh().getValueAsDouble();

        steeringPidController.setReference(rotations, ControlType.kPosition);
        drivePidController.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
    }

    public double getMetersTraveled() {
        return driveEncoder.getPosition() * Constants.SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getMetersTraveled(), angle);
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = state;
    }
}
