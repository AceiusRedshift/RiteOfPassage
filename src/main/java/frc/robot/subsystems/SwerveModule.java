package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;

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

    private final Supplier<Double> steeringEncoderPosition;

    public SwerveModule(int driveMotorId, int steeringMotorId, int steeringEncoderId, double steeringEncoderOffset,
            Translation2d moduleOffset) {
        this.moduleOffset = moduleOffset;

        // IDK
        targetState = new SwerveModuleState();

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
        steeringEncoder.getConfigurator()
                .apply(new CANcoderConfiguration()
                        .withMagnetSensor(new MagnetSensorConfigs()
                                .withMagnetOffset(SwerveModuleConstants.MAGNET_OFFSET_ROTATIONS)));

        drivePidController = driveMotor.getPIDController();
        drivePidController.setP(0.0001);
        drivePidController.setI(0);
        drivePidController.setD(0);
        driveEncoder.setPositionConversionFactor(1 / ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)));
        driveEncoder.setVelocityConversionFactor(1 / ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)));

        steeringPidController = steeringMotor.getPIDController();
        steeringPidController.setP(1);
        steeringPidController.setI(0);
        steeringPidController.setD(0);
        steeringPidController.setPositionPIDWrappingEnabled(true);
        steeringPidController.setPositionPIDWrappingMinInput(0);
        steeringPidController.setPositionPIDWrappingMaxInput(1);
        steeringMotor.getEncoder().setPositionConversionFactor(1 / (12.8 / 1.0));
        steeringMotor.getEncoder().setVelocityConversionFactor(1 / (12.8 / 1.0));

        steeringEncoderPosition = steeringEncoder.getAbsolutePosition().asSupplier();
    }

    @Override
    public void periodic() {
        // Michael: mix up here, target state has the angle you want to go to, the
        // steering encoder has the actuall current angle of the wheel
        steeringPidController.setReference(targetState.angle.getRotations(), ControlType.kPosition);

        // Michael: also remeber that targetState.speedMetersPerSecond is in m/sec, and
        // setReference in velocity mode takes rotations/sec (RPM)
        // like how you had to convert in getMetersTraveled, but reverse
        drivePidController.setReference(
                Units.radiansToRotations(
                        targetState.speedMetersPerSecond / (Constants.SwerveModuleConstants.WHEEL_DIAMETER_METERS
                                / 2)),
                ControlType.kVelocity);
    }

    public double getMetersTraveled() {
        // M: this is only really useful for creating SwerveModulePosition, i would keep
        // private
        return driveEncoder.getPosition() * Constants.SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
    }

    public SwerveModulePosition getPosition() {
        // M: what in the sigma is angle??
        return new SwerveModulePosition(getMetersTraveled(), angle);
    }

    public Translation2d getOffset() {
        return moduleOffset;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = state;

        // currenly the module may preform a 180 to get the target anlge (for example if
        // we reverse directions)
        // we dont ever acutally need to rotate more than +/- 90 degrees to reach a
        // desired angle, since we can instead just drive in reverse
        // SwerveModuleState has a method to optimise this
    }
}
