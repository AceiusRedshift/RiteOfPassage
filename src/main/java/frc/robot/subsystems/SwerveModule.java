package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
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

    private final Supplier<Double> steeringEncoderPosition;

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

        // MICHAEL: steering encoder offset is the distance between the actual forward
        // position of the module and what the encoder believes to be the zero position
        // to get the forward position of the module, basically when the encoder reads
        // steeringEncoderOffset then the wheel is forward

        // MICHAEL: steering encoder offset can be done though CANcoderConfiguration
        // MagnetSensorConfigs, which is NOT documented anywhere ( i believe since you
        // are meant to do this though there hardware client),
        // so heres the code to do it (obviuslly not exspecting you to know this)
        cancoder
                .getConfigurator()
                .apply(
                        new CANcoderConfiguration()
                                .withMagnetSensor(
                                        new MagnetSensorConfigs()
                                                .withMagnetOffset(config.absoluteEncoderOffset().getRotations())));
        // basically this code just has the magnetic encoder add absoluteEncoderOffset
        // to its reading, meaning 0 is now forward as we want it

        // now that you have a absolute encoder telling you the actuall forward angle,
        // you can do two things (up to you)
        // you can do what Aleah liked to do and create a new PIDcontroller on the
        // roboRIO and only plug in the absolute angle from the encoder as the measured
        // the setpoint from target SwerveModuleState as the desired
        // this has the benifit of even if the gear slips the rotate pid will still work
        // or you can do whtat i like and set the rotate motor encoder to match the
        // absolute encoder on init, and treat the relative encoder as absolute now that
        // it has been matched
        // this has the benifit of using the supior pid controller built into the spark
        // max (better since it can run it cycles of pid much faster since it does not
        // have to run on command scedulor loop)

        // we will ask hardware what the chance of a slip is on the new swerve module
        // gear ratios, (it was very low last year)
        // for now just pick one method

        // also sorry if this made no sence we can chat later

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

        steeringEncoderPosition = steeringEncoder.getAbsolutePosition().asSupplier();
    }

    @Override
    public void periodic() {
        // Get the current rotations of the steering encoder
        final double rotations = steeringEncoderPosition.get();

        // Michael: mix up here, target state has the angle you want to go to, the
        // steering encoder has the actuall current angle of the wheel
        steeringPidController.setReference(rotations, ControlType.kPosition);

        // Michael: also remeber that targetState.speedMetersPerSecond is in m/sec, and
        // setReference in velocity mode takes rotations/sec (RPM)
        // like how you had to convert in getMetersTraveled, but reverse
        drivePidController.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
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
