package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveToPose extends Command {
    private final Drivetrain drivetrain;
    private final Pose2d targetPose;
    private Pose2d currentPose;

    private PIDController xPidController = new PIDController(0, 0, 0);
    private PIDController yPidController = new PIDController(0, 0, 0);
    private PIDController rPidController = new PIDController(0, 0, 0);

    /**
     * Inject dependencies and reserve systems
     */
    public DriveToPose(Drivetrain drivetrain, Pose2d desiredPose) {
        this.drivetrain = drivetrain;
        this.targetPose = desiredPose;

        addRequirements(this.drivetrain);
    }

    /**
     * Begin moving
     */
    @Override
    public void initialize() {
        xPidController.setTolerance(SwerveModuleConstants.POSITION_TOLERANCE_METERS);
        yPidController.setTolerance(SwerveModuleConstants.POSITION_TOLERANCE_METERS);
        rPidController.setTolerance(SwerveModuleConstants.ANGLE_TOLERANCE_RADIANS);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
                xPidController.calculate(currentPose.getX(), targetPose.getX()),
                yPidController.calculate(currentPose.getY(), targetPose.getY()),
                // Radians
                rPidController.calculate(currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians()));

        drivetrain.setCurrentSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return xPidController.atSetpoint() && yPidController.atSetpoint() && rPidController.atSetpoint();
    }

    /**
     * Turn off motors when we don't want the robot to move anymore
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
