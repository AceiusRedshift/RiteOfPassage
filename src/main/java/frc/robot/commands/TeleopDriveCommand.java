package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends Command {
    Drivetrain drivetrain;
    CommandXboxController controller;
    Boolean fieldRelative;

    public TeleopDriveCommand(Drivetrain drivetrain, CommandXboxController controller, Boolean fieldRelative) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.fieldRelative = fieldRelative;

        addRequirements(drivetrain);
    }

    /**
     * Begin moving
     */
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Michael: 4, extra not required feature. we like to square and copy sign of
        // rotation speed from controller to make nicer driver exsperence (better
        // control at low speeds, quick ramp to fast speeds)

        double forward = controller.getLeftY() * Constants.SwerveModuleConstants.MAX_SPEED_MS;
        double strafe = controller.getLeftX() * -Constants.SwerveModuleConstants.MAX_SPEED_MS;
        double rotation = controller.getRightX() * Constants.SwerveModuleConstants.MAX_SPEED_MS;

        SmartDashboard.putNumber("F", forward);
        SmartDashboard.putNumber("S", strafe);
        SmartDashboard.putNumber("R", rotation);

        if (fieldRelative) {
            double gyroDeg = drivetrain.getGyro().getAngle();
            double gyroRad = gyroDeg * (Math.PI / 180);
            double temp = (forward * Math.cos(gyroRad)) + (strafe * Math.sin(gyroRad));
            strafe = (-forward * Math.sin(gyroRad)) + (strafe * Math.cos(gyroRad));
            forward = temp;
        }

        // I think this is the right order?
        drivetrain.setSpeeds(new ChassisSpeeds(forward, strafe, rotation));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Turn off motors when we don't want the robot to move anymore
     */
    //
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}