package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends Command {
    Drivetrain drivetrain;
    CommandXboxController controller;
    Boolean fieldRelative;

    public TeleopDriveCommand(Drivetrain drivetrain, CommandXboxController controller, Boolean fieldRelative) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.fieldRelative = fieldRelative;
    }

    /**
     * Begin moving
     */
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (fieldRelative) {

        } else {
            drivetrain.setSpeeds(
                    new ChassisSpeeds(controller.getLeftX(), controller.getRightX(), controller.getRightX()));
        }

        // Consider the following
        // double rcw = pJoystick -> GetTwist();
        // double forwrd = pJoystick -> GetY() * -1; /* Invert stick Y axis */
        // double strafe = pJoystick -> GetX();

        // float pi = 3.1415926;

        // /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

        // double gyro_degrees = ahrs -> GetYaw();
        // float gyro_radians = gyro_degrees * pi / 180;
        // float temp = forwrd * cos(gyro_radians) +
        // strafe * sin(gyro_radians);
        // strafe = -forwrd * sin(gyro_radians) +
        // strafe * cos(gyro_radians);
        // fwd = temp;

        // /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
        /* rotated by the gyro angle, and can be sent to drive system */
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Turn off motors when we don't want the robot to move anymore
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}