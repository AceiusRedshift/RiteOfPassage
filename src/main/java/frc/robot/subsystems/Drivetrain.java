package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class Drivetrain extends SubsystemBase {
        private SwerveModule frontLeftSwerveModule;
        private SwerveModule frontRightSwerveModule;
        private SwerveModule backLeftSwerveModule;
        private SwerveModule backRightSwerveModule;

        private SwerveDriveKinematics kinematics;
        private SwerveDriveOdometry odometry;
        private AHRS gyro;

        private Pose2d targetPose = new Pose2d();
        private Pose2d currentPose = new Pose2d();

        private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);

        private ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

        public Drivetrain() {
                Translation2d frontLeftSwervePosition = new Translation2d(SwerveModuleConstants.MODULE_LOCATION_X,
                                SwerveModuleConstants.MODULE_LOCATION_Y);
                Translation2d frontRightSwervePosition = new Translation2d(SwerveModuleConstants.MODULE_LOCATION_X,
                                -SwerveModuleConstants.MODULE_LOCATION_Y);
                Translation2d backLeftSwervePosition = new Translation2d(-SwerveModuleConstants.MODULE_LOCATION_X,
                                SwerveModuleConstants.MODULE_LOCATION_Y);
                Translation2d backRightSwervePosition = new Translation2d(-SwerveModuleConstants.MODULE_LOCATION_X,
                                -SwerveModuleConstants.MODULE_LOCATION_Y);

                frontLeftSwerveModule = new SwerveModule(SwerveModuleConstants.VELOCITY_MOTOR_ID_FL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_FL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL,
                                frontLeftSwervePosition);

                frontRightSwerveModule = new SwerveModule(
                                SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR,
                                frontRightSwervePosition);

                backLeftSwerveModule = new SwerveModule(
                                SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL,
                                backLeftSwervePosition);

                backRightSwerveModule = new SwerveModule(
                                SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
                                SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR,
                                backRightSwervePosition);

                kinematics = new SwerveDriveKinematics(
                                frontLeftSwervePosition,
                                frontRightSwervePosition,
                                backLeftSwervePosition,
                                backRightSwervePosition);

                odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getAngle()),
                                new SwerveModulePosition[] {
                                                frontLeftSwerveModule.getPosition(),
                                                frontRightSwerveModule.getPosition(),
                                                backLeftSwerveModule.getPosition(),
                                                backRightSwerveModule.getPosition()
                                });
        }

        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] {
                                frontLeftSwerveModule.getPosition(),
                                frontRightSwerveModule.getPosition(),
                                backLeftSwerveModule.getPosition(),
                                backRightSwerveModule.getPosition()
                };
        }

        public void setSpeeds(ChassisSpeeds speeds) {
                targetChassisSpeeds = speeds;
        }

        public void stop() {
                targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        }

        @Override
        public void periodic() {
                currentPose = odometry.update(gyro.getRotation2d(), getModulePositions());

                SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetChassisSpeeds);
                frontLeftSwerveModule.setTargetState(states[0]);
                frontRightSwerveModule.setTargetState(states[1]);
                backLeftSwerveModule.setTargetState(states[2]);
                backRightSwerveModule.setTargetState(states[3]);
        }
}
