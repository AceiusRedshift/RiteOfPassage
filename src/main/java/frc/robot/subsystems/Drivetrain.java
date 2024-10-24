package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
        private SwerveModule frontLeftSwerveModule;
        private SwerveModule frontRightSwerveModule;
        private SwerveModule backLeftSwerveModule;
        private SwerveModule backRightSwerveModule;

        private SwerveDriveKinematics kinematics;

        private SwerveDriveOdometry odometry;
        private SwerveDrivePoseEstimator poseEstimator;

        private Pigeon2 gyro;

        private ChassisSpeeds targetSpeeds = new ChassisSpeeds(0, 0, 0);
        private ChassisSpeeds currentSpeeds = new ChassisSpeeds(0, 0, 0);

        public Drivetrain(SwerveModule frontLeftSwerveModule, SwerveModule frontRightSwerveModule,
                        SwerveModule backLeftSwerveModule, SwerveModule backRightSwerveModule) {
                SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                                frontLeftSwerveModule.getPosition(),
                                frontRightSwerveModule.getPosition(),
                                backLeftSwerveModule.getPosition(),
                                backRightSwerveModule.getPosition()
                };

                kinematics = new SwerveDriveKinematics(
                                frontLeftSwerveModule.getOffset(),
                                frontRightSwerveModule.getOffset(),
                                backLeftSwerveModule.getOffset(),
                                backRightSwerveModule.getOffset());

                odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), modulePositions);

                poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), modulePositions,
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        }

        public SwerveDriveWheelPositions getModulePositions() {
                return new SwerveDriveWheelPositions(new SwerveModulePosition[] {
                                frontLeftSwerveModule.getPosition(),
                                frontRightSwerveModule.getPosition(),
                                backLeftSwerveModule.getPosition(),
                                backRightSwerveModule.getPosition()
                });
        }

        private SwerveModuleState[] getStates() {
                return kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(targetSpeeds, Robot.kDefaultPeriod));
        }

        public SwerveDriveWheelStates getSwerveDriveWheelStates() {
                return new SwerveDriveWheelStates(getStates());
        }

        public void setSpeeds(ChassisSpeeds speeds) {
                targetSpeeds = speeds;
        }

        public ChassisSpeeds getSpeeds() {
                return kinematics.toChassisSpeeds(kinematics.toSwerveModuleStates(currentSpeeds));
        }

        public void stop() {
                targetSpeeds = new ChassisSpeeds(0, 0, 0);
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = getStates();
                SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveModuleConstants.MAX_SPEED_MS);

                frontLeftSwerveModule.setTargetState(states[0]);
                frontRightSwerveModule.setTargetState(states[1]);
                backLeftSwerveModule.setTargetState(states[2]);
                backRightSwerveModule.setTargetState(states[3]);
        }
}
