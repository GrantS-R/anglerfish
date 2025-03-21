
// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static java.lang.StrictMath.PI;

import frc.lib.geometry.Wheel;
import frc.lib.util.FeedforwardConstantsIO;
import frc.lib.util.MotorConstantsIO;
import frc.lib.util.PIDConstantsIO;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import java.util.Arrays;
import java.util.Hashtable;
import java.util.List;

public class Constants {
    public static class DeviceIDs {
        public static final int BACK_LEFT_TURN = 5;
        public static final int BACK_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 7;
        public static final int FRONT_RIGHT_TURN = 8;
        public static final int BACK_RIGHT_DRIVE = 2;
        public static final int BACK_RIGHT_TURN = 3;
        public static final int FRONT_LEFT_DRIVE = 6;
        public static final int FRONT_LEFT_TURN = 1;
        public static final int INDEXER = 9;
        public static final int SHOOTER_TOP = 11;
        public static final int SHOOTER_BOTTOM = 13;
        public static final int LEFT_INTAKE = 14;
        public static final int RIGHT_INTAKE = 15;
        
        public static final int PIGEON = 10;

        public static final int INDEXER_SENSOR = 0;
        public static final int[] TOP_SHOOTER_ENCODER = {1, 2};
        public static final int[] BOTTOM_SHOOTER_ENCODER = {3, 4};
    }

    public static class SensorConstants {
        public static final int ENCODER_SAMPLES_PER_AVERAGE = 100;

        public static final double TURNING_ENCODER_POSITION_FACTOR =
                2 * Math.PI;
        public static final double TURNING_ENCODER_VELOCITY_FACTOR =
                (2 * Math.PI) / 60.0;

        public static final Distance DRIVING_ENCODER_POSITION_FACTOR =
                Inches.of((1.5 * 2 * Math.PI) / RobotConstants.DRIVE_GEAR_RATIO);
        public static final LinearVelocity DRIVING_ENCODER_VELOCITY_FACTOR =
                MetersPerSecond.of(
                        ((RobotConstants.DRIVE_WHEELS.radius.in(Meters) * 2 * Math.PI)
                                        / RobotConstants.DRIVE_GEAR_RATIO)
                                / 60.0);

        public static final int DRIVE_ENCODER_SAMPLING_DEPTH = 2;

        public static final double SHOOTER_ALLOWED_ERROR = 0.1;
    }

    public static class MotorConstants {
        public static final double CAN_TIMEOUT =
                20; // Default value, but if CAN utilization gets too high, pop it to 0, or
        // bump it up+

        public static final MotorConstantsIO INTAKE =
                new MotorConstantsIO(false, 60, IdleMode.kCoast);

        public static final MotorConstantsIO INDEXER =
                new MotorConstantsIO(true, 60, IdleMode.kBrake);

        public static final MotorConstantsIO TOP_SHOOTER =
                new MotorConstantsIO(true, 60, IdleMode.kCoast);
        public static final MotorConstantsIO BOTTOM_SHOOTER =
                new MotorConstantsIO(true, 60, IdleMode.kCoast);

        public static final MotorConstantsIO DRIVE =
                new MotorConstantsIO(false, 60, IdleMode.kBrake);
        public static final MotorConstantsIO TURN =
                new MotorConstantsIO(false, 30, IdleMode.kBrake);
    }

    public static class ControlConstants {

        // =====================Drive=====================

        public static final double DRIVE_TURN_KS = 0.0;
        public static final PIDConstantsIO TURN_PID_CONSTANTS =
                // new PIDConstantsIO(3.4, 0, 0, 35, 0, 0); // Rubber
                new PIDConstantsIO(3.5, 0, 0, 35, 0, 0); // TPU
        public static final PIDConstantsIO DRIVE_PID_CONSTANTS =
                new PIDConstantsIO(0.2, 0, 0, 40, 0, 0);
        public static final FeedforwardConstantsIO DRIVE_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 0.245, 0, 0, 2.65, 0);
        public static final FeedforwardConstantsIO TURN_FEEDFORWARD_CONSTANTS =
                // new FeedforwardConstantsIO(0.2, 0.5, 0, 0, 0, 0); // Rubber
                new FeedforwardConstantsIO(0.22, 0.5, 0, 0, 0, 0); // TPU

        public static final double DEADBAND = 0.025;

        public static final PIDConstantsIO ANGLE_ALIGNMENT_PID_CONSTANTS =
                new PIDConstantsIO(0.925, 0, 0, 3.5, 0, 0);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(2),
                                RobotConstants.ROBOT_WIDTH.divide(2)), // Front Left
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(2),
                                RobotConstants.ROBOT_WIDTH.divide(-2)), // Front Right
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(-2),
                                RobotConstants.ROBOT_WIDTH.divide(2)), // Back Left
                        new Translation2d(
                                RobotConstants.ROBOT_LENGTH.divide(-2),
                                RobotConstants.ROBOT_WIDTH.divide(-2)) // Back Right
                        );

        // =====================Shooter=====================

        public static final FeedforwardConstantsIO TOP_SHOOTER_FEEEDFORWARD =
                new FeedforwardConstantsIO(0, 0.000178, 0, .0021);
        public static final FeedforwardConstantsIO BOTTOM_SHOOTER_FEEDFORWARD =
                new FeedforwardConstantsIO(0, 0.000182, 0, .0021);
        public static final PIDConstantsIO SHOOTER_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.000245, 0, 0, .023, 0, 0);

        // =====================Intake=====================

        public static final PIDConstantsIO INTAKE_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.01, 0, 0, .6, 0, 0);
        public static final FeedforwardConstantsIO INTAKE_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 10.25, 0, .0022);

        // =====================Indexer=====================

        public static final PIDConstantsIO INDEXER_FEEDBACK_CONSTANTS =
                new PIDConstantsIO(0.05, 0, 0, .36, 0, 0);
        public static final FeedforwardConstantsIO INDEXER_FEEDFORWARD_CONSTANTS =
                new FeedforwardConstantsIO(0, 2.4, 0, .0021);

        // =====================Pathplanner=====================

        public static final PIDConstantsIO TRANSLATION_PID_CONSTANTS =
                new PIDConstantsIO(5.4, 0, 0, 40, 0, 0);
        public static final PIDConstantsIO ROTATION_PID_CONSTANTS =
                new PIDConstantsIO(3.5, 0, 0, 30, 0, 0);

        public static final PIDConstantsIO STAY_PUT_TRANSLATION_PID_CONSTANTS =
                new PIDConstantsIO(4, 0, 0, 3, 0, 0);
        public static final PIDConstantsIO STAY_PUT_ROTATION_PID_CONSTANTS =
                new PIDConstantsIO(8, 0, 0, 25, 0, 0);

        public static final Distance REPLANNING_ERROR_THRESHOLD = Inches.of(6);
        public static final Distance REPLANNING_ERROR_SPIKE_THRESHOLD = Inches.of(4);

        public static final PathConstraints PATH_CONSTRAINTS =
                new PathConstraints(
                        RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                        RobotConstants.MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond),
                        RobotConstants.MAX_ANGULAR_VELOCITY,
                        RobotConstants.MAX_ANGULAR_ACCELERATION);

        public static final Distance ALLOWED_AUTO_ERROR = Inches.of(10);
    }

    public static final class SetpointConstants {
        public static final double INTAKE_SPEED = 600;
        public static final double INTAKE_REVERSE_SPEED = -600;

        public static final double SHOOTER_INDEXER_SPEED = 3000;
        public static final double INTAKE_INDEXER_SPEED = 600;

        public enum SHOOTER_SPEEDS {
            SPEAKER(Arrays.asList(RPM.of(3850), RPM.of(3850))),
            AMP(Arrays.asList(RPM.of(300), RPM.of(1000))),
            INTAKE(Arrays.asList(RPM.of(-800), RPM.of(-602))),
            REVERSE(Arrays.asList(RPM.of(-400), RPM.of(-400))),
            IDLE(Arrays.asList(RPM.of(500), RPM.of(500)));

            public final AngularVelocity topSpeed;
            public final AngularVelocity bottomSpeed;

            SHOOTER_SPEEDS(List<AngularVelocity> speeds) {
                this.topSpeed = speeds.get(0);
                this.bottomSpeed = speeds.get(1);
            }
        }
    }

    public static final class RobotConstants {
        public static final Voltage NOMINAL_VOLTAGE = Volts.of(12);

        public static final Wheel INTAKE_WHEELS = new Wheel(Inches.of(1.5));

        public static final Wheel DRIVE_WHEELS = new Wheel(Inches.of(1.5));
        private static final Distance ROBOT_WIDTH = Inches.of(21.375);
        private static final Distance ROBOT_LENGTH = Inches.of(21.375);

        public static final LinearVelocity MAX_LINEAR_SPEED =
                MetersPerSecond.of(1.5);
        public static final double MAX_ANGULAR_VELOCITY =
                2 * PI;
        public static final LinearAcceleration MAX_LINEAR_ACCELERATION =
                MetersPerSecondPerSecond.of(2);
        public static final double MAX_ANGULAR_ACCELERATION =
                4 * Math.PI;

        private static final int DRIVING_PINION_TEETH = 13;
        public static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVING_PINION_TEETH * 15);
        public static final double TURN_GEAR_RATIO = (62.0 / 14) * 12;
    }

    public static class AlignConstants {
        public static final Pose2d AMP_POSITION =
                new Pose2d(1.88, 7.73, Rotation2d.fromRadians(-Math.PI / 2));

        public static final Distance ALLOWED_DISTANCE_FROM_AMP = Meters.of(2);

        public static final Distance SHOOT_DISTANCES_METERS = Meters.of(1.541018);
        public static final Pose2d SPEAKER_POSITION = new Pose2d();

        public static final Translation2d ALLOWED_DISTANCE_FROM_SHOOT = new Translation2d(2, 2);

        public static final Translation2d ALLOWED_DISTANCE_FROM_NOTE = new Translation2d(2, 2);
    }

    public static class VisionConstants {
        public static final int ACTIVE_CAMERAS = 3;

        public static final Pose2d[] CAMERA_POSITIONS = {
            new Pose2d(-0.1524, -0.26035, new Rotation2d(Math.toRadians(90))),
            new Pose2d(-0.1651, 0.0254, new Rotation2d(Math.toRadians(180))),
            new Pose2d(-0.1524, 0.26035, new Rotation2d(Math.toRadians(270)))
        }; // x is forward, y is left, counterclockwise on rotation

        public static final String[][] TOPIC_NAMES = {
            {"Cam1Tvec", "Cam1Rvec", "Cam1Ids"},
            {"Cam2Tvec", "Cam2Rvec", "Cam2Ids"},
            {"Cam3Tvec", "Cam3Rvec", "Cam3Ids"}
        };

        public static final double[] VECTOR_DEFAULT_VALUE = {0};
        public static final int ID_DEFAULT_VALUE = 0;

        public static final Distance MAX_MEASUREMENT_DIFFERENCE = Meters.of(1.5);
        public static final Rotation2d MAX_ANGLE_DIFFERENCE = Rotation2d.fromDegrees(10);

        public static final double ROTATION_STDEV = 50 * Math.PI;
        public static final double TRANSLATION_STDEV_ORDER = 2;
        public static final double TRANSLATION_STDEV_SCALAR = 2;
    }

    public static class AutoConstants {
        public static Hashtable<String, Pose2d> STARTING_POSITIONS =
                new Hashtable<>() {
                    {
                        put("Subwoofer - Left", new Pose2d(0.668, 6.72, new Rotation2d(PI / 3)));
                        put("Subwoofer - Front", new Pose2d(1.374775, 5.553456, new Rotation2d()));
                        put("Subwoofer - Right", new Pose2d(0.668, 4.39, new Rotation2d(-PI / 3)));
                    }
                };

        public static Pose2d[] SCORING_POSITIONS = {
            new Pose2d(0.668, 6.72, new Rotation2d(PI / 3)),
            new Pose2d(1.374775, 5.553456, new Rotation2d()),
            new Pose2d(0.668, 4.39, new Rotation2d(-PI / 3))
        };

        public static Translation2d[] NOTE_POSITIONS = {
            new Translation2d(2.8956, 7.001256),
            new Translation2d(2.8956, 5.553456),
            new Translation2d(2.8956, 4.105656)
        };

        public static final Distance REPLANNING_THRESHOLD = Inches.of(6);

        public static final boolean IS_RED = false;
    }
}
