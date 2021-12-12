// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveConstants {
        public static final int kFrontLeftId = 3;
        public static final int kFrontRightId = 0;
        public static final int kBackLeftId = 7;
        public static final int kBackRightId = 4;

        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kGearRatio = 10.71;

        
        public static final double kTrackWidthMeters = 0.6; // TODO: placeholder (Track Width)
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    }

    public final static class ControllerConstants {
        public static final int CONTROLLER_PORT = 0; // Controller port

        // Sticks
        public static final int S_RIGHT_X_PORT = 4; // Right stick x
        public static final int S_RIGHT_Y_PORT = 5; // Right stick y
        public static final int S_LEFT_X_PORT = 0; // Left stick x
        public static final int S_LEFT_Y_PORT = 1; // Left stick y

        public static final int S_LEFT = 9; // Left stick button
        public static final int S_RIGHT = 10; // Right stick button

        // Triggers
        public static final int TRIGGER_RIGHT_PORT = 3; // Right trigger
        public static final int TRIGGER_LEFT_PORT = 2; // Left trigger

        // Bumpers
        public static final int BUMPER_RIGHT_PORT = 6; // Right bumper
        public static final int BUMPER_LEFT_PORT = 5; // Left bumper

        // Buttons
        public static final int BUTTON_A_PORT = 1; // A Button
        public static final int BUTTON_B_PORT = 2; // B Button
        public static final int BUTTON_X_PORT = 3; // X Button
        public static final int BUTTON_Y_PORT = 4; // Y Button

        // Special buttons
        public static final int BUTTON_MENU_PORT = 8; // Menu Button
        public static final int BUTTON_START_PORT = 7; // Start button
    }

    public static final class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double ksVolts = 1; // TODO: Placeholder (kS)
        public static final double kvVoltSecondsPerMeter = 2; // TODO: PLACEHOLDER (kV)
        public static final double kaVoltSecondsSquaredPerMeter = 0.2; // TODO: PLACEHOLDER (kA)
        public static final double kPDriveVel = 10; // TODO: placeholder (kP)

        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

        public static Trajectory getTrajectory(String path) {
            try {
                return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("/home/lvuser/deploy/output/" + path + ".wpilib.json"));
            } catch (Exception e) {
                System.out.println("[ERROR] Something bad happened. You bloody idiot");
                e.printStackTrace();
                return null;
            }
        }
    }
}
