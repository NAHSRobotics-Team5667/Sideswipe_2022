package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollower {
    public static Command getRamseteCommand(Trajectory path, Drivetrain m_drive) {
        RamseteCommand auto = new RamseteCommand(
            path, 
            m_drive::getPose, 
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
            // AutoConstants.autoMotorFeedforward, 
            new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, 
            m_drive::getWheelSpeeds, 
            new PIDController(AutoConstants.kPDriveVel, 0, 0), 
            new PIDController(AutoConstants.kPDriveVel, 0, 0),
            m_drive::tankDriveVoltage, 
            m_drive);

        m_drive.resetOdometry(path.getInitialPose());

        return auto.andThen(() -> m_drive.tankDriveVoltage(0,0));
    }
}
