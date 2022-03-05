package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooter;
    public ShooterCommand(ShooterSubsystem shooter) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    shooter.setTurretSpeed(0);
    shooter.setShooterSpeed(0);
    }

    @Override
    public void execute() {

        boolean rightBumper = RobotContainer.getController().getRightBumper();
        boolean leftBumper = RobotContainer.getController().getLeftBumper();
        if (rightBumper && shooter.getTurretAngle()<90) {
            shooter.setTurretSpeed(.1);
        }else if (leftBumper && shooter.getTurretAngle()>-90){
            shooter.setTurretSpeed(-.1);
        }else{
            shooter.setTurretSpeed(0);
        }
        SmartDashboard.putBoolean("Right bumper", rightBumper);
        SmartDashboard.putBoolean("Left bumper", leftBumper);

        double rightTrigger = RobotContainer.controller.getRightTrigger();
        shooter.setShooterSpeed(rightTrigger);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setTurretSpeed(0);
        shooter.setShooterSpeed(0);
    }

}
