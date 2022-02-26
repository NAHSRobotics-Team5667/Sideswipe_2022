// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class shootercommandments extends CommandBase {
  private TurretSubsystem turret;
  /** Creates a new shootercommandments. */
  public shootercommandments(TurretSubsystem turret) {
      this.turret = turret;
      addRequirements(turret);
      // Use addRequirements() here to declare subsystem dependencies.
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(turret.turretAngle()>=90){
      if(RobotContainer.getController().getLeftBumper()){
        turret.set(0.1);
      }
    }else if(turret.turretAngle()<=-90){
      if(RobotContainer.getController().getRightBumper()){
        turret.set(-0.1);
      }
    }else{
      if(RobotContainer.getController().getLeftBumper()){
        turret.set(0.1);
      }else if(RobotContainer.getController().getRightBumper()){
        turret.set(-0.1);
      }else{
        turret.set(0);
      }
    }
    
  }
   
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
