// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants.TURRET_GEAR_RATIO;

/**
 * The subsystem for the shooter
 */
public class ShooterSubsystem extends SubsystemBase {
    private WPI_TalonFX m_ShooterMotor;
    private WPI_TalonFX m_TurretMotor;
    private double turretAngle = 0;
    private double shooterSpeed = 0;
    /**
     * Creates a new ShooterSubsystem.
     */
    public ShooterSubsystem() {
        m_ShooterMotor = new WPI_TalonFX(Constants.ShooterConstants.kShooterId);
        m_TurretMotor = new WPI_TalonFX(Constants.ShooterConstants.kTurretID);
        m_TurretMotor.setSelectedSensorPosition(0);
        m_ShooterMotor.setNeutralMode(NeutralMode.Brake);
        m_TurretMotor.setNeutralMode(NeutralMode.Brake);


        updateTurretAngle();
    }


    /**
     * Updates {@link #turretAngle} to the angle, in degrees, of the rotation of the turret.
     */
    public void updateTurretAngle() {
        turretAngle = (m_TurretMotor.getSelectedSensorPosition() * TURRET_GEAR_RATIO * 360) / 2048;
    }


    /**
     * @return {@link #turretAngle} - the Angle in degrees of the rotation of the turret.
     * Should be between -90 and 90.
     */
    public double getTurretAngle() {
        return turretAngle;
    }


    /**
     * Sets the rotation percent of the Turret motor.
     * Will not set the speed if the rotation will make the angle exceed 90 degrees.
     *
     * @param percentOutput percent, from -1, to 1, to set the turret speed.
     */
    public void setTurretSpeed(double percentOutput) {
        updateTurretAngle();
        if ((turretAngle < 90 || percentOutput <= 0) && (turretAngle > -90 || percentOutput >= 0)) {
            m_TurretMotor.set(ControlMode.PercentOutput, percentOutput);
        }
    }
    public void setShooterSpeed(double percentOutput) {
        m_ShooterMotor.set(ControlMode.PercentOutput, percentOutput);
    }


    @Override
    public void periodic() {
        updateTurretAngle();
        SmartDashboard.putNumber("turretAngle", turretAngle);
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);


        // This method will be called once per scheduler run
    }
}
