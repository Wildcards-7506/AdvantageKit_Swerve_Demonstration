// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax m_shooterL;
  private final SparkMax m_shooterR;
  private final SparkMaxConfig c_shooterL;
  private final SparkMaxConfig c_shooterR;
  private final SparkClosedLoopController pid_shooterR;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterL = new SparkMax(12, MotorType.kBrushless);
    m_shooterR = new SparkMax(13, MotorType.kBrushless);
    c_shooterL = new SparkMaxConfig();
    c_shooterR = new SparkMaxConfig();
    pid_shooterR = m_shooterR.getClosedLoopController();

    c_shooterL.idleMode(IdleMode.kCoast).smartCurrentLimit(40).follow(13, true);

    c_shooterR.idleMode(IdleMode.kCoast).smartCurrentLimit(40).closedLoop.pid(5, 0, 0);

    m_shooterL.configure(
        c_shooterL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterR.configure(
        c_shooterR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
  }

  public void setShooterVelocity(double setpoint) {
    pid_shooterR.setReference(setpoint, ControlType.kVelocity);
  }

  public double getShooterVelocity() {
    return m_shooterR.getEncoder().getVelocity();
  }
}
