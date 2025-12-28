// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final SparkMax m_climberL;
  private final SparkMax m_climberR;
  private final SparkMaxConfig c_climber;

  /** Creates a new Climber. */
  public Climber() {
    m_climberL = new SparkMax(14, MotorType.kBrushless);
    m_climberR = new SparkMax(15, MotorType.kBrushless);
    c_climber = new SparkMaxConfig();

    c_climber.smartCurrentLimit(60).idleMode(IdleMode.kBrake);
    c_climber
        .softLimit
        .forwardSoftLimit(12)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    c_climber.encoder.positionConversionFactor(1 / 5.0 * 5.0);

    m_climberL.configure(c_climber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_climberR.configure(c_climber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", getClimberPosition());
  }

  public void setClimberVoltage(double setpoint) {
    m_climberL.setVoltage(setpoint);
    m_climberR.setVoltage(setpoint);
  }

  public double getClimberPosition() {
    return m_climberL.getEncoder().getPosition();
  }
}
