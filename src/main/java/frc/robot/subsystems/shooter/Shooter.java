// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonFX mShooterA = new TalonFX(ShooterConstants.kShootAMotorId);
  private TalonFX mShooterB = new TalonFX(ShooterConstants.kShootBMotorId);
  public Encoder eShooter = new Encoder(0, 1);

  final VelocityVoltage kRequest = new VelocityVoltage(0).withSlot(0);
  
  public Shooter() {

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1;
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    mShooterA.getConfigurator().apply(slot0Configs);
    mShooterB.getConfigurator().apply(slot0Configs);

    SmartDashboard.putNumber("shooter setter", 55);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooterCount", eShooter.get());
  }

  public void shootzero() {
    mShooterA.set(0.0);
    mShooterB.set(0.0);
  }

  public void shootNumMethod() {
    mShooterA.setControl(kRequest.withVelocity(SmartDashboard.getNumber("shooter setter", 0.0)).withSlot(0));
    mShooterB.setControl(kRequest.withVelocity(SmartDashboard.getNumber("shooter setter", 0.0)).withSlot(0));
  }

  public void shootNumMethod(double n) {
    mShooterA.setControl(kRequest.withVelocity(n).withSlot(0));
    mShooterB.setControl(kRequest.withVelocity(n).withSlot(0));
  }

  public void shootZero() {
    mShooterA.set(0.0);
    mShooterB.set(0.0);
  }
}
