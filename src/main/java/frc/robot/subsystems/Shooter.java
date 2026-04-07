// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonFX mShooterA = new TalonFX(9);
  private TalonFX mShooterB = new TalonFX(10);
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
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shootVariable(double n) {
    mShooterA.setControl(kRequest.withVelocity(n).withSlot(0));
    mShooterB.setControl(kRequest.withVelocity(n).withSlot(0));
  }

  public void shootZero() {
    mShooterA.set(0.0);
    mShooterB.set(0.0);
  }
}
