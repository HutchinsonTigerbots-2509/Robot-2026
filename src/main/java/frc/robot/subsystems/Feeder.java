// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public Feeder() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1;
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    mFeeder.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX mFeeder = new TalonFX(13);

  final VelocityVoltage kRequest = new VelocityVoltage(0).withSlot(0);

  public void feedzero() {
    mFeeder.set(0.0);
  }

  public void feedPower() {
    mFeeder.set(-0.8);
  }

  public void feedReverse() {
    mFeeder.set(0.3);
  }

  public void feedZero() {
    mFeeder.set(0);
  }

  public void feedVariable(double n) {
    mFeeder.setControl(kRequest.withVelocity(n).withSlot(0));
  }
}
