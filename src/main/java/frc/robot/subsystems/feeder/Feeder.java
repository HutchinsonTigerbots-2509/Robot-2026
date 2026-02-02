// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public Feeder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public TalonFX mFeeder = new TalonFX(FeederConstants.kFeederMotorId);

  public void feedToShoot() {
    mFeeder.set(0.1);
  }

  public void feedZero() {
    mFeeder.set(0);
  }

  public void feednegative() {
    mFeeder.set(-0.1);
  }
}
