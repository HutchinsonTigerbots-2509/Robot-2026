// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederHopper extends SubsystemBase {
  /** Creates a new Feeder. */
  public FeederHopper() {
    SmartDashboard.putNumber("feeder setter", -0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX mFeeder = new TalonFX(FeederHopperConstants.kFeederMotorId);
  private TalonFX mHopper = new TalonFX(FeederHopperConstants.kHopperMotorId);

  public void feedzero() {
    mFeeder.set(0.0);
  }

  public void feederNumMethod() {
    mFeeder.set(SmartDashboard.getNumber("feeder setter", 0.0));
  }

  // VVVVV Comp methods below VVVVV
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
