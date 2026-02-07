// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public Feeder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public TalonFX mFeeder = new TalonFX(FeederConstants.kFeederMotorId);

  public Double num = 0.1;
  public void feedincrement() {
    if(num < 1) {
      num = num + 0.01;
      SmartDashboard.putNumber("feeder speed", num);
    }
    else {
      SmartDashboard.putNumber("feeder speed", num);
    }
  }
  
  public void feedincrement10() {
    if(num < 1) {
      num = num + 0.10;
      SmartDashboard.putNumber("feeder speed", num);
    }
    else {
      SmartDashboard.putNumber("feeder speed", num);
    }
  }

  public void feedincrementreset() {
    num = 0.1;
    SmartDashboard.putNumber("feeder speed", num);
  }

  public void feednum() {
    mFeeder.set(num);
    SmartDashboard.putNumber("feeder speed", num);
  }

  public void feedzero() {
    mFeeder.set(0.0);
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
