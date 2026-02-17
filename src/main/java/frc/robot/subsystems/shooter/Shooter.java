// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederHopper;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonFX mShooterA = new TalonFX(ShooterConstants.kShootAMotorId);
  private TalonFX mShooterB = new TalonFX(ShooterConstants.kShootBMotorId);
  private Encoder eShooter = new Encoder(0, 1); //TODO: Put in actual encoder channels

  private Double kOffset; //This should be the shooter speed target.
  private Double maxRPM = 120000.0; //This is an approximation.
  private Double max2RPM = 0.0;
  
  public Shooter() {
    // ShooterConstants.shootPID.setTolerance(0.0);
    // ShooterConstants.shootPID.setSetpoint(0.0); //TODO: Find acceptable setpoint.
    kOffset = 0.0; // Find an acceptable offset.

    // FOR TESTING PURPOSES
    ShooterConstants.shootPID.setSetpoint(0.9);
    kOffset = 0.6;
    SmartDashboard.putNumber("shooter setter", 0.55);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rpm = eShooter.getRate();
    finder();
    SmartDashboard.putNumber("Shooter RPMs", rpm);
    SmartDashboard.putNumber("PID Output", maxSpeed(ShooterConstants.shootPID.calculate(propSpeed())));
    SmartDashboard.putNumber("Max RPMs", max2RPM);
    SmartDashboard.putNumber("Motor Ouput", maxSpeed(ShooterConstants.shootPID.calculate(eShooter.getRate()) + kOffset));
    SmartDashboard.putNumber("Proportion Ouput", propSpeed());
    SmartDashboard.putNumber("PID2 Ouput", ShooterConstants.shootPID.calculate(propSpeed()));
  }

  private Double rpm = eShooter.getRate();

  public void shootzero() {
    mShooterA.set(0.0);
    mShooterB.set(0.0);
  }

  public void shootNumMethod() {
    mShooterA.set(SmartDashboard.getNumber("shooter setter", 0.0));
    mShooterB.set(SmartDashboard.getNumber("shooter setter", 0.0));
  }

  // ^^^^^ The methods above are for testing ^^^^^

  public void shootZero() {
    mShooterA.set(0.0);
    mShooterB.set(0.0);
  }

  public void shootCancel(FeederHopper sFeederHopper) {
    sFeederHopper.feedZero();
    shootZero();
  }

  public double shootOutput(double output, double setpoint) {
    SmartDashboard.putNumber("input", output);
    SmartDashboard.putNumber("setpointIn", setpoint);
    output = output * 1.0; //TODO: Find encoder and motor ratio. The decrease in RPMs as the battery drains will make this difficult.
    setpoint = setpoint * 1.0;
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("setpointOut", setpoint);
    if (output > setpoint) {
      if (output > 1.0) {
        return 1.0;
      }
      else {
        return output;
      }
    }
    else {
      return setpoint;
    }
  }

  public void shootWithPID() {
    mShooterA.set(maxSpeed(ShooterConstants.shootPID.calculate(propSpeed()) + kOffset));
    mShooterB.set(maxSpeed(ShooterConstants.shootPID.calculate(propSpeed()) + kOffset));
  }

  private double maxSpeed(Double output) { // Add some tolerance to prevent chattering.
    if (output < 1.0 && output > 0.0) {
      return output;
    } else if (output > 1.0) {
      return 1.0;
    } else {
      return 0.0;
    }

  }

  private double propSpeed() {
    return (eShooter.getRate())/(maxRPM);
  } 

  private void finder() {
    if (max2RPM < eShooter.getRate()) {
      max2RPM = eShooter.getRate();
    }
  }
}
