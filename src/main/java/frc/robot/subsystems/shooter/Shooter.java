// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.Feeder;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonFX mShooter1 = new TalonFX(ShooterConstants.kShoot1MotorId);
  private TalonFX mShooter2 = new TalonFX(ShooterConstants.kShoot2MotorId);
  private Encoder eShooter = new Encoder(1, 2); //TODO: Put in actual encoder channels

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

  private Double num = 0.5;
  private Double rpm = eShooter.getRate();
  public void shootincrement() {
    if(num < 1) {
      num = num + 0.01;
      SmartDashboard.putNumber("shooter speed", num);
    }
    else {
      SmartDashboard.putNumber("shooter speed", num);
    }
  }
  
  public void shootincrement10() {
    if(num < 1) {
      num = num + 0.10;
      SmartDashboard.putNumber("shooter speed", num);
    }
    else {
      SmartDashboard.putNumber("shooter speed", num);
    }
  }

  public void shootresetincrement() {
    num = 0.5;
    SmartDashboard.putNumber("shooter speed", num);
  }

  public void shootnum() {
    mShooter1.set(num);
    mShooter2.set(num);
    SmartDashboard.putNumber("shooter speed", num);
    // SmartDashboard.putNumber("Shooteractual", maxSpeed(ShooterConstants.shootPID.calculate(getRPMProp(eShooter.getRate()))));
    SmartDashboard.putNumber("Shooter RPMs", rpm);
  }

  public void shootrecall() {
    SmartDashboard.putNumber("shooter speed", num);
  }

  public void shootzero() {
    mShooter1.set(0.0);
    mShooter2.set(0.0);
  }

  // ^^^^^ The methods above are for testing ^^^^^

  public void shootZero() {
    mShooter1.set(0.0);
    mShooter2.set(0.0);
  }

  public void shootCancel(Feeder sFeeder) {
    sFeeder.feedZero();
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
    mShooter1.set(maxSpeed(ShooterConstants.shootPID.calculate(propSpeed()) + kOffset));
    mShooter2.set(maxSpeed(ShooterConstants.shootPID.calculate(propSpeed()) + kOffset));
  }

  private double maxSpeed(Double output) {
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
