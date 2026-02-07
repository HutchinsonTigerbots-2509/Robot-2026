// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.vision.Vision;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonFX mShooter1 = new TalonFX(ShooterConstants.kShoot1MotorId);
  private TalonFX mShooter2 = new TalonFX(ShooterConstants.kShoot2MotorId);
  private Encoder eShooter = new Encoder(1, 2); //TODO: Put in actual encoder channels
  
  public Shooter() {
    SmartDashboard.putNumber("Shooteractual", 0);
    SmartDashboard.putNumber("shooter speed", num);
    SmartDashboard.putNumber("Shooter RPMs", rpm);
    ShooterConstants.shootPID.setTolerance(0.0); //TODO: Find acceptable tolerance.
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Shooter RPMs", rpm);
    rpm = eShooter.getRate();
  }

  private Double num = 0.5;
  private Double rpm = eShooter.getRate();
  private Double maxRPM;
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
    System.out.println(rpm);
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

  public void shootUnload(Feeder sFeeder) { // I think this method will require RunCommands
    shootWithFeeder(sFeeder);
  }

  public void shootCancel(Feeder sFeeder) {
    sFeeder.feedZero();
    shootZero();
  }

  public void shootWithFeeder(Feeder sFeeder) { //TODO: We are going to need to add an encoder to the shooter
    double setpoint = 0.0; //TODO: Find acceptable setpoint. If we shoot from a set distance, this can be moved to the constructor.
    ShooterConstants.shootPID.setSetpoint(setpoint);
    mShooter1.set(shootOutput(ShooterConstants.shootPID.calculate(eShooter.getRate()), setpoint));
    mShooter2.set(shootOutput(ShooterConstants.shootPID.calculate(eShooter.getRate()), setpoint));
    if (ShooterConstants.shootPID.atSetpoint()) {
      sFeeder.feedToShoot();
    }
    else {
      sFeeder.feedZero();
    }
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

  public void shoot18(Feeder sFeeder) {
    mShooter1.set(maxSpeed(ShooterConstants.shootPID.calculate(getRPMProp(eShooter.getRate()))));
    mShooter2.set(maxSpeed(ShooterConstants.shootPID.calculate(getRPMProp(eShooter.getRate()))));
    SmartDashboard.putNumber("Shooteractual", maxSpeed(ShooterConstants.shootPID.calculate(getRPMProp(eShooter.getRate()))));
    SmartDashboard.putNumber("Shooter RPMs", rpm);
    System.out.println(rpm);
    if (ShooterConstants.shootPID.atSetpoint()) {
      sFeeder.feednum();
    }
    else {
      sFeeder.feedzero();
    }
  }

  private double maxSpeed(Double output) {
    if (ShooterConstants.shootPID.calculate(eShooter.getRate()) < 1.0) {
      return output;
    }
    return 1.0;
  }

  private double getRPMProp(double rpm) {
    return rpm / maxRPM;
  }
}
