// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.vision.Vision;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  public TalonFX mShooter1 = new TalonFX(ShooterConstants.kShoot1MotorId);
  public TalonFX mShooter2 = new TalonFX(ShooterConstants.kShoot2MotorId);
  public Encoder eShooter = new Encoder(8, 7); //TODO: Put in actual encoder channels
  
  public Shooter() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Double num = 0.5;
  public void shootincrement() {
    if(num < 1) {
      num = num + 0.05;
      System.out.println("YOUR SPEED IS: " + num);
      SmartDashboard.putNumber("shooter speed", num);
    }
    else {
      System.out.println("MAXIMUM SPEED");
      SmartDashboard.putNumber("shooter speed", num);
    }
  }

  public void shootresetincrement() {
    num = 0.5;
    System.out.println("YOUR SPEED HAS BEEN RESET TO: " + num);
    SmartDashboard.putNumber("shooter speed", num);
  }

  public void shootnum() {
    mShooter1.set(num);
    mShooter2.set(num);
    System.out.println("YOU ARE SHOOTING AT: " + num);
    SmartDashboard.putNumber("shooter speed", num);
  }

  public void shootrecall() {
    System.out.println("YOUR SPEED IS: " + num);
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

  public void shootUnload(Drivetrain sDrivetrain, Feeder sFeeder, Vision sVision) {
    // This method should align the drivetrain with the fuel hub and then call shootWithFeeder(sFeeder)
  }

  public void shootCancel(Drivetrain sDrivetrain, Feeder sFeeder) {
    // sDrivetrain.applyRequest(() -> idle);
    sFeeder.feedZero();
  }

  public void shootWithFeeder(Feeder sFeeder) { //TODO: We are going to need to add an encoder to the shooter
    double setpoint = 0.0; //TODO: Find acceptable setpoint.
    ShooterConstants.shootPID.setSetpoint(setpoint);
    ShooterConstants.shootPID.setIZone(0.0); //TODO: Find acceptable IZone. I would like to put this in the constructor
    ShooterConstants.shootPID.setTolerance(0.0); //TODO: Find acceptable tolerance. I would like to put this in the constructor
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
}
