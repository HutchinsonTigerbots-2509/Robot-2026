// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public TalonFX mIntake = new TalonFX(IntakeConstants.kIntakeMotorId);

  public void intakeForward() {
    mIntake.set(-0.2);
  }

  public void intakeZero() {
    mIntake.set(0.0);
  }

  // VVVVV Methods below are for testing VVVVV

  public Double num = -0.10;
  public void intakeincrement() {
    if(num > -1) {
      num = num - 0.01;
      System.out.println("YOUR SPEED IS: " + num);
      SmartDashboard.putNumber("intake speed", num);
    }
    else {
      System.out.println("MAXIMUM SPEED");
      SmartDashboard.putNumber("intake speed", num);
    }
  }

  public void intakeresetnum() {
    num = -0.10;
    System.out.println("YOUR SPEED HAS BEEN RESET TO: " + num);
    SmartDashboard.putNumber("intake speed", num);
  }

  public void intakenum() {
    mIntake.set(num);
    System.out.println("YOU ARE INTAKING AT: " + num);
    SmartDashboard.putNumber("intake speed", num);
  }

  public void intakerecall() {
    System.out.println("YOUR SPEED IS: " + num);
    SmartDashboard.putNumber("intake speed", num);
  }
}
