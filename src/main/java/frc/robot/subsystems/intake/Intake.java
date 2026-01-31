// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  static public TalonFX mIntake = new TalonFX(IntakeConstants.kIntakeMotorId);

  public static void intakepositive() {
    mIntake.set(1);
  }

  public static void intakezero() {
    mIntake.set(0);
  }

  public static void intakenegative1() {
    mIntake.set(-0.4);
  }

  public static void intakenegative75() {
    mIntake.set(-0.35);
  }

  public static void intakenegative5() {
    mIntake.set(-0.3);
  }

  public static void intakenegative25() {
    mIntake.set(-0.25);
  }
}
