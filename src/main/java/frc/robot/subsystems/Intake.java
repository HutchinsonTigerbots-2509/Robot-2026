// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public Intake() {
    mIntakeA.setNeutralMode(NeutralModeValue.Brake);
    mIntakeB.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TalonFX mIntakeA = new TalonFX(13);
  private TalonFX mIntakeB = new TalonFX(14);

  public void intakeZero() {
    mIntakeA.set(0.0);
    mIntakeB.set(0.0);
  }

  public void intakeForward() {
    mIntakeA.set(1.0);
    mIntakeB.set(-1.0);
  }

  public void intakeShoot() {
    mIntakeA.set(0.1);
    mIntakeB.set(-0.1);
  }

  public void intakeReverse() {
    mIntakeA.set(-1.0);
    mIntakeB.set(1.0);
  }
}
