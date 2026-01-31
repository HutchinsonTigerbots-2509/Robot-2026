// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  public static Double num = 0.5;
    public static PIDController shootPID;
  
    public Shooter() {}
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    static public TalonFX mShooter1 = new TalonFX(ShooterConstants.kShoot1MotorId);
    static public TalonFX mShooter2 = new TalonFX(ShooterConstants.kShoot2MotorId);
  
    public static void shootpositive1() {
      mShooter1.set(1);
      mShooter2.set(1);
    }
  
    public static void shootpositive95() {
      mShooter1.set(0.95);
      mShooter2.set(0.95);
    }
  
    public static void shootpositive875() {
      mShooter1.set(0.875);
      mShooter2.set(0.875);
    }
  
    public static void shootpositive8() {
      mShooter1.set(0.8);
      mShooter2.set(0.8);
    }
  
    public static void shootpositive725() {
      mShooter1.set(0.725);
      mShooter2.set(0.725);
    }
  
    public static void shootpositive65() {
      mShooter1.set(0.65);
      mShooter2.set(0.65);
    }
  
    public static void shootpositive575() {
      mShooter1.set(0.575);
      mShooter2.set(0.575);
    }
  
    public static void shootpositive5() {
      mShooter1.set(0.5);
      mShooter2.set(0.5);
    }
  
    public static void shootzero() {
      mShooter1.set(0);
      mShooter2.set(0);
    }
  
    public static void shootnegative() {
      mShooter1.set(-1);
      mShooter2.set(-1);
    }
  
    public static void shootincrement() {
      if(num < 0) {
        num = num + 0.05;
        System.out.println("YOUR SPEED IS: " + num);
      }
      else {
        System.out.println("MAXIMUM SPEED");
      }
    }

    public static void shootresetincrement() {
      num = 0.5;
      System.out.println("YOUR SPEED HAS BEEN RESET TO: " + num);
    }

    public static void shoot() {
      mShooter1.set(num);
      mShooter2.set(num);
      System.out.println("YOU ARE SHOOTING AT: " + num);
    }

  // public static void shootvariable(Double num) {
  //   mShooter1.set(num);
  //   mShooter2.set(num);
  // }

  // public static PIDController shootPID = new PIDController(
  //   kShootP, 
  //   kShootI, 
  //   kShootD 
  //   // IDConstants.kDriveUpdateRate
  // );

}
