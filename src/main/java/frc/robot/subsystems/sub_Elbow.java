// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class sub_Elbow extends SubsystemBase {

  //public static ShuffleboardTab myTab = Shuffleboard.getTab("PID_Tuning");
  //public static GenericEntry Elbow_kP = myTab.add("Turret_kP", 1).getEntry();
  //public static GenericEntry Elbow_kI = myTab.add("Turret_kI", 1).getEntry(); 
  //public static GenericEntry Elbow_kD = myTab.add("Turret_kD", 1).getEntry();
  //public static GenericEntry Elbow_MaxOutput = myTab.add("Shoulder Max Output", 1).getEntry();
  public static WPI_TalonFX Elbow_motor = new WPI_TalonFX(10);

  private static void Reset_Elbow_Motor(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    Send_Elbow_PID_Variables();
    //motor.config_kP(0, 0.1);
    //motor.config_kI(0, 0);
    //motor.config_kD(0, 0);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 25);
    motor.configClosedLoopPeakOutput(0, 0.1);
    //motor.configClosedloopRamp(0.5);
  }

  public static void Send_Elbow_PID_Variables() {
    //Elbow_motor.config_kP(0, sub_Elbow.Elbow_kP.getDouble(0.0));
    //Elbow_motor.config_kI(0, sub_Elbow.Elbow_kI.getDouble(0.0));
    //Elbow_motor.config_kD(0, sub_Elbow.Elbow_kP.getDouble(0.0));
    //Elbow_motor.configClosedLoopPeakOutput(0, sub_Elbow.Elbow_MaxOutput.getDouble(0.0));
    Elbow_motor.config_kP(0, 0.01);
    Elbow_motor.config_kI(0, 0);
    Elbow_motor.config_kD(0, 0);
    Elbow_motor.configClosedLoopPeakOutput(0, .2);
  }

  public static void Elbow_Reset_Encoder() {
    Elbow_motor.getSensorCollection().setIntegratedSensorPosition(0, 10);
  }

  /** Creates a new sub__Elbow. */
  public sub_Elbow() {
    Reset_Elbow_Motor(Elbow_motor);
  }

  @Override
  public void periodic() {
    /* 
    SmartDashboard.putNumber("Elbow Encoder Reading", Elbow_motor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("Elbow Encoder Speed", Elbow_motor.getSensorCollection().getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Elbow Error", Elbow_motor.getClosedLoopError());

    SmartDashboard.putNumber("Elbow Forward Limit Switch", Elbow_motor.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Elbow Reverse Limit Switch", Elbow_motor.isRevLimitSwitchClosed());

    SmartDashboard.putNumber("Elbow Motor Temperature", Elbow_motor.getTemperature());
    SmartDashboard.putNumber("Elbow Supply Current", Elbow_motor.getSupplyCurrent());
    SmartDashboard.putString("Elbow Control Mode", Elbow_motor.getControlMode().toString());
    SmartDashboard.putNumber("Elbow Bus Voltage", Elbow_motor.getBusVoltage());
    */
  }
}
