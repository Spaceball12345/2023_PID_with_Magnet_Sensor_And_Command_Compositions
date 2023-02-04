
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class sub_Shoulder extends SubsystemBase {

  public static ShuffleboardTab myTab = Shuffleboard.getTab("PID_Tuning");
  public static GenericEntry Shoulder_kP = myTab.add("Turret_kP", 1).getEntry();
  //public static GenericEntry Shoulder_kI = myTab.add("Turret_kI", 1).getEntry(); 
  //public static GenericEntry Shoulder_kD = myTab.add("Turret_kD", 1).getEntry();
  public static GenericEntry Shoulder_MaxOutput = myTab.add("Shoulder Max Output", 1).getEntry();


  public static WPI_TalonFX Shoulder_motor = new WPI_TalonFX(Constants.ShoulderConstants.ShoulderMotorID);
  public static DigitalInput Shoulder_Magnetic_Sensor = new DigitalInput(Constants.ShoulderConstants.Magnetic_Sensor_Pin);



  public static void Set_Shoulder_Motor_Up() {
    Shoulder_motor.configFactoryDefault();
    Shoulder_motor.config_kP(0, 0.1);
    Shoulder_motor.config_kI(0, 0);
    Shoulder_motor.config_kD(0, 0);
    Shoulder_motor.setNeutralMode(NeutralMode.Coast);
    Shoulder_motor.setStatusFramePeriod(StatusFrame.Status_1_General, 25);
    Shoulder_motor.configClosedLoopPeakOutput(0, 0);
    Shoulder_motor.configForwardSoftLimitThreshold(15000);
    Shoulder_motor.configForwardSoftLimitEnable(true);
    //Send_Shoulder_PID_Variables();
    //motor.configClosedloopRamp(0.5);
    // This resets the encoder to 0 when the forward limit switch is hit
    //motor.configClearPositionOnLimitF(true, 0);
  }

  public static void Send_Shoulder_PID_Variables() {
    Shoulder_motor.config_kP(0, sub_Shoulder.Shoulder_kP.getDouble(0.0));
    //Shoulder_motor.config_kI(0, sub_Shoulder.Shoulder_kI.getDouble(0.0));
    //Shoulder_motor.config_kD(0, sub_Shoulder.Shoulder_kD.getDouble(0.0));
    Shoulder_motor.configClosedLoopPeakOutput(0, sub_Shoulder.Shoulder_MaxOutput.getDouble(0.0));
    //Shoulder_motor.config_kP(0, 0.1);
    //Shoulder_motor.config_kI(0, 0);
    //Shoulder_motor.config_kD(0, 0);
    //Shoulder_motor.configClosedLoopPeakOutput(0,0.8);

  }

  public static void Shoulder_Reset_Encoder() {
    Shoulder_motor.getSensorCollection().setIntegratedSensorPosition(0, 10);
  }

  public sub_Shoulder() {
    Set_Shoulder_Motor_Up();
  }


  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Encoder Reading", Shoulder_motor.getSensorCollection().getIntegratedSensorPosition());
    //SmartDashboard.putNumber("Shoulder Encoder Speed", Shoulder_motor.getSensorCollection().getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Shoulder Error", Shoulder_motor.getClosedLoopError());
    //SmartDashboard.putBoolean("Shoulder Magnetic Sensor", Shoulder_Magnetic_Sensor.get());

    SmartDashboard.putNumber("Shoulder Forward Limit Switch", Shoulder_motor.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Shoulder Reverse Limit Switch", Shoulder_motor.isRevLimitSwitchClosed());

    //SmartDashboard.putNumber("Shoulder Motor Temperature", Shoulder_motor.getTemperature());
    //SmartDashboard.putNumber("Shoulder Supply Current", Shoulder_motor.getSupplyCurrent());
    //SmartDashboard.putString("Shoulder Control Mode", Shoulder_motor.getControlMode().toString());
    //SmartDashboard.putNumber("Shoulder Bus Voltage", Shoulder_motor.getBusVoltage());

  }
}
