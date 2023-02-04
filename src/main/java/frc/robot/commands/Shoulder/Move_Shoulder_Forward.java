package frc.robot.commands.Shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.sub_Shoulder;

public class Move_Shoulder_Forward extends CommandBase {

  public Move_Shoulder_Forward(sub_Shoulder turret) {
    addRequirements(turret);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    sub_Shoulder.Send_Shoulder_PID_Variables();

    sub_Shoulder.Shoulder_motor.set(ControlMode.Position, Constants.ShoulderConstants.ShoulderForwardPosition);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
