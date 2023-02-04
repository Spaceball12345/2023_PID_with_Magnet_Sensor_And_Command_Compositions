// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandCompositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Elbow.Elbow_Lower_Arm;
import frc.robot.commands.Shoulder.Move_Shoulder_Back;
import frc.robot.commands.Shoulder.Move_Shoulder_Forward;
import frc.robot.subsystems.sub_Elbow;
import frc.robot.subsystems.sub_Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToLockedPosition extends ParallelCommandGroup {
  /** Creates a new GoToLockedPosition. */
    
  public GoToLockedPosition(sub_Shoulder m_Shoulder_Back, sub_Elbow m_Elbow_Lower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Move_Shoulder_Back(m_Shoulder_Back),
      new Elbow_Lower_Arm(m_Elbow_Lower)
      );
  }
}