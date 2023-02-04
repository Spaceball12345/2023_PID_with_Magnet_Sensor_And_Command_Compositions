package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Elbow.Elbow_Manual_Control;
import frc.robot.commands.Shoulder.Move_Shoulder_Back;
import frc.robot.commands.Shoulder.Move_Shoulder_Forward;
import frc.robot.commands.Shoulder.Reset_Shoulder_Encoder;
import frc.robot.commands.Shoulder.Shoulder_Manual_Control;
import frc.robot.subsystems.sub_Elbow;
import frc.robot.subsystems.sub_Shoulder;

public class RobotContainer {

  private final sub_Shoulder m_Shoulder = new sub_Shoulder();
  private final sub_Elbow m_Elbow = new sub_Elbow();

  public static XboxController m_controller = new XboxController(Constants.OIConstants.kDriverControllerPort);

  public static JoystickButton A = new JoystickButton(m_controller, 1);
  public static JoystickButton B = new JoystickButton(m_controller, 2);
  public static JoystickButton X = new JoystickButton(m_controller, 3);
  public static JoystickButton Y = new JoystickButton(m_controller, 4);
  public static JoystickButton LB = new JoystickButton(m_controller, 5);
  public static JoystickButton RB = new JoystickButton(m_controller, 6);  
  public static JoystickButton Select = new JoystickButton(m_controller, 7);
  public static JoystickButton Start = new JoystickButton(m_controller, 8);
  public static JoystickButton LStickButton = new JoystickButton(m_controller, 9);
  public static JoystickButton RStickButton = new JoystickButton(m_controller, 10);


  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_Shoulder.setDefaultCommand(new Shoulder_Manual_Control(m_Shoulder));
    m_Elbow.setDefaultCommand(new Elbow_Manual_Control(m_Elbow));

    X.whenPressed(new Move_Shoulder_Back(m_Shoulder));
    A.whenPressed(new Shoulder_Manual_Control(m_Shoulder));
    B.whenPressed(new Move_Shoulder_Forward(m_Shoulder));
    Start.whenPressed(new Reset_Shoulder_Encoder(m_Shoulder));



  }

}
