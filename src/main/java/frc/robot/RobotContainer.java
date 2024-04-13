// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.AMP.AmpShooter;
import frc.robot.commands.AMP.AmpShooterMid;
import frc.robot.commands.autonomous.StartAuto;
import frc.robot.commands.drivetrain.NorthUntilInterupt;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.team1891.common.control.AxisTrigger;
import frc.team1891.common.control.POVTrigger;
import frc.team1891.common.control.POVTrigger.POV;
import frc.team1891.common.logger.BullLogger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final DriveTrain m_DriveTrain = new DriveTrain();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

      SendableChooser<Command> m_chooser = new SendableChooser<>();
    // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command m_simpleAuto =
      new StartAuto(m_IntakeSubsystem, m_ShooterSubsystem);

      m_chooser.setDefaultOption();

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.

  // public final Intake m_intake = Intake.getInstance();
  // public final GrabbyArm m_arm = GrabbyArm.getInstance();
  // public final Conveyer m_conveyer = Conveyer.getInstance();
  // public final Grabber m_Grabber = Grabber.getInstance();
  // public final PneumaticHub m_Hub = new PneumaticHub();

  // public final MatrixLEDs m_LEDSystem = MatrixLEDs.getInstance();

  // private static final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  // This is big bad, but I'm lazy -stephen
  // public static Compressor getCompressor() {
  //   return m_compressor;
  // }

    
  public static boolean scoringForCubes = false;
  public static double setAngle = 0;
  //private final AbsoluteAngleJoystickDrive m_absoluteDrive = new AbsoluteAngleJoystickDrive(m_DriveTrain, null, null, null);

  BullLogger logger = new BullLogger("Main log", false, false);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // public static final XboxController m_driverController =
  //   new XboxController(OperatorConstants.kDriverControllerPort) {
  //     public double getRawAxis(int axis) {
  //       return MathUtil.applyDeadband(super.getRawAxis(axis), .15);
  //     };
  // };
  private final Joystick m_Joystick = new Joystick(1);
  public static final XboxController m_driverController = new XboxController(0);


  // This is big bad, but I'm lazy -stephen
  public static XboxController getController() {
    return m_driverController;
  }
  private JoystickButton m_FaceForward = new JoystickButton(m_Joystick, XboxController.Button.kLeftStick.value);
  private JoystickButton m_xwheels = new JoystickButton(m_Joystick, XboxController.Button.kStart.value);
  private AxisTrigger m_rightStickTrig = new AxisTrigger(m_Joystick, XboxController.Axis.kRightX.value,.13);
  private POVTrigger m_POVNorth = new POVTrigger(m_Joystick, POV.NORTH);

  // belt back and forth
  // grab and drop

    // DEMO BUTTON
    String number_image = "";
  public void periodic() {

  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Autos.load();

    configureBindings();
    m_chooser.setDefaultOption("StartAuto", new StartAuto(m_IntakeSubsystem, m_ShooterSubsystem)); 
    
    m_DriveTrain.setDefaultCommand(
        new RunCommand(
            () -> {
                m_Joystick.setRumble(RumbleType.kBothRumble, 0.0);
                final double DEADBAND = .2;
                double x = m_Joystick.getRawAxis(0);
                double y = m_Joystick.getRawAxis(1);
                double z = m_Joystick.getRawAxis(5);
                if (Math.abs(x) > DEADBAND) {
                  y = MathUtil.applyDeadband(y, DEADBAND*.6);
                } else {
                  y = MathUtil.applyDeadband(y, DEADBAND);
                }
                 if (Math.abs(y) > DEADBAND) {
                  x = MathUtil.applyDeadband(x, DEADBAND*.6);
                } else {
                  x = MathUtil.applyDeadband(x, DEADBAND);
                }
                z = MathUtil.applyDeadband(z, DEADBAND);

                

              if ((m_Joystick.getRawButton(0) || m_Joystick.getRawButton(1) || m_Joystick.getRawButton(15) || m_Joystick.getRawButton(14))){
                  m_DriveTrain.holonomicDrive(-y/8,-x/8,z/8,true); 
              } else if (!m_Joystick.getRawButton(15)){
                  m_DriveTrain.holonomicDrive(-y,-x,z,true);    
              }
              
            }, m_DriveTrain));
    // if(DriverStation.isDSAttached()) {
    //   firstLogo.schedule();
    // }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    double r2Threshold = 0.425;
    double l2Threshold = 0.425;

    AxisTrigger r2AxisTrigger = new AxisTrigger(m_driverController, XboxController.Axis.kRightTrigger.value, r2Threshold);
    r2AxisTrigger.whileTrue(new ShooterCommand(m_ShooterSubsystem));
    
    AxisTrigger l2AxisTrigger = new AxisTrigger(m_driverController, XboxController.Axis.kLeftTrigger.value, l2Threshold );
    l2AxisTrigger.whileTrue(new IntakeCommand(m_IntakeSubsystem));

    m_FaceForward.onTrue(new NorthUntilInterupt(m_DriveTrain,()-> m_Joystick.getRawAxis(0),() -> m_Joystick.getRawAxis(1),() -> m_rightStickTrig.getAsBoolean()));

    JoystickButton l1Button = new JoystickButton(m_driverController, 5);
    l1Button.whileTrue(new ReverseIntake(m_IntakeSubsystem));

    JoystickButton triangle = new JoystickButton(m_driverController, 4);
    triangle.whileTrue(new AmpShooter(m_ShooterSubsystem));
    
    JoystickButton circle = new JoystickButton(m_driverController, 2);
    circle.whileTrue(new AmpShooterMid(m_ShooterSubsystem));

    m_FaceForward.onTrue(new NorthUntilInterupt(m_DriveTrain,()-> m_driverController.getLeftX(),() -> m_driverController.getLeftY(),() -> m_rightStickTrig.getAsBoolean()));

   
    double tx = LimelightHelpers.getTX("");
    
    //m_alignToPlaceButton.onTrue(new DriveToPose(m_DriveTrain, ()-> m_DriveTrain.pickConeScoringArea().getPose2d(), () -> m_leftrightTrigger.or(m_forwardBack.or(m_rightStickTrig)).getAsBoolean()));
      //m_alignToPlaceButton.onTrue(new ConditionalCommand(
      //new DriveToPose(m_DriveTrain, ()-> m_DriveTrain.pickCubeScoringArea().getPose2d(), () -> m_leftrightTrigger.or(m_forwardBack.or(m_rightStickTrig)).getAsBoolean()),
      //new DriveToPose(m_DriveTrain, ()-> m_DriveTrain.pickConeScoringArea().getPose2d(), () -> m_leftrightTrigger.or(m_forwardBack.or(m_rightStickTrig)).getAsBoolean()),
      //()-> scoringForCubes));

    m_POVNorth.onTrue(
      new InstantCommand(
            () ->
            {
              if(Robot.isRedAlliance()){
                m_DriveTrain.resetAngle(180);
              }else{
                m_DriveTrain.resetAngle(0);
              }
            },
                m_DriveTrain));


    m_xwheels.onTrue(
      new RunCommand(
        ()->{
          m_DriveTrain.moduleXConfiguration();
        }, m_DriveTrain)
        .withTimeout(2)
    );

    SmartDashboard.putData("set to 90", new InstantCommand(){
      @Override
      public void initialize() {
        setAngle = 90;
      }
    });
    SmartDashboard.putData("set to 0", new InstantCommand(){
      @Override
      public void initialize() {
        setAngle = 30;
      }
    });
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
