/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.BlueA;
import frc.robot.autonomous.BlueB;
import frc.robot.autonomous.F1_Move_Forward;
import frc.robot.autonomous.L1_EnemyPair_Front3;
import frc.robot.autonomous.M1_Shoot3_Front3_Shoot3;
import frc.robot.autonomous.M3_Shoot3_Buddy5;
import frc.robot.autonomous.R1_WholeSide10;
import frc.robot.autonomous.R2_Shoot3_FriendlyTriple;
import frc.robot.autonomous.RedA;
import frc.robot.autonomous.RedB;
import frc.robot.autonomous.T3_DriveForwardIntakeDriveBackward;
import frc.robot.autonomous.T4_ShootDriveForward;
import frc.robot.autonomous.T5_ShootDriveBackward;
import frc.robot.autonomous._GalacticAutoMode;
import frc.robot.autonomous.H1_Figure8;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.autonomous._NotImplementedProperlyException;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.vision.LimeTargetInfo;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XboxConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  //XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  private LedStringSubsystem m_ledstring;
  private DriveSubsystem m_robotDrive;
  private IntakeSubsystem m_intake;
  private ShootClimbSubsystem m_shootclimb;
  private SequencerSubsystem m_sequencer;
  private LimeLightSubsystem m_vision;

  // Controller Triggers
  public enum HanTriggers {
    DR_TRIG_LEFT, DR_TRIG_RIGHT, OP_TRIG_LEFT, OP_TRIG_RIGHT
  }
  public enum HanMode { MODE_SHOOT, MODE_CLIMB }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @throws _NotImplementedProperlyException
   */
  public RobotContainer(LedStringSubsystem m_ledstring, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequencer,
      ShootClimbSubsystem m_shootclimb, LimeLightSubsystem m_vision) {
    this.m_ledstring = m_ledstring;
    this.m_robotDrive = m_robotDrive;
    this.m_intake = m_intake;
    this.m_sequencer = m_sequencer;
    this.m_vision = m_vision;
    this.m_shootclimb = m_shootclimb;

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive.drive(
            // Get the x speed. We are inverting this because Xbox controllers return
            // negative values when we push forward.
            -m_driverController.getY(Hand.kLeft) * Math.abs(m_driverController.getY(Hand.kLeft)) * DriveConstants.kMaxSpeedMetersPerSecond,

            // Get the y speed or sideways/strafe speed. We are inverting this because
            // we want a positive value when we pull to the left. Xbox controllers
            // return positive values when you pull to the right by default.
            -m_driverController.getX(Hand.kLeft) * Math.abs(m_driverController.getX(Hand.kLeft)) * DriveConstants.kMaxSpeedMetersPerSecond,

            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            -m_driverController.getX(Hand.kRight) * Math.abs(m_driverController.getX(Hand.kRight)),

            -m_driverController.getY(Hand.kRight) * Math.abs(m_driverController.getY(Hand.kRight)), 
            
            true),

            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kX.value)
    // .whenPressed(new TurnToAngle(30, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the 'A' button is pressed, with a 5
    // second timeout
    // new JoystickButton(m_driverController, Button.kA.value)
    // .whenPressed(new TurnToAngleProfiled(30, m_robotDrive).withTimeout(5));

    // Activate Intake via Operator Left Axis/Trigger
    new HanTrigger(HanTriggers.DR_TRIG_RIGHT).whileActiveContinuous(new ShootSeqCommand(m_shootclimb ,m_sequencer), true);
    // Activate Intake via Operator Left Front Top - Up is Intaking, Down is Reset 
    //new JoystickButton(m_operatorController, 3).whileActiveContinuous(new IntakeSeqCommand(m_intake, m_sequencer));
    new JoystickButton(m_operatorController, 14).whileActiveContinuous(new SeqEjectCommand(m_intake, m_sequencer), true);

    //Map right bumper to rotation lock to power port
    new JoystickButton(m_driverController, XboxConstants.kRBumper)
      .whenActive(new VisionRotateCommand(m_vision, m_robotDrive, m_driverController));

    new JoystickButton(m_driverController, XboxConstants.kLBumper)
      .whenPressed(new InstantCommand(() -> { 
        SmartDashboard.putString("GalacticAuto", getGalacticAutoCommand().toString());
      }));

    new JoystickButton(m_operatorController, 8) // convert -1 to +1 TO 0 to 1
      .whileActiveContinuous(() -> m_shootclimb.spinShooter((m_operatorController.getRawAxis(4)+1)/2))
      .whenInactive(() -> m_shootclimb.stopShooting());
      //.whenActive(new InstantCommand(m_shootclimb::enableShooting, m_shootclimb))
      //.whileActiveContinuous(new JoystickShooter(m_shootclimb, () -> m_operatorController.getRawAxis(4)), false
      //.whileActiveContinuous(new ShootSeqCommand(m_shootclimb, m_sequencer, () -> m_operatorController.getRawAxis(4)), false
    //);

    // scale drive speed according to axis 5 (but only when switch 12 is UP)
    new JoystickButton(m_operatorController, 12)
      .whileActiveContinuous(() -> m_robotDrive.setDriveSpeedScaler(m_operatorController.getRawAxis(5)))
      .whenInactive(() -> m_robotDrive.setDriveSpeedScaler(1));

    new JoystickButton(m_operatorController, 15).whileActiveContinuous(
      new IntakeSeqCommand(m_intake, m_sequencer)
    );
    

    // Climbing Command - CURRENT
    new JoystickButton(m_operatorController, 9).whileActiveContinuous(
       new ClimbingCommand(m_shootclimb, () -> m_operatorController.getRawAxis(1)), true
    );


    // Climber Extend
    //new JoystickButton(m_operatorController, 1)
    // .whileActiveOnce(new InstantCommand(m_shootclimb::climbExtend, m_shootclimb));
    // Climber Retract
    //new JoystickButton(m_operatorController, 2)
    // .whileActiveOnce(new InstantCommand(m_shootclimb::climbRetract, m_shootclimb));

    // Activate Shooter via Operator Right Axis/Trigger
    //new HanTrigger(HanTriggers.DR_TRIG_RIGHT).whileActiveContinuous(new ShootSeqCommand(m_sequencer));
    // Shooting
    //new ModeTrigger(HanMode.MODE_SHOOT).whenActive(
    //  new InstantCommand(m_shootclimb::enableShooting, m_shootclimb)
    //);

    // Sequencer ejects works when button is held
   // new JoystickButton(m_operatorController, 14)
   // .whenHeld(new InstantCommand(m_sequencer::reverse, m_sequencer))
   // .whenReleased(new InstantCommand(m_sequencer::stop, m_sequencer));

    new JoystickButton(m_driverController, 7).whenPressed(new InstantCommand(() -> { m_robotDrive.resetGyro(); System.out.println("Reset gyro"); }, m_robotDrive));
    new JoystickButton(m_driverController, XboxConstants.kMenu).whenPressed(new InstantCommand(() -> { m_robotDrive.resetEncoders(); System.out.println("Reset encoders"); }, m_robotDrive));

    // Climbing Command
    //new ModeTrigger(HanMode.MODE_CLIMB).whileActiveContinuous(
    //  new ClimbingCommand(m_shootclimb, () -> m_operatorController.getRawAxis(1)), true
    //);

    /*
    // Intake & Sequencer ejects works will button is held
    new JoystickButton(m_operatorController, 1)
        .whenHeld(new ParallelCommandGroup(new InstantCommand(m_intake::eject, m_intake),
            new InstantCommand(m_sequencer::reverse, m_sequencer)))
        .whenReleased(new ParallelCommandGroup(new InstantCommand(m_intake::retract, m_intake),
            new InstantCommand(m_sequencer::stop, m_sequencer)));

    // Activate Intake via Operator Left Axis/Trigger
    //new HanTrigger(HanTriggers.OP_TRIG_LEFT).whileActiveContinuous(new IntakeSeqCommand(m_intake, m_sequencer));

    // Activate Shooter via Operator Right Axis/Trigger
    //new HanTrigger(HanTriggers.OP_TRIG_RIGHT).whileActiveOnce(new ShootSeqCommand(m_shootclimb, m_sequencer));

    // Select Shoot or Climb Mode
    new JoystickButton(m_operatorController, 3).whenPressed(new InstantCommand(m_shootclimb::modeClimb, m_shootclimb));
    // .whenReleased(new InstantCommand(m_ShootClimbSubsystem::off,
    // m_ShootClimbSubsystem));
    new JoystickButton(m_operatorController, 4)
        // .whenPressed(new InstantCommand(m_ShootClimbSubsystem::on,
        // m_ShootClimbSubsystem))
        .whenReleased(new InstantCommand(m_shootclimb::modeShoot, m_shootclimb));

    new JoystickButton(m_operatorController, 8)
    // .whenPressed(new InstantCommand(m_ShootClimbSubsystem::on,
    // m_ShootClimbSubsystem))

    
      .toggleWhenPressed(new ParallelCommandGroup[] {
        new InstantCommand(m_shootclimb::modeShoot, m_shootclimb)

    });
    */
  }

  private double getDistance(Translation2d pos1, Translation2d pos2){
    return Math.sqrt(
      Math.pow(pos2.getX() - pos1.getX(), 2) + Math.pow(pos2.getY() - pos1.getY(), 2)
    );
  }

  private double getDistance(Translation2d pos1, LimeTargetInfo targetPos){
    Translation2d pos2 = new Translation2d(targetPos.getY(), targetPos.getZ());
    return getDistance(pos1, pos2);
  }

  public _GalacticAutoMode getGalacticAutoCommand(){

    LimeTargetInfo[] ballPositions = m_vision.getBallPositions();
    _GalacticAutoMode[] modes = new _GalacticAutoMode[] {
      new RedA(m_robotDrive, null, m_shootclimb),
      new RedB(m_robotDrive, null, m_shootclimb),
      new BlueA(m_robotDrive, null, m_shootclimb),
      new BlueB(m_robotDrive, null, m_shootclimb),
    };

    double[] storedDifference = new double[modes.length];

    for(int m = 0; m < modes.length; m++){
      Translation2d[] positions = modes[m].getBallPositions();
      for(int p = 0; p < positions.length; p++) {
        storedDifference[m] += getDistance(positions[p], ballPositions[p]);
      }
    }

    int minIndex = 0;
    for(int i = 0; i < storedDifference.length; i++){
      if(storedDifference[i] < storedDifference[minIndex]){
        minIndex = i;
      }
    }

    return modes[minIndex];

/*
    Algorithm that Patrick, Christian, and David worked on


import java.util.Random;

class positionAlgorithm{
    public static void main(String[] args) {
        Random rand = new Random();
        
        double[][] positionZero = { {1.0,2.0,3.0} , {5.0,6.0,7.0} };
        double[][] positionOne = { {19.0,47.0,34.0} , {23.0,43.0,12.0} };
        double[][] positionTwo = { {21.0,25.0,43.0} , {37.0,23.0,23.0} };
        double[][] positionThree = { {12.0,32.0,43.0} , {23.0,43.0,34.0} };
       double[][] randPosition = { {rand.nextInt(50),rand.nextInt(50),rand.nextInt(50)} , {rand.nextInt(50),rand.nextInt(50),rand.nextInt(50)} };
       //double[][] randPosition = { {19.0,47.0,34.0} , {23.0,43.0,12.0} };
 
       double[] storedDifference = new double [4]; 
        
        for (int i = 1; i < 5 ; i++)
        {
            switch (i){
                case 1: 
                    for(int j = 0; j < 2; j++)
                    {
                        storedDifference[0] += difference(randPosition[0][j], positionZero[0][j], randPosition[1][j], positionZero[1][j]);
                    }
                break;

                case 2: 
                    for(int j = 0; j < 2; j++)
                    {
                        storedDifference[1] += difference(randPosition[0][j], positionOne[0][j], randPosition[1][j], positionOne[1][j]);
                    }
                break;

                case 3: 
                    for(int j = 0; j < 2; j++)
                    {
                        storedDifference[2] += difference(randPosition[0][j], positionTwo[0][j], randPosition[1][j], positionTwo[1][j]);
                    }
                break;

                case 4: 
                    for(int j = 0; j < 2; j++)
                    {
                        storedDifference[3] += difference(randPosition[0][j], positionThree[0][j], randPosition[1][j], positionThree[1][j]);
                    }
                break;
            }
        }

        System.out.println(storedDifference[0]);
        double Min = Double.min (Double.min(storedDifference[0], storedDifference[1]), Double.min(storedDifference[2], storedDifference[3]));
        int position = 0;

        if (Min == storedDifference[0]){
            position = 1;
        } else if (Min == storedDifference[1]){
            position = 2;
        } else if (Min == storedDifference[2]){
            position = 3;
        } else if (Min == storedDifference[3]){
            position = 4;
        }

        System.out.println(position);

    }

    private static double difference (double xOne, double xTwo, double yOne, double yTwo){
        double differencePosition = Math.sqrt(Math.pow((xTwo - xOne), 2.0) + Math.pow((yTwo - yOne), 2.0) );
        return differencePosition;
    } 
}
    */
  }

  public _NamedAutoMode getNamedAutonomousCommand(String autoSelected) {
    String autoMode = "";
    int initialDelaySeconds = 0;
    int secondaryDelaySeconds = 0;
    if(autoSelected.length() > 1){
      autoMode = autoSelected.substring(0, 2);
    }
    if(autoSelected.length() > 2){
      try{
        initialDelaySeconds = Integer.parseInt(autoSelected.substring(2, 2));
      }
      catch(Exception e){
        System.out.println("INITIAL DELAY did not parse -- defaulting to 0 seconds!!!");
      }
    }
    if(autoSelected.length() > 3){
      try{
        secondaryDelaySeconds = Integer.parseInt(autoSelected.substring(3, 3));
      }
      catch(Exception e){
        System.out.println("SECONDARY DELAY did not parse -- defaulting to 0 seconds!!!");
      }
    }

    _NamedAutoMode selectedAutoMode = null;

    
    try{
      selectedAutoMode = createNamedAutoMode(autoMode);
    }
    catch(_NotImplementedProperlyException e){
      System.err.println("SELECTED MODE NOT IMPLEMENTED -- DEFAULT TO F1_MOVE_FORWARD!!!");
      try{
        selectedAutoMode = new _NamedAutoMode(new F1_Move_Forward(m_robotDrive));
      }
      catch(_NotImplementedProperlyException e2){
        System.err.println("F1_Move_Forward could NOT be created -- Aborting!!!");
        return null;
      }
    }
    if(selectedAutoMode != null){
      selectedAutoMode.delayableStrafingAutoMode.setInitialDelaySeconds(initialDelaySeconds);
      selectedAutoMode.delayableStrafingAutoMode.setSecondaryDelaySeconds(secondaryDelaySeconds);
    }

    return selectedAutoMode;
  }

  private _NamedAutoMode createNamedAutoMode(String autoModeName) throws _NotImplementedProperlyException {
    switch(autoModeName){
      case "F1": return new _NamedAutoMode(new F1_Move_Forward(m_robotDrive));

      case "L1": return new _NamedAutoMode(new L1_EnemyPair_Front3(m_robotDrive, m_intake, m_sequencer, m_shootclimb));
      case "M1": return new _NamedAutoMode(new M1_Shoot3_Front3_Shoot3(m_robotDrive, m_intake, m_sequencer, m_shootclimb));
      case "M3": return new _NamedAutoMode(new M3_Shoot3_Buddy5(m_robotDrive, m_intake, m_sequencer, m_shootclimb));
      case "R1": return new _NamedAutoMode(new R1_WholeSide10(m_robotDrive, m_intake, m_sequencer, m_shootclimb));
      case "R2": return new _NamedAutoMode(new R2_Shoot3_FriendlyTriple(m_robotDrive, m_intake, m_sequencer, m_shootclimb));

      case "T3": return new _NamedAutoMode(new T3_DriveForwardIntakeDriveBackward(m_robotDrive, m_intake, m_sequencer, m_shootclimb));
      case "T4": return new _NamedAutoMode(new T4_ShootDriveForward(m_robotDrive, m_sequencer, m_shootclimb));
      case "T5": return new _NamedAutoMode(new T5_ShootDriveBackward(m_robotDrive, m_sequencer, m_shootclimb));

      case "H1": return new _NamedAutoMode(new H1_Figure8(m_robotDrive));
      
      default: 
        System.err.println("FATAL: SELECTED AUTO MODE " + autoModeName + " DOES NOT MAP TO A JAVA CLASS!!!!");
        return null;
    }
  }
    
  public class StickTrigger extends Trigger {
    public boolean get() {
      //double v = operatorController.getY(Hand.kRight);
      //v = operatorController.getX(Hand.kRight);
      //x = operatorController.getRawAxis(0);
      double y = m_operatorController.getRawAxis(1);
      return Math.abs(y) < 0.2 ? false : true; 
    }
  }

  // Enables Use of controller axis/trigger by creating a Custom Trigger
  public class HanTrigger extends Trigger {
    HanTriggers desired;
    double triggerValue = 0;

    public HanTrigger(HanTriggers selected) {
      this.desired = selected;
    }

    @Override
    public boolean get() {
      switch (desired) {
        case DR_TRIG_LEFT:
          triggerValue = m_driverController.getTriggerAxis(Hand.kLeft);
          break;
        case DR_TRIG_RIGHT:
          triggerValue = m_driverController.getTriggerAxis(Hand.kRight);
          break;
        case OP_TRIG_LEFT:
          //triggerValue = m_operatorController.getTriggerAxis(Hand.kLeft);
          break;
        case OP_TRIG_RIGHT:
          //triggerValue = m_operatorController.getTriggerAxis(Hand.kRight);
          break;
      }
      return (Math.abs(triggerValue) > 0.5);
    }
  }

  public class ModeTrigger extends Trigger {
    HanMode mode;
    boolean result;
    public ModeTrigger (HanMode mode) {
      this.mode = mode;
    }

    public boolean get() {
      boolean left = m_operatorController.getRawButton(1);
      boolean right = m_operatorController.getRawButton(12);
      switch (mode) {
        case MODE_SHOOT:
          result = ((!left) && (!right));
          break;
        case MODE_CLIMB:
          result = (left && right);
          break;
      }
      //double v = operatorController.getY(Hand.kRight);
      return result; 
    }
  }

}
