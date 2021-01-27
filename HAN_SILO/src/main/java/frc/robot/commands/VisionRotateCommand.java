package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class VisionRotateCommand extends CommandBase {

    private final LimeLightSubsystem m_vision;
    private final DriveSubsystem m_drive;
    private final XboxController m_driverController;

    public VisionRotateCommand(LimeLightSubsystem visionSubsystem, DriveSubsystem driveSubsystem, XboxController driveController){
        m_vision = visionSubsystem;
        m_drive = driveSubsystem;
        m_driverController = driveController;
    }

    @Override
    public void initialize(){
        m_drive.setVisionHeadingOverride(true);
        m_vision.enableLED();
    }

    @Override
    public void execute(){
        if(m_vision.hasTarget()){
            double targetAngle = m_drive.getHeading() - m_vision.getLastTarget().getY();
            SmartDashboard.putNumber("Vis_TargetAngle", targetAngle);
            m_drive.setHeadingControllerGoal(targetAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_drive.setVisionHeadingOverride(false);
        m_vision.disableLED();
    }

    @Override
    public boolean isFinished(){
        //When the right bumper is released, stop the command
        return !m_driverController.getRawButton(XboxConstants.kRBumper);
    } 

}