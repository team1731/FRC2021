package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.IntakeSeqCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JevoisVisionSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
// This is the slalom path with the robot starting by the goal and driving backwards.
public class H2_Slalom extends _DelayableStrafingAutoMode {
    public H2_Slalom(DriveSubsystem m_robotDrive) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),
            createSwerveCommand(m_robotDrive, "Slalom ", TrajectoryDirection.REV, 
            TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][]
                {{0.0, 0.0, -90},
                {-30, -0.0},    
                {-60, -30},    
                {-90, -60},    
                {-210, -60},    
                {-230, -30}, 
                {-270, -0.0},
                {-300, -30},
                {-270, -60},
                {-230, -30},
                {-210, 0.0},
                {-90, 0.0},
                {-60, -30},
                {-30, -60},
                {0.0, -60,0}}

            ));

        
 

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}