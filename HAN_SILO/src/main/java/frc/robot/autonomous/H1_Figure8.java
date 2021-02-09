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

public class H1_Figure8 extends _DelayableStrafingAutoMode {
    public H1_Figure8(DriveSubsystem m_robotDrive) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),
            createSwerveCommand(m_robotDrive, "BACKWARD TO ENEMY PAIR", TrajectoryDirection.REV, 
            TrajectoryHeading.UNROTATE, 0, new double[][]
                {{0.0, 0.0, 0},
                {-0.762, -0.0},    // initial pose
                {-1.524, -0.762},    
                {-2.286, -1.524},    
                {-5.334, -1.524},    
                {-5.842, -0.762}, 
                {-6.858, -0.0},
                {-7.62, -0.762},
                {-6.858, -1.524},
                {-5.842, -0.762},
                {-5.334, 0.0},
                {-2.286, 0.0},
                {-1.524, -0.762},
                {-0.762, -1.524},
                {0.0, -1.524,0}}

            ));

        
 

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}