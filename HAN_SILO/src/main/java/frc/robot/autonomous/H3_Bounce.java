package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;

// This is the bounce path with the robot starting at the start one and driving forwards.
public class H3_Bounce extends _DelayableStrafingAutoMode {
    public H3_Bounce(DriveSubsystem m_robotDrive) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),
            createSwerveCommand(m_robotDrive, "Bounce", TrajectoryDirection.REV, 
            TrajectoryHeading.UNROTATE, 0, new double[][]
                {{0.0, 0.0, 0},
                {-0.762, -0.0},    
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