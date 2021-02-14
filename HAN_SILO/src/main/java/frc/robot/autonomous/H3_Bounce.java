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
            TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] //these are INCHES
                {{0.0, 0.0, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field +x axis (facing the right side)
                {  0,  -10},    
                { 10,  -20},    
                { 20,  -30},    
                { 30,  -35},    
                { 40,  -40}, 
                { 50,  -45},
                { 60,  -50}, //A3 pylon

                {-30,  -80},
                {-60, -110}, //low point
                {-30, -140},

                { 30, -140},
                { 60, -140}, //A6 pylon
                { 30, -150},


                {-30, -160},
                {-55, -170}, //low points
                {-55, -185},
                {-55, -200}, //low points
                {-30, -210},

                { 30, -220},
                { 60, -230}, //A9 pylon
                { 30, -220},

                { 30, -240}, 
                { 10, -260}, 
                {  0, -280, 0}} //finish zone
            ));

        
 

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}