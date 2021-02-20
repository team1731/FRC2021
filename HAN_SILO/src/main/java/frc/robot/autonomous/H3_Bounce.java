package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;

// This is the bounce path with the robot starting at the start one and driving forwards.
public class H3_Bounce extends _DelayableStrafingAutoMode {
    public H3_Bounce(DriveSubsystem m_robotDrive) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

        //     createSwerveCommand(m_robotDrive, "Bounce: start zone to A3", TrajectoryDirection.REV, 
        //         TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
        //         {0.0, 0.0, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field +x axis (facing the right side)
        //         {  0,  -10},    
        //         { 10,  -20},    
        //         { 20,  -30},    
        //         { 30,  -35},    
        //         { 40,  -40}, 
        //         { 50,  -45},
        //         { 60,  -50, 0} //A3 pylon
        //     }),

        //     createSwerveCommand(m_robotDrive, "Bounce: A3 to A6", TrajectoryDirection.REV, 
        //         TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
        //         {60,  -50, 0}, //A3 pylon

        //         {-30,  -80},
        //         {-60, -110}, //low point
        //         {-30, -140},

        //         { 30, -140},
        //         { 60, -140, 0} //A6 pylon
        //     }),
        
        //     createSwerveCommand(m_robotDrive, "Bounce: A6 to A9", TrajectoryDirection.REV, 
        //         TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
        //         { 60, -140, 0}, //A6 pylon
        //         { 30, -150},


        //         {-30, -160},
        //         {-55, -170}, //low points
        //         {-55, -185},
        //         {-55, -200}, //low points
        //         {-30, -210},

        //         { 30, -220},
        //         { 60, -230, 0} //A9 pylon
        //     }),
        
        //     createSwerveCommand(m_robotDrive, "Bounce: A9 to finish zone", TrajectoryDirection.REV, 
        //         TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
        //         { 60, -230, 0}, //A9 pylon
        //         { 30, -220},

        //         { 30, -240}, 
        //         { 10, -260}, 
        //         {  0, -280, 0}  //finish zone
        //     })
        // );

        createSwerveCommand(m_robotDrive, "Bounce", TrajectoryDirection.REV, 
            TrajectoryHeading.UNROTATE, 0, new double[][]
                {{0.0, 0.0, 0},
                {1.0, -0.0},    
        //         {-0.762, -0.0},    
        //         {-1.524, -0.762},    
        //         {-2.286, -1.524},    
        //         {-5.334, -1.524},    
        //         {-5.842, -0.762}, 
        //         {-6.858, -0.0},
        //         {-7.62, -0.762},
        //         {-6.858, -1.524},
        //         {-5.842, -0.762},
        //         {-5.334, 0.0},
        //         {-2.286, 0.0},
        //         {-1.524, -0.762},
        //         {-0.762, -1.524},
               {1.0, -0.0, 0}}
        ));
 
        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}