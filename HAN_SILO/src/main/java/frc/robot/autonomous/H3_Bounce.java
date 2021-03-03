package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;

// This is the bounce path with the robot starting at the start one and driving forwards.
public class H3_Bounce extends _DelayableStrafingAutoMode {
    public H3_Bounce(DriveSubsystem m_robotDrive) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

             createSwerveCommand(m_robotDrive, "Bounce: start zone to A3", TrajectoryDirection.FWD, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 {0.0, 0.0, -90}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field +x axis (facing the right side)
                 {  0,  -50}, 
                 { 60,  -50, 0} //A3 pylon
             }),

             createSwerveCommand(m_robotDrive, "Bounce: A3 to A6", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 {60,  -50, 0}, //A3 pylon
                 {-30, -50}, //low point
                 {-30, -140},
                 { 60, -140, 0} //A6 pylon
             }),
        
             createSwerveCommand(m_robotDrive, "Bounce: A6 to A9", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 { 60, -140, 0}, //A6 pylon
                 { -30, -140},
                 { -30, -230},
                 { 60, -230, 0} //A9 pylon
             }),
        
             createSwerveCommand(m_robotDrive, "Bounce: A9 to finish zone", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 { 60, -230, 0}, //A9 pylon
                 { 0, -230},
                 {  0, -280, -180}  //finish zone
             })
         );
/*
        createSwerveCommand(m_robotDrive, "Bounce", TrajectoryDirection.FWD, 
            TrajectoryHeading.UNROTATE, 0, new double[][]
                {{0.0, 0.0, -90},
                {0.0, -0.5},  
                {0.0, -0.75},   
               {0.0, -1.0, -90}}
        ));
 */
        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}  

/*
H3_Bounce: Bounce: ﻿
﻿﻿﻿﻿﻿﻿ trajectory duration 2.0139250705796177 ﻿
﻿﻿﻿﻿﻿﻿ state 0                 poseMetersX 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 0                 poseMetersY 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 0         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 0     velocityMetersPerSecond 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 1                 poseMetersX -0.002136005002898488 ﻿
﻿﻿﻿﻿﻿﻿ state 1                 poseMetersY -0.2499894994694076 ﻿
﻿﻿﻿﻿﻿﻿ state 1         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 1     velocityMetersPerSecond 1.0 ﻿
﻿﻿﻿﻿﻿﻿ state 2                 poseMetersX 0.009079354468835014 ﻿
﻿﻿﻿﻿﻿﻿ state 2                 poseMetersY -0.7091892456306932 ﻿
﻿﻿﻿﻿﻿﻿ state 2         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 2     velocityMetersPerSecond 0.6387664429695296 ﻿
﻿﻿﻿﻿﻿﻿ state 3                 poseMetersX -0.04772132853083087 ﻿
﻿﻿﻿﻿﻿﻿ state 3                 poseMetersY -0.9695916629254524 ﻿
﻿﻿﻿﻿﻿﻿ state 3         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 3     velocityMetersPerSecond 0.15490503530257205 ﻿
﻿﻿﻿﻿﻿﻿ state 4                 poseMetersX -1.938496805434049E-4 ﻿
﻿﻿﻿﻿﻿﻿ state 4                 poseMetersY -0.9999952613225599 ﻿
﻿﻿﻿﻿﻿﻿ state 4         poseMetersTheta Deg -86.17518209721761 ﻿
﻿﻿﻿﻿﻿﻿ state 4     velocityMetersPerSecond 0.027850141159235203 ﻿
﻿﻿﻿﻿﻿﻿ state (end)             poseMetersX -1.3877787807814457E-17 ﻿
﻿﻿﻿﻿﻿﻿ state (end)             poseMetersY -1.0 ﻿
﻿﻿﻿﻿﻿﻿ state (end)     poseMetersTheta Deg -90.0 ﻿
﻿﻿﻿﻿﻿﻿ state (end) velocityMetersPerSecond 0.0 ﻿
﻿﻿﻿﻿﻿﻿ Running actual autonomous mode --> H3_Bounce ﻿

*///         {-1.524, -0.762},    
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