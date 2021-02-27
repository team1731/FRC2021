package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeSeqCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SequencerSubsystem;

// This is the bounce path with the robot starting at the start one and driving forwards.
public class H0_GalacticSearch extends _DelayableStrafingAutoMode {
    public H0_GalacticSearch(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
    LimeLightSubsystem m_limelight) {
        SequentialCommandGroup commandGroup = null;
        int field_orientation = 0;//m_limelight.getFieldOrientation(); 
        switch(field_orientation){
            case 0: //Red A (C3, D5, A6)
                commandGroup = new SequentialCommandGroup(
                    new IntakeSeqCommand(m_intake, m_sequence, true),
                    createSwerveCommand(m_robotDrive, "Red A", TrajectoryDirection.REV, 
                        TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                        {0.0, 0.0, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field -x axis (facing the start zone)
                        { -45, 0}, //Ball C3
                        { -93, 30},   
                        { -105, 30},//D5    
                        { -123, -60},    
                        { -135, -60}, // A6   
                        { -300,  -60, 0} //Endzone
                    })
                );
                break;
            case 1: //Red B (B3, D5, B7)
                commandGroup = new SequentialCommandGroup(
                    new IntakeSeqCommand(m_intake, m_sequence, true),
                    createSwerveCommand(m_robotDrive, "Red B", TrajectoryDirection.REV, 
                        TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                        {0.0, 0.0, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field -x axis (facing the start zone)
                        { -45, 0}, //Ball C3
                        { -93, 30},   
                        { -105, 30},//D5    
                        { -123, -60},    
                        { -135, -60}, // A6   
                        { -300,  -60, 0} //Endzone
                    })
                );
                break;
            case 2: //Blue A (E6, B7, C9)
                commandGroup = new SequentialCommandGroup(
                    new IntakeSeqCommand(m_intake, m_sequence, true),
                    createSwerveCommand(m_robotDrive, "Blue A", TrajectoryDirection.REV, 
                        TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                        {0.0, 0.0, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field -x axis (facing the start zone)
                        { -45, 0}, //Ball C3
                        { -93, 30},   
                        { -105, 30},//D5    
                        { -123, -60},    
                        { -135, -60}, // A6   
                        { -300,  -60, 0} //Endzone
                    })
                );
                break;
            case 3: //Blue B (D6, B8, D10)
                commandGroup = new SequentialCommandGroup(
                    new IntakeSeqCommand(m_intake, m_sequence, true),
                    createSwerveCommand(m_robotDrive, "Blue B", TrajectoryDirection.REV, 
                        TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                        {0.0, 0.0, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field -x axis (facing the start zone)
                        { -45, 0}, //Ball C3
                        { -93, 30},   
                        { -105, 30},//D5    
                        { -123, -60},    
                        { -135, -60}, // A6   
                        { -300,  -60, 0} //Endzone
                    })
                );
                break;
        }       
        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}  

/*
H0_GalacticSearch: Bounce: ﻿
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
﻿﻿﻿﻿﻿﻿ Running actual autonomous mode --> H0_GalacticSearch ﻿

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