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

public class B1_BarrelRacing extends _DelayableStrafingAutoMode {
    public B1_BarrelRacing(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
            ShootClimbSubsystem m_shootclimb) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            // ENEMY PAIR
            createSwerveCommand(m_robotDrive, "BARREL RACING", TrajectoryDirection.REV, 
                                TrajectoryHeading.UNROTATE, -31, new double[][]
                {{0.0, 0.0, 0},
                {-41.8,  -9.6},    // 1
                {-35.4,  -3.2},    // 2
                {-38.6,   9.6},    // 3
                {-16.1,  32.1},    // 4
                { 22.5,  16.1},    // 5
                {  9.6, -28.9},    // 6
                {-22.5, -19.3},    // 7
                {-19.3,   0.0},    // 8
                {-32.1,  -3.2},    // 9
                {-28.9,  -6.4},    // 10
                {-32.1, -16.1},    // 11
                { -6.4, -22.5},    // 12
                { 28.9,  -6.4},    // 13
                { 16.1,  16.1},    // 14
                { -3.2,  22.5},    // 15
                {-19.3,  32.1},    // 16
                {-35.4,  22.5},    // 17
                {-35.4,   6.4},    // 18
                {-19.3, -28.9},    // 19
                {-22.5, -22.5},    // 20
                { 41.8,   0.0},    // 21
                {  3.2,  -6.4},    // 22
                { 32.1,  -6.4},    // 23
                { 38.6,  -3.2},    // 24
                { 41.8,   0.0},    // 25
                { 35.4,   0.0},    // 26
                {54.6, 0.0, 0}}    // 27 - final pose
            )
        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}