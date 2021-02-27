package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

public class BlueA extends _GalacticAutoMode {

    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(7, 10),
            new Translation2d(12, 54),
            new Translation2d(0.5, 1),
        };
    }

    public BlueA(DriveSubsystem m_robotDrive,
            SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb) {

                //Do an auto path for this set of cells

                
                //This is all left over from T5_ShootDriveBackwards
            /*
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            // SHOOT 3
            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),

            new WaitCommand(3),

            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(2),

            new WaitCommand(getSecondaryDelaySeconds()),

            createSwerveCommand(m_robotDrive, "BACKWARD 1 METER", TrajectoryDirection.REV, 
                                TrajectoryHeading.DO_NOTHING, 0, new double[][]
                {{0, 0, 0},    // initial pose
                {-0.5, 0},     // waypoint(s)
                {-1, 0, 0}}    // final pose
            )
        );

        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
        */
    }
}