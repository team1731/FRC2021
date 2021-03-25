package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeExtend;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.IntakeSeqCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

// This is the bounce path with the robot starting at the start one and driving forwards.
public class H0_GalacticSearch extends _DelayableStrafingAutoMode {
    Pose2d initialPose;
    Integer field_orientation;

    @Override
    public Pose2d getInitialPose(){
        return initialPose;
    }

    @Override
    public Integer getFieldOrientation(){
        return field_orientation;
    }

    public H0_GalacticSearch(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
    LimeLightSubsystem m_limelight, ShootClimbSubsystem m_shootclimb) {
        SequentialCommandGroup commandGroup = null;
        Pose2d initialPoseTrajectory;
        field_orientation = m_limelight.getFieldOrientation(); 
        Path trajectoryPath = null;
        String trajectoryName = "";
        switch(field_orientation){
            case 0: //Red A (C3, D5, A6)
                SmartDashboard.putString("SelectedGalactic", "RedA");
                System.out.println("\nConstructing RedA auto\n");
                trajectoryName = "RedA: entire path";
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/RedPathA.wpilib.json");
                break;
            case 1: //Red B (B3, D5, B7)
                SmartDashboard.putString("SelectedGalactic", "RedB");
                System.out.println("\nConstructing RedB auto\n");
                trajectoryName = "RedB: entire path";
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/RedPathB.wpilib.json");
                break;
            case 2: //Blue A (E6, B7, C9)
                SmartDashboard.putString("SelectedGalactic", "BlueA");
                System.out.println("\nConstructing BlueA auto\n");
                trajectoryName = "BlueA: entire path";
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/BluePathA.wpilib.json");
                break;
            case 3: //Blue B (D6, B8, D10)
                SmartDashboard.putString("SelectedGalactic", "BlueB");
                System.out.println("\nConstructing BlueB auto\n");
                trajectoryName = "BlueB: entire path";
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/BluePathB.wpilib.json");
                break;
        }       
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            initialPoseTrajectory = trajectory.getInitialPose();
            initialPose = new Pose2d(initialPoseTrajectory.getX(), initialPoseTrajectory.getY(), Rotation2d.fromDegrees(180.0));

            m_robotDrive.resetOdometry(initialPose); //because PathWeaver path uses absolute field coords
            commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),
                new ParallelCommandGroup(
                    new IntakeSeqCommand(m_intake, m_sequence, true).withTimeout(6),
                    createSwerveCommand(m_robotDrive, trajectoryName, 180.0, trajectory)
                ),
                new IntakeRetract(m_intake),
                new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(5)
            );
        } catch (IOException ex){
            System.out.println("Path not found: " + trajectoryPath);
            DriverStation.reportError("Unable to open trajectory " + trajectoryName, ex.getStackTrace());
            commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()));
        } finally {
            System.out.println("Path: " + trajectoryPath);
        }
        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}  
