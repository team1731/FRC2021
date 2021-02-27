package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RedA extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.14375, -0.68333),
            new Translation2d(-0.61082, -0.23622),
            new Translation2d(0, 0) //Need to rescan these values
        };
    }

}
