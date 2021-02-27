package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class BlueB extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.11250, -0.22500),
            new Translation2d(-0.35938, -0.125),
            new Translation2d(0.08750, -0.07917)
        };
    }

}
