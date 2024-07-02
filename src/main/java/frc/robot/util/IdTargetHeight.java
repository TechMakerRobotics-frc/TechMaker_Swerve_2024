package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class IdTargetHeight {
    
    private double targetHeight;

    public double idToHeight(int tagId){
        if (tagId == 1 
            || tagId == 2 
            || tagId == 5 
            || tagId == 6 
            || tagId == 9 
            || tagId == 10
        ) { targetHeight = Units.inchesToMeters(53.38);
        } else if (tagId == 3 
            || tagId == 4 
            || tagId == 7 
            || tagId == 8
            ) { targetHeight = Units.inchesToMeters(57.13);
        } else if (tagId == 11
            || tagId == 12
            || tagId == 13
            || tagId == 14
            || tagId == 15
            || tagId == 16
            ) { targetHeight = Units.inchesToMeters(52.00);
        } else {
            System.out.println("Id para AprilTag Inválida");
        };
        return targetHeight;
    }
}
