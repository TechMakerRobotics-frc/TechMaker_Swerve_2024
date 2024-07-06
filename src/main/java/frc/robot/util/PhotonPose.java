package frc.robot.util;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.UtilConstants.VisionConstants;

public class PhotonPose {

    private final PhotonTags vision;
    private final Drive drive;

    double targetPitch = 0;
    double targetPose = 0;

    public PhotonPose(PhotonTags vision, Drive drive) {
        this.vision = vision;
        this.drive = drive;
    }

    // Calculate robot's field relative pose
    Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
        VisionConstants.CAMERA_HEIGHT_METERS,
        IdTargetHeight.idToHeight(vision.getTargetCurrentId()),
        VisionConstants.CAMERA_PITCH_RADIANS, 
        targetPitch, 
        Rotation2d.fromDegrees(-vision.getPipelineToPose()), 
        drive.getRotation(), 
        targetPose,        //Falta adicionar um pose2d Aqui para funcionar
        vision.getCamera() //Ainda falta adicionar um dado transform2d aqui para funcionar.
        );
    
}
