// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

import java.util.*;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private Matrix<N3, N1> curStdDevs;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    camera = new PhotonCamera(kCameraName);
 
    photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    curStdDevs = kSingleTagStdDevs;
  }

   /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        // Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // for (var change : camera.getAllUnreadResults()) {
        //     visionEst = photonEstimator.update(change);
        //     updateEstimationStdDevs(visionEst, change.getTargets());
        // }
        // return visionEst;
        var latestResult = camera.getLatestResult();
        if (latestResult.hasTargets()) {
            Optional<EstimatedRobotPose> visionEst = photonEstimator.update(latestResult);
            updateEstimationStdDevs(visionEst, latestResult.getTargets());
            return visionEst;
        }
        return Optional.empty();
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation()
                          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(10, 10, Math.PI);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public PhotonPipelineResult getLatestResult() {
      return camera.getLatestResult();
    }

    public List<PhotonPipelineResult> getAllUnreadResults() {
      return camera.getAllUnreadResults();
    }

    public boolean hasTarget() {
      return camera.getLatestResult().hasTargets();
    }

    public double getYaw() {
      return camera.getLatestResult().getBestTarget().getYaw();
    }

    public double getDistance() {
      return PhotonUtils.calculateDistanceToTargetMeters(
        0.5, // Measured with a tape measure, or in CAD.
        1.435, // From 2024 game manual for ID 7
        Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
        Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
