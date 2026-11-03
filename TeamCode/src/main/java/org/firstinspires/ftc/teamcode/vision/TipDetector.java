package org.firstinspires.ftc.teamcode.vision;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.RobotBaseOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TipDetector extends SubsystemBase {
    public enum TippedSide {AUDIENCE, SCORING, UNKNOWN}
    AprilTagProcessor tags;
    Telemetry telemetry;
    RobotBaseOp.Alliance alliance;
    public RobotBaseOp.TippedSide tippedSide = RobotBaseOp.TippedSide.UNKNOWN;
    Drive drive;

    public TipDetector(AprilTagProcessor tags, Telemetry telemetry, RobotBaseOp.Alliance alliance, Drive drive) {
        this.tags = tags;
        this.telemetry=telemetry;
        this.alliance = alliance;
        this.drive = drive;
    }

    public RobotBaseOp.TippedSide tip(){
        return tippedSide;
    }
        public void periodic () {
            tippedSide = RobotBaseOp.TippedSide.UNKNOWN;
            telemetry.addData("Detections", tags.getDetections().size());
            for (AprilTagDetection tag : tags.getDetections()) {

                if (tag.ftcPose == null) continue;

                if (tag instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) tag;
                    telemetry.addData("Cluster", cluster.metadata.name);
                    if (cluster.metadata.name.startsWith("B") && alliance == RobotBaseOp.Alliance.BLUE) {
                        if (drive.getPosition().getY(DistanceUnit.INCH) < 48 && cluster.metadata.shortName.contains("AUDIENCE")) {
                            tippedSide = RobotBaseOp.TippedSide.AUDIENCE;
                        } else if (drive.getPosition().getY(DistanceUnit.INCH) > 96 && cluster.metadata.shortName.contains("SCORING")) {
                            tippedSide = RobotBaseOp.TippedSide.SCORING;
                        }
                    } else if (cluster.metadata.name.startsWith("R") && alliance == RobotBaseOp.Alliance.RED) {
                        if (drive.getPosition().getY(DistanceUnit.INCH) < 48 && cluster.metadata.shortName.contains("AUDIENCE")) {
                            tippedSide = RobotBaseOp.TippedSide.AUDIENCE;
                        } else if (drive.getPosition().getY(DistanceUnit.INCH) > 96 && cluster.metadata.shortName.contains("SCORING")) {
                            tippedSide = RobotBaseOp.TippedSide.SCORING;
                        }
                    }
                    telemetry.addData("Tipped Side:", tippedSide);
                }
            }
        }
    }

