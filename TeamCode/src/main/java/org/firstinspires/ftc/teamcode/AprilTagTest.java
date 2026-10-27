package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

@TeleOp(name = "AprilTag Test")
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        AprilTagProcessor tags = new AprilTagProcessor.Builder().build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tags)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Detections", tags.getDetections().size());

            for (AprilTagDetection tag : tags.getDetections()) {

                if (tag.ftcPose == null) continue;

                if (tag instanceof AprilTagSingleDetection) {
                    telemetry.addData("Tag ID",
                            ((AprilTagSingleDetection) tag).id);
                } else {
                    telemetry.addData("Cluster", tag.metadata.name);
                }

                telemetry.addData("Distance", "%.1f inches", tag.ftcPose.range);
                telemetry.addData("Bearing", "%.1f degrees", tag.ftcPose.bearing);
                telemetry.addData("Roll", "%.1f degrees", tag.ftcPose.roll);
                telemetry.addData("Pitch", "%.1f degrees", tag.ftcPose.pitch);
                telemetry.addData("Yaw", "%.1f degrees", tag.ftcPose.yaw);
                telemetry.addLine();
            }

            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}
