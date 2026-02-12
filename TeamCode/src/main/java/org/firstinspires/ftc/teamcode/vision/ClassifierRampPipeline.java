package org.firstinspires.ftc.teamcode.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


// (originally from april-tag example)


public class ClassifierRampPipeline extends OpenCvPipeline
{
    private Mat grey = new Mat();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    double fx;
    double fy;
    double cx;
    double cy;

    // UNITS ARE METERS
    double tagsize;


    public ClassifierRampPipeline()
    {
        double tagsize = 0.0508; // meters (!!)
        // Lens intrinsics
        // UNITS ARE PIXELS
        // these are for C920s
        // <Calibration
        //     size="800 600"
        //     focalLength="775.79f, 775.79f"
        //     principalPoint="400.898f, 300.79f"
        //     distortionCoefficients="0.112507, -0.272067, 0, 0, 0.15775, 0, 0, 0"
        //     />
        // double fx = 775.79;
        // double fy = 775.79;
        // double cx = 400.898;
        // double cy = 300.79;


        //        <Calibration
        //    size="640 480"
        //    focalLength="622.001f, 622.001f"
        //    principalPoint="319.803f, 241.251f"
        //    distortionCoefficients="0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0"
        //    />
        double fx = 622.001;
        double fy = 622.001;
        double cx = 319.803;
        double cy = 241.251;


        // (copied from XML in this project, for C270 640x480
        //focalLength="822.317f, 822.317f"
        //principalPoint="319.495f, 242.502f"
        // double fx = 822.317;
        // double fy = 822.317;
        // double cx = 319.495;
        // double cy = 242.502;


        this.tagsize = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();
    }

    @Override
    public Mat processFrame(Mat input)
    {

    }


    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }



    Pose aprilTagPoseToOpenCvPose(AprilTagPose aprilTagPose)
    {
        Pose pose = new Pose();
        pose.tvec.put(0,0, aprilTagPose.x);
        pose.tvec.put(1,0, aprilTagPose.y);
        pose.tvec.put(2,0, aprilTagPose.z);

        Mat R = new Mat(3, 3, CvType.CV_32F);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.put(i,j, aprilTagPose.R.get(i,j));
            }
        }

        Calib3d.Rodrigues(R, pose.rvec);

        return pose;
    }

      /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}
