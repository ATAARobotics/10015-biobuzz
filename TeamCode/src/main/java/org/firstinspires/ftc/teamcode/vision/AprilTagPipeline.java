package org.firstinspires.ftc.teamcode.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
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

import org.opencv.features2d.ORB;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.Features2d;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

// copied from OpenCV examples
public class AprilTagPipeline extends OpenCvPipeline
{
    private MatOfKeyPoint keypoints;
    private Mat descriptors;
    private Mat dbg;

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    // one distance for every april tag (1 through 6) that we "might"
    // care about. Negative numbers means we can't see it
    // currently. Useful for "virtual fence". index 0 is not used.
    public double[] distances;

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
    double tagsizeX;
    double tagsizeY;

    int target_tag = -1;
    double distance_mm;
    double strafe_mm;
    boolean detected = false;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public double distance() { return distance_mm;}
    public double strafe() { return strafe_mm; }
    public boolean has_result() { return detected; }
    public void set_target(int tid) { target_tag = tid; }

    public AprilTagPipeline()//int target_tag_id)
    {
        // for arducam i think, from ilyas
        double fx = 597.628;
        double fy = 597.628;
        double cx = 485.707;
        double cy = 294.54;

        descriptors = new Mat();
        dbg = new Mat();
        keypoints = new MatOfKeyPoint();

        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        ORB orb = ORB.create();
        BFMatcher bfm = BFMatcher.create();
//        List<Mat> images = new LinkedList<Mat>();
//        images.add(input);

        orb.detect(input, keypoints);
        orb.compute(input, keypoints, descriptors);

        // need a second image ...
        // then
        //bfm.match();

        Features2d.drawKeypoints(input, keypoints, dbg);
        return dbg;
    }
}
