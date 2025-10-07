package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

public class BallDetector extends OpenCvPipeline {

    public Scalar lowerHSV = new Scalar(28.0, 104.0, 0.0, 0.0);
    public Scalar upperHSV = new Scalar(88.0, 255.0, 255.0, 0.0);
    private Mat hsvBinaryMat = new Mat();

    public int erodeValue = ((int) (13));
    public int dilateValue = ((int) (13));
    private Mat element = null;
    private Mat hsvBinaryMatErodedDilated = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness = 3;

    private Mat inputContours = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // "Color Threshold"
        Imgproc.cvtColor(input, hsvBinaryMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvBinaryMat, lowerHSV, upperHSV, hsvBinaryMat);

        // "Erode and Dilate"
        hsvBinaryMat.copyTo(hsvBinaryMatErodedDilated);
        if(erodeValue > 0) {
            this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeValue, erodeValue));
            Imgproc.erode(hsvBinaryMatErodedDilated, hsvBinaryMatErodedDilated, element);

            element.release();
        }

        if(dilateValue > 0) {
            this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateValue, dilateValue));
            Imgproc.dilate(hsvBinaryMatErodedDilated, hsvBinaryMatErodedDilated, element);

            element.release();
        }

        // "Simple Find Contours"
        contours.clear();
        hierarchy.release();
        Imgproc.findContours(hsvBinaryMatErodedDilated, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // "Draw Contours"
        input.copyTo(inputContours);
        Imgproc.drawContours(inputContours, contours, -1, lineColor, lineThickness);

        return inputContours;
    }
}
