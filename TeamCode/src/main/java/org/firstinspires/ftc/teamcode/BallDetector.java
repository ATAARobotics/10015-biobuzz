package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

public class BallDetector extends OpenCvPipeline {

    public Scalar purpleLowerHSV = new Scalar(123.0, 88.0, 44.0, 0.0);
    public Scalar purpleUpperHSV = new Scalar(239.0, 214.0, 173.0, 0.0);
    private Mat hsvBinaryMat = new Mat();

    public int erodeValue = ((int) (13));
    public int dilateValue = ((int) (13));
    private Mat element = null;
    private Mat hsvBinaryMatErodedDilated = new Mat();

    public Scalar greenLowerHSV = new Scalar(28.0, 104.0, 0.0, 0.0);
    public Scalar greenUpperHSV = new Scalar(88.0, 255.0, 255.0, 0.0);
    private Mat hsvBinaryMat1 = new Mat();

    public int erodeValue1 = ((int) (13));
    public int dilateValue1 = ((int) (13));
    private Mat element1 = null;
    private Mat hsvBinaryMat1ErodedDilated = new Mat();

    public Mat bitwiseORMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness = 3;

    private Mat inputContours = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // "Color Threshold"
        Imgproc.cvtColor(input, hsvBinaryMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvBinaryMat, purpleLowerHSV, purpleUpperHSV, hsvBinaryMat);

        // "Erode and Dilate"
        hsvBinaryMat.copyTo(hsvBinaryMatErodedDilated);
        if (erodeValue > 0) {
            this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeValue, erodeValue));
            Imgproc.erode(hsvBinaryMatErodedDilated, hsvBinaryMatErodedDilated, element);

            element.release();
        }

        if (dilateValue > 0) {
            this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateValue, dilateValue));
            Imgproc.dilate(hsvBinaryMatErodedDilated, hsvBinaryMatErodedDilated, element);

            element.release();
        }

        // "Color Threshold"
        Imgproc.cvtColor(input, hsvBinaryMat1, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvBinaryMat1, greenLowerHSV, greenUpperHSV, hsvBinaryMat1);

        // "Erode and Dilate"
        hsvBinaryMat1.copyTo(hsvBinaryMat1ErodedDilated);
        if (erodeValue1 > 0) {
            this.element1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeValue1, erodeValue1));
            Imgproc.erode(hsvBinaryMat1ErodedDilated, hsvBinaryMat1ErodedDilated, element1);

            element1.release();
        }

        if (dilateValue1 > 0) {
            this.element1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateValue1, dilateValue1));
            Imgproc.dilate(hsvBinaryMat1ErodedDilated, hsvBinaryMat1ErodedDilated, element1);

            element1.release();
        }

        // "Bitwise OR"
        bitwiseORMat.release();
        Core.bitwise_or(hsvBinaryMatErodedDilated, hsvBinaryMat1ErodedDilated, bitwiseORMat);

        // "Simple Find Contours"
        contours.clear();
        hierarchy.release();
        Imgproc.findContours(bitwiseORMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // "Draw Contours"
        input.copyTo(inputContours);
        Imgproc.drawContours(inputContours, contours, -1, lineColor, lineThickness);

        return inputContours;
    }
}
