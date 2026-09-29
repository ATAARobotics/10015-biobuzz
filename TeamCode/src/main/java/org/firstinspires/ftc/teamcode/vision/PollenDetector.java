package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


/*
 * Good Java example of making an FTC VisionProcessor:
 * Learn Java for FTC, Chapter 16, especially FirstVisionProcessor.java
 * https://github.com/alan412/LearnJavaForFTC/blob/master/LearnJavaForFTC.pdf
 */

@Configurable
public class PollenDetector implements VisionProcessor {
    public static double H_MIN = 20;
    public static double H_MAX = 40;

    public static double S_MIN = 150;
    public static double S_MAX = 255;

    public static double V_MIN = 100;
    public static double V_MAX = 255;

    public static double MIN_CIRCULARITY = 0.0;
    public static double MIN_AREA = 24;


    // OpenCV uses Mat objects to hold images.
    // The FTC example in Learn Java for FTC also uses Mats for the camera image and HSV image.

    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat dilateKernel =
            Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT,
                    new Size(3, 3));

    private final Mat erodeKernel =
            Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT,
                    new Size(7, 7));

    private volatile List<PollenBlob> blobs =
            Collections.emptyList();

    private int frameWidth;

    private final Paint paint = new Paint();


    public PollenDetector() {

        // Canvas/Paint example is also in Learn Java for FTC Chapter 16.
        // https://github.com/alan412/LearnJavaForFTC/blob/master/LearnJavaForFTC.pdf

        paint.setColor(Color.YELLOW);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3);
    }


    // Just a small class to save the information we want about each blob.

    public static class PollenBlob {

        public final double x;
        public final double y;
        public final double radius;
        public final double area;
        public final double circularity;


        public PollenBlob(
                double x,
                double y,
                double radius,
                double area,
                double circularity) {

            this.x = x;
            this.y = y;
            this.radius = radius;
            this.area = area;
            this.circularity = circularity;
        }
    }


    /*
     * VisionProcessor requires init(), processFrame(), and onDrawFrame().
     *
     * FTC example:
     * https://github.com/alan412/LearnJavaForFTC/blob/master/LearnJavaForFTC.pdf
     *
     */

    @Override
    public void init(
            int width,
            int height,
            CameraCalibration calibration) {

        frameWidth = width;
    }


    @Override
    public Object processFrame(
            Mat frame,
            long captureTimeNanos) {

        //Convert the camera frame from RGB to HSV.
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);


    // Make a black and white mask
        Core.inRange(
                hsv,
                new Scalar(H_MIN, S_MIN, V_MIN),
                new Scalar(H_MAX, S_MAX, V_MAX),
                mask);
    // Dilation grows the white areas and erosion shrinks them.

        Imgproc.dilate(mask, mask, dilateKernel);
        Imgproc.erode(mask, mask, erodeKernel);

        /*
         * Find the outlines of the white areas.
         *
         * OpenCV Java findContours example:
         * https://docs.opencv.org/4.x/df/d0d/tutorial_find_contours.html
         */

        List<MatOfPoint> contours =
                new ArrayList<>();

        // RETR_EXTERNAL -> keep only outer-most contours
        // CHAIN_APPROX_SIMPLE -> store simplifeid set of points instead of every single boundary pixel
        Imgproc.findContours(
                mask,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);

        List<PollenBlob> found = new ArrayList<>();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);


            /*
             * arcLength() uses MatOfPoint2f.
             *
             * The OpenCV Java example does basically this:
             *
             * Imgproc.arcLength(
             *     new MatOfPoint2f(contours.get(i).toArray()),
             *     true);
             *
             *  https://docs.opencv.org/4.13.0/d0/d49/tutorial_moments.html
             */

            MatOfPoint2f curve =
                    new MatOfPoint2f(
                            contour.toArray());


            double perimeter = Imgproc.arcLength(curve,true);
            if (perimeter > 0) {
                // Circularity = 4*pi*area / perimeter^2
                double circularity = 4.0 * Math.PI * area / (perimeter * perimeter);
                if (circularity >= MIN_CIRCULARITY && circularity <= 1.0 && area >MIN_AREA) {

                    Point center =
                            new Point();

                    float[] radius =
                            new float[1];

                    Imgproc.minEnclosingCircle(
                            curve,
                            center,
                            radius);


                    found.add(
                            new PollenBlob(
                                    center.x,
                                    center.y,
                                    radius[0],
                                    area,
                                    circularity));
                }
            }

            curve.release();
            contour.release();
        }


        // Biggest blobs first.

        found.sort(
                (a, b) ->
                        Double.compare(
                                b.area,
                                a.area));


        blobs = found;


        /*
         * Whatever processFrame() returns is passed to onDrawFrame()
         * as userContext.
         *
         * This is explained in Learn Java for FTC Chapter 16.
         * https://github.com/alan412/LearnJavaForFTC/blob/master/LearnJavaForFTC.pdf
         */

        return found;
    }


    /*
     * Draw circles over the pollen in the camera preview.
     *
     * Learn Java for FTC has actual Java Canvas/Paint/onDrawFrame code:
     * https://github.com/alan412/LearnJavaForFTC/blob/master/LearnJavaForFTC.pdf
     */

    @Override
    @SuppressWarnings("unchecked")
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext) {

        List<PollenBlob> found =
                (List<PollenBlob>) userContext;


        for (PollenBlob blob : found) {
            canvas.drawCircle(
                    (float) blob.x * scaleBmpPxToCanvasPx,
                    (float) blob.y * scaleBmpPxToCanvasPx,
                    (float) blob.radius * scaleBmpPxToCanvasPx,
                    paint);
        }
    }

    public List<PollenBlob> getBlobs() {
        return blobs;
    }


    public int getFrameWidth() {

        return frameWidth;
    }
}