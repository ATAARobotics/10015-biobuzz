package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

//import com.bylazar.configurables.annotations.Configurable;

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

//@Configurable
public class PollenDetector implements VisionProcessor {

    public static double H_MIN = 20;
    public static double H_MAX = 40;

    public static double S_MIN = 100;
    public static double S_MAX = 255;

    public static double V_MIN = 100;
    public static double V_MAX = 255;

    public static double MIN_CIRCULARITY = 0.7;
    public static double MIN_AREA = 100;


    /*
     * 0 = normal camera
     * 1 = HSV mask after erosion/dilation
     * 2 = filtered mask with accepted pollen only
     */
    public static double VIEW_MODE = 0;


    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();

    // Holds only blobs that passed area/circularity filtering.
    private final Mat filteredMask = new Mat();


    private final Mat dilateKernel =
            Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT,
                    new Size(7, 7));

    private final Mat erodeKernel =
            Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT,
                    new Size(7, 7));


    private volatile List<PollenBlob> blobs =
            Collections.emptyList();

    private int frameWidth;

    private final Paint paint = new Paint();


    public PollenDetector() {

        paint.setColor(Color.YELLOW);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3);
    }


    /*
     * Saves the information about each detected pollen.
     */
    public static class PollenBlob {

        public final double x;
        public final double y;

        public final double radius;
        public final double radiusInverse;

        public final double area;
        public final double circularity;

        public final double radiusVsDistSlope = 890.42;
        public final double radiusVsDistIntercept = -1.112;

        public final double pollenDistance;


        public PollenBlob(
                double x,
                double y,
                double radius,
                double area,
                double circularity) {

            this.x = x;
            this.y = y;

            this.radius = radius;

            this.radiusInverse = 1.0 / radius;

            /*
             * Calibration equation:
             *
             * distance = slope * (1/radius) + intercept
             */
            this.pollenDistance =
                    radiusVsDistSlope * radiusInverse
                            + radiusVsDistIntercept;

            this.area = area;
            this.circularity = circularity;
        }
    }


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


        // Convert camera frame from RGB to HSV.
        Imgproc.cvtColor(
                frame,
                hsv,
                Imgproc.COLOR_RGB2HSV);


        // Make the black-and-white HSV mask.
        Core.inRange(
                hsv,
                new Scalar(
                        H_MIN,
                        S_MIN,
                        V_MIN),
                new Scalar(
                        H_MAX,
                        S_MAX,
                        V_MAX),
                mask);


        /*
         * Erosion removes small white areas.
         * Dilation grows the surviving areas back.
         */
        Imgproc.erode(
                mask,
                mask,
                erodeKernel);

        Imgproc.dilate(
                mask,
                mask,
                dilateKernel);


        /*
         * Find contours in the mask.
         */
        List<MatOfPoint> contours =
                new ArrayList<>();

        Imgproc.findContours(
                mask,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);


        /*
         * Start with a completely black filtered mask.
         */
        filteredMask.create(
                mask.rows(),
                mask.cols(),
                mask.type());

        filteredMask.setTo(
                new Scalar(0));


        List<PollenBlob> found =
                new ArrayList<>();


        for (MatOfPoint contour : contours) {

            double area =
                    Imgproc.contourArea(contour);


            MatOfPoint2f curve =
                    new MatOfPoint2f(
                            contour.toArray());


            double perimeter =
                    Imgproc.arcLength(
                            curve,
                            true);


            if (perimeter > 0) {

                /*
                 * Circularity:
                 *
                 * 4 * pi * area / perimeter^2
                 */
                double circularity =
                        4.0 * Math.PI * area
                                / (perimeter * perimeter);


                if (circularity >= MIN_CIRCULARITY
                        && circularity <= 1.0
                        && area > MIN_AREA) {


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


                    /*
                     * Draw accepted pollen into filteredMask.
                     *
                     * Anything rejected by area/circularity
                     * remains black.
                     */
                    Imgproc.drawContours(
                            filteredMask,
                            Collections.singletonList(contour),
                            -1,
                            new Scalar(255),
                            Imgproc.FILLED);
                }
            }


            curve.release();
            contour.release();
        }


        /*
         * Closest pollen first.
         *
         * Smaller pollenDistance = closer.
         */
        found.sort(
                (a, b) ->
                        Double.compare(
                                a.pollenDistance,
                                b.pollenDistance));


        blobs = found;


        /*
         * Change the image displayed by EOCV-Sim / VisionPortal.
         *
         * 0 = leave frame unchanged
         * 1 = show HSV/morphology mask
         * 2 = show only accepted pollen
         */

        if (VIEW_MODE >= 1.5) {

            // Accepted pollen only.
            Imgproc.cvtColor(
                    filteredMask,
                    frame,
                    Imgproc.COLOR_GRAY2RGB);

        } else if (VIEW_MODE >= 0.5) {

            // HSV mask after erosion/dilation.
            Imgproc.cvtColor(
                    mask,
                    frame,
                    Imgproc.COLOR_GRAY2RGB);
        }


        /*
         * The returned list gets passed into
         * onDrawFrame() as userContext.
         */
        return found;
    }


    /*
     * Draw circles around accepted pollen.
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


        if (!(userContext instanceof List)) {
            return;
        }


        List<PollenBlob> found =
                (List<PollenBlob>) userContext;


        for (PollenBlob blob : found) {

            canvas.drawCircle(
                    (float) blob.x
                            * scaleBmpPxToCanvasPx,

                    (float) blob.y
                            * scaleBmpPxToCanvasPx,

                    (float) blob.radius
                            * scaleBmpPxToCanvasPx,

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