package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * note: re-worked from example pipeline code provided from Skystone game
 * the dilate/erode filters were from their ideas
 */


// NOTE: "sideways left" is correct orientation in Sim

public class SimplePipeline extends OpenCvPipeline
{
    private RotatedRect sample;

    /*
     * Working image buffers
     */
    Mat processed = new Mat();
    Mat out = new Mat();

    Mat dilate_rect = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5.5, 5.5));

    // in each "Scalar" are "H, S and V" values -- only because we're
    // using the "HSV" colour-space below when using "min" and "max";
    // they could be "R, G and B" if we'd left it in the RGB
    // colour-space

    // colour filtering constants
    public static Scalar min = new Scalar(18, 175, 100);
    public static Scalar max = new Scalar(68, 255, 255);

    // EasyOpenCV-Sim can access and change all "public" variables
    public static boolean debug_filter = false;  // return early from filtering
    public static boolean debug_outlines = true;  // draw angles / bounding rects

    @Override
    public void onViewportTapped()
    {
	debug_filter = !debug_filter;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // do not ever do a "new Mat" inside this method; pre-allocate them elsewhere

        // Convert the input image to HSV colour-space
        Imgproc.cvtColor(input, processed, Imgproc.COLOR_RGB2HSV);
        out = input;

	// make entire image black/white based on the mix/max HSV
	// values (for yellow)
	Core.inRange(processed, min, max, processed);

        // OpenCV says that "erode, then dilate" is a good thing to
        // do; playing with this in EasyOpenCV-Sim shows that a couple
        // erodes and then "several" dilates work well
	Imgproc.erode(processed, processed, dilate_rect);
	Imgproc.erode(processed, processed, dilate_rect);
	Imgproc.dilate(processed, processed, dilate_rect);
        Imgproc.dilate(processed, processed, dilate_rect);
        Imgproc.dilate(processed, processed, dilate_rect);
        Imgproc.dilate(processed, processed, dilate_rect);

	if (debug_filter) {
	    return processed;
	}

        // find blobs
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        RotatedRect best = null;
	double bigY = 0;
        for(MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            // Fit a rotated rectangle to the contour
            RotatedRect rect = Imgproc.minAreaRect(contour2f);
	    // sample-detector looked at "minimum size" to exclude
	    // outliers -- we could here too via "rect.size.area()"
	    if (rect.center.y > bigY) {
		best = rect;
		bigY = rect.center.y;
	    }
            if (debug_outlines) {
                drawRect(rect, out, new Scalar(0, 0, 0));
            }
        }
	if (best != null) {
	    sample = best;

            if (debug_outlines) {
                drawRect(sample, out, new Scalar(255, 0, 255));
            }
	}

        return out;
    }

    static void drawRect(RotatedRect rect, Mat target, Scalar color)
    {
        /*
         * Draws a rotated rectangle by drawing each of the 4 lines individually
         */
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; ++i)
        {
            Imgproc.line(target, points[i], points[(i + 1) % 4], color, 2);
        }
    }

}
