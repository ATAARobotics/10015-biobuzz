package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorDetector extends OpenCvPipeline {
    public String result = "unknown";
    public Scalar min;
    public Scalar max;
    
    public ColorDetector() {
        min = new Scalar(70, 112, 139);
        max = new Scalar(88, 255, 255);
    }
    public ColorDetector(boolean red) {
        if (red) {
            min = new Scalar(100, 100, 100);
            max = new Scalar(160, 255, 255);
        } else {
            // blue
            min = new Scalar(22, 31, 69);
            max = new Scalar(69, 255, 255);
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        // don't create new Mat's in this method? apparently ... Mat processed = new Mat();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

        // filter for one colour
        Core.inRange(input, min, max, input);

        Imgproc.dilate(input, input, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(input, input, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(input, input, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

        int width = 50;
        int height = 55;

        int x0 = 100;
        int y0 = 236;
        int x1 = 330;
        int y1 = 214;
        int x2 = 560;
        int y2 = 247;

        // count how many pixels are "on" in each region of interest

        int left = 0;
        for (int x=x0; x < x0 + width; x++) {
            for (int y=y0; y < y0 + height; y++) {
                double[] px = input.get(y, x);
                if (px != null && (int)px[0] > 128) {
                    left++;
                }
            }
        }

        int mid = 0;
        for (int x=x1; x < x1 + width; x++) {
            for (int y=y1; y < y1 + height; y++) {
                double[] px = input.get(y, x);
                if (px != null && (int)px[0] > 128){
                    mid++;
                }
            }
        }

        double right = 0;
        for (int x=x2; x < x2 + width; x++) {
            for (int y=y2; y < y2 + height; y++) {
                double[] px = input.get(y, x);
                if (px != null && (int)px[0] > 128) {
                    right++;
                }
            }
        }


        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);
        double biggest = Math.max(left, Math.max(mid, right));
        if (biggest > 100) {
            if (left == biggest) { result = "left";  }
            if (mid == biggest) { result = "middle"; }
            if (right == biggest) { result = "right"; }
        }


        Imgproc.putText(input, "L:" + left, new Point(x0, y0), Imgproc.FONT_HERSHEY_PLAIN, 1, red);
        Imgproc.putText(input, "M:" + mid, new Point(x1, y1), Imgproc.FONT_HERSHEY_PLAIN, 1, red);
        Imgproc.putText(input, "R:" + right, new Point(x2, y2), Imgproc.FONT_HERSHEY_PLAIN, 1, red);

        Imgproc.rectangle(input, new Point(x0, y0), new Point(x0 + width, y0 + width), biggest == left ? green : red, 2);
        Imgproc.rectangle(input, new Point(x1, y1), new Point(x1 + width, y1 + width), biggest == mid ? green : red, 2);
        Imgproc.rectangle(input, new Point(x2, y2), new Point(x2 + width, y2 + width), biggest == right ? green : red, 2);

        return input;
    }
}
