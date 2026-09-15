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
    public Scalar min;
    public Scalar max;
    
    public ColorDetector() {
        min = new Scalar(70, 112, 139);
        max = new Scalar(88, 255, 255);
    }

    @Override
    public Mat processFrame(Mat input) {
        // don't create new Mat's in this method? apparently ... Mat processed = new Mat();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

        // filter for one colour
        Core.inRange(input, min, max, input);

	Imgproc.erode(input, input, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
	
	Imgproc.dilate(input, input, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10)));


        return input;
    }
}
