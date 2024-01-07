package org.firstinspires.ftc.teamcode.CustomStuff.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PropDetectionPipeline implements VisionProcessor {

    public static int hsvLow1 = 30;
    public static int hsvLow2 = 170;
    public static int hsvLow3 = 30;
    public static int hsvHigh1 = 180;
    public static int hsvHigh2 = 255;
    public static int hsvHigh3 = 255;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        // if something is wrong, we assume there's no skystone


        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(hsvLow1, hsvLow2, hsvLow3); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(hsvHigh1, hsvHigh2, hsvHigh3); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);
      // return the mat with rectangles drawn

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(edges, contours, -1, new Scalar(255, 255, 255));
        thresh.copyTo(input);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
