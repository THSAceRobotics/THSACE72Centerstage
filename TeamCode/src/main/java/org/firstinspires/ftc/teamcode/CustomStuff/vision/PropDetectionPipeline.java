package org.firstinspires.ftc.teamcode.CustomStuff.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class PropDetectionPipeline implements VisionProcessor {

    public static int hsvLow1 = 30;
    public static int hsvLow2 = 170;
    public static int hsvLow3 = 30;
    public static int hsvHigh1 = 180;
    public static int hsvHigh2 = 255;
    public static int hsvHigh3 = 255;
    public static int numContours = 0;

    //private final double X_MIN_RIGHT, Y_MIN_RIGHT, X_MIN_LEFT, Y_MIN_LEFT, X_MIN_MID, Y_MIN_MID, X_MAX_RIGHT, Y_MAX_RIGHT, X_MAX_LEFT, Y_MAX_LEFT, X_MAX_MID, Y_MAX_MID;

    public static double largestContourArea = 0;
    public static double largestContourX, largestContourY;
    private MatOfPoint largestContour;
    public int location;

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

        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Mat a = new Mat();
        Imgproc.findContours(thresh, contours, edges, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        numContours = contours.size();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea && area > 20) {
                largestContour = contour;
                largestContourArea = area;
            }
        }

        if (largestContour != null) {
            Moments moment = Imgproc.moments(largestContour);
            largestContourX = (moment.m10 / moment.m00);
            largestContourY = (moment.m01 / moment.m00);
        }
        /*
        if(X_MIN_LEFT < largestContourX && largestContourX < X_MAX_LEFT && Y_MIN_LEFT < largestContourY && largestContourY < Y_MAX_LEFT) {
            location = 1;
        } else if (X_MIN_MID < largestContourX && largestContourX < X_MAX_MID && Y_MIN_MID < largestContourY && largestContourY < Y_MAX_MID) {
            location = 2;
        } else {
            location = 3;
        }
        */
        edges.copyTo(input);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
