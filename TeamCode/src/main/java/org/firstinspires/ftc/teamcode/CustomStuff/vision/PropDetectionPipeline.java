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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class PropDetectionPipeline implements VisionProcessor {

    public Scalar lowHSVRed = new Scalar(30,170, 30);
    public Scalar highHSVRed = new Scalar(180, 255, 255);
    public Scalar lowHSVBlue = new Scalar(30, 80, 40);
    public Scalar highHSVBlue = new Scalar(180, 200, 150);

    public static boolean isBlue = true;
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
        Mat mat1 = input.clone();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone


        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Mat thresh = new Mat();
        if(isBlue) {
            Core.inRange(mat, lowHSVBlue, highHSVBlue, thresh);
        } else {
            Core.inRange(mat, lowHSVRed, highHSVRed, thresh);// higher bound HSV for yellow
        }


        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range

      // return the mat with rectangles drawn

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();

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
        Rect r = new Rect(new Point(largestContourX - 100, largestContourY - 100), new Point(largestContourX + 100, largestContourY + 100));
        Imgproc.rectangle(mat1, r, new Scalar(0, 0, 0));
        mat1.copyTo(input);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
