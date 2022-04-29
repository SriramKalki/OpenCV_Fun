package org.firstinspires.ftc.teamcode.pipeline;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Pipeline extends OpenCvPipeline {
    //creates 3 matrices

    Mat mat = new Mat();
    Mat output = new Mat();
    //creates 2 rectangles
    Rect upperROI = new Rect(new Point(0,0), new Point(1,1));
    Rect lowerROI;
    //enum for TSE position

    //TODO find the threshold value- the minimum upper/lowerValue needed to ensure that there is a TSE
    final double threshold = 50;
    //For camera stream coloring purposes
    final Scalar green = new Scalar(0, 255, 0);
    final Scalar red = new Scalar(255, 0, 0);

    Scalar lowHSV = new Scalar(23, 50, 70);
    Scalar highHSV = new Scalar(32, 255, 255);


    //Telemetry
    Telemetry telemetry;

    //CONSTRUCTOR
    public Pipeline(Telemetry t) {
        //setting telemetry
        telemetry = t;

    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        output = input;
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mat, lowHSV, highHSV, mat);

        List<MatOfPoint> countersList = new ArrayList<>();
        Imgproc.findContours(mat, countersList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(mat, countersList,0, new Scalar(255,0,0));
        Imgproc.drawContours(output, countersList,0, new Scalar(255,0,0));
        Rect theRealDucky = new Rect(new Point(0,0), new Point(1,1));
        for (MatOfPoint countor : countersList)
        {

            Rect rect = Imgproc.boundingRect(countor);
            if (rect.area() > theRealDucky.area()) {
                theRealDucky = rect;
            }

        }

        Imgproc.rectangle(output, theRealDucky, red);

        telemetry.update();
        return output;

    }
}