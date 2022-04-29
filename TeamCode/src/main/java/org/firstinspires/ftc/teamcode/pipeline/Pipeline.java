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
    Mat leftSub, middleSub, rightSub = new Mat();
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
    public enum Location{
        LEFT,MIDDLE,RIGHT
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        output = input;
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.line(output,new Point(175/3.0,0), new Point(175/3.0,300), red);
        Imgproc.line(output,new Point(175/3.0 * 2.0,0), new Point(175/3.0 * 2.0,300), red);
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
//
        Imgproc.rectangle(output, theRealDucky, green);
        //x and y are top left coords
        Point leftTopPoint = new Point(theRealDucky.x,theRealDucky.y);
        Point rightTopPoint = new Point(theRealDucky.x + theRealDucky.width,theRealDucky.y);
        Location location;

        //left rectangle is from 0 to 175/3
        double areaLeft = Math.max(0,Math.min(175/3.0 - leftTopPoint.x, 175/3.0))* theRealDucky.height;
        double areaMiddle = Math.max(0,Math.min(2*175/3.0 - leftTopPoint.x, 175/3.0))* theRealDucky.height;
        double areaRight = Math.max(0,rightTopPoint.x - 2*175/3.0)* theRealDucky.height;

        if(areaLeft >= areaMiddle && areaLeft >= areaRight){
            location = Location.LEFT;
        }else if(areaMiddle >= areaRight && areaMiddle >= areaLeft){
            location = Location.RIGHT;
        }else{
            location = Location.MIDDLE;
        }
        telemetry.addData("left area: ",areaLeft);
        telemetry.addData("middle area: ", areaMiddle);
        telemetry.addData("right area: ", areaRight);
        switch(location){
            case LEFT:
                telemetry.addData("Location: ", "left");
                break;
            case RIGHT:
                telemetry.addData("Location: ", "right");
                break;
            case MIDDLE:
                telemetry.addData("Location: ", "middle");
                break;
        }
        telemetry.update();
        return output;

    }
}