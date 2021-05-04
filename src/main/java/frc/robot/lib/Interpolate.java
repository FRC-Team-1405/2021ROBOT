package frc.robot.lib;

import edu.wpi.first.wpilibj.util.Units;

public class Interpolate { 

    private static class Point {
        public final double input; 
        public final double output; 

        public Point(double input, double output){ 
            this.input = input;
            this.output = output;     
        }
    }

    private Point[] points;

    public Interpolate(Point[] points) {
        this.points = points;
    }

    public Interpolate(String importFile){
        // ToDo load json file
        this.points = new Point[] { new Point(Units.feetToMeters(0) *100.0, 10000),
                                    new Point(Units.feetToMeters(10)*100.0, 180000),
                                    new Point(Units.feetToMeters(20)*100.0, 350000)
                                };

    }

    public  double CalculateOutput(double input){
        int lowIndex = 0;
        int highIndex = 0;
        double output = 0.0;
        
    
        if(input <= points[0].input){
          output = points[0].output;
        }
        else if(input >= points[points.length-1].input){
          output = points[points.length-1].output;
        }
        else{
          for(int i = 1; i < points.length; i++){
            if(input <= points[i].output){
              lowIndex = i-1;
              highIndex = i;
              output = Calculate(input, points[lowIndex], points[highIndex]);
              break;
            }
          }
        }
      return output;
    }

    private static double Calculate(double input, Point low, Point high){ 
      //From slope point formula (y-y_1) = m(x-x_1) -> y = m(x-x_1) + y_1
      double output = (((high.output - low.output) / (high.input - low.input)) * (input - low.input) + low.output);
      return output;
    }
}