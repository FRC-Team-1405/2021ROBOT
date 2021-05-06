package frc.robot.lib;

public class Interpolate { 

    public static class Point {
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

    public  double calculate(double input){
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
              output = calculatePoint(input, points[lowIndex], points[highIndex]);
              break;
            }
          }
        }
      return output;
    }

    private static double calculatePoint(double input, Point low, Point high){ 
      //From slope point formula (y-y_1) = m(x-x_1) -> y = m(x-x_1) + y_1
      double output = (((high.output - low.output) / (high.input - low.input)) * (input - low.input) + low.output);
      return output;
    }
}