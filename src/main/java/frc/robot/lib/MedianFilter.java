/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import java.util.stream.DoubleStream;

/**
 * Add your docs here.
 */
public class MedianFilter extends BaseFilter {

    public MedianFilter(int size) {
        super(size);
    }

    @Override
    public double calculation() {
        return DoubleStream.of(values) // use for median 
                     .sorted()
                     .skip(values.length/2)
                     .findFirst()
                     .getAsDouble();
    }

}
