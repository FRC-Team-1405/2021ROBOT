/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;
import java.util.stream.DoubleStream;

public class MeanFilter extends BaseFilter {
    public MeanFilter(int size) {
        super(size);
    }

    @Override
    public double calculation() {
        return DoubleStream.of(values).average().getAsDouble();
    }
}
