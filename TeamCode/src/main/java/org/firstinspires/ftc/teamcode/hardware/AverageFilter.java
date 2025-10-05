package org.firstinspires.ftc.teamcode.hardware;

public class AverageFilter {
    private int length = 5;
    private double currentSum = 0.0;
    private double currentAverage = 0.0;
    private double[] values = null;
    private double maxDifferenceForNewValue = 0.2;
    public AverageFilter(int length, double maxDifference){

        // allocate the array, should default to all zeros
        values = new double[length];
        maxDifferenceForNewValue = maxDifference;
    }

    public void setMaxDifference(double maxDiff){
        maxDifferenceForNewValue = maxDiff;
    }

    public double getMaxDifferenceForNewValue(){
        return maxDifferenceForNewValue;
    }

    public void setNewValue(double newValue){

        // storing values in the array with most recent value at 0 and oldest value
        // at 'length - 1' in the array

        // first thing to do is to subtract the value that is going to leave the array
        // out of the current sum - this way, we don't have to recompute the sum every time
        currentSum -= values[values.length - 1];

        // next thing to do when setting a new value is move everything by one
        // location in the array
        for (int i=0; i < values.length - 1; i++) {
            values[i+1] = values[i];
        }

        // next thing to do is determine the new value, check the difference of
        // the potential new value with the current most recent - if it's beyond the threshold
        // then use the most recent again as a repeat, if it's not then set it
        double newValueToUse = 0.0;
        if (Math.abs(newValue - values[0]) > maxDifferenceForNewValue){
            newValueToUse = values[0];
        }
        else {
            newValueToUse = newValue;
        }

        // now that we know whether we are using the new value or the current most recent,
        // add the value to the running sum and then compute the most recent average
        values[0] = newValueToUse;
        currentSum += newValueToUse;
        currentAverage = currentSum / (double)values.length;
    }

    public double getCurrentAverage(){
        return currentAverage;
    }

}
