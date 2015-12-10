/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package util;

/**
 *
 * @author Hijbul
 */
public class Timer {

  private long startTime = 0;
  private long endTime   = 0;

  public void start(){
    this.startTime = System.currentTimeMillis();
  }

  public void end() {
    this.endTime   = System.currentTimeMillis();  
  }

  public long getStartTime() {
    return this.startTime;
  }

  public long getEndTime() {
    return this.endTime;
  }

  public long getTotalTime() {
    return this.endTime - this.startTime;
  }
//  public long getElapsedTime() {
//    return this.endTime - this.startTime;
//  }
  /**
 *  elapsed time in hours/minutes/seconds -- //getElapsedTimeHoursMinutesSecondsString
 * @return String
 */
  
public String getElapsedTime() {     
    long elapsedTime = this.endTime - this.startTime;
    String format = String.format("%%0%dd", 2);
    elapsedTime = elapsedTime / 1000;
    String seconds = String.format(format, elapsedTime % 60);
    String minutes = String.format(format, (elapsedTime % 3600) / 60);
    String hours = String.format(format, elapsedTime / 3600);
    String time =  hours + ":" + minutes + ":" + seconds;
    return time;
}

}