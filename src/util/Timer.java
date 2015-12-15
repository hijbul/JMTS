
/*
 * Copyright (C) 2015 by
 *
 * 	Md. Hijbul Alam
 *	hijbul@korea.ac.kr or hijbul@gmail.com
 * 	Dept. of Computer Science
 * 	Korea University
 *
 * 	SangKeun Lee
 *	yalphy@korea.ac.kr
 * 	Dept. of Computer Science
 * 	Korea University
 *
 *
 * JMTS is a free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * JMTS is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with JMTS; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */
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