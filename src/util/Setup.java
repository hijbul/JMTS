
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

import java.io.*;


/**
 *
 * @author Hijbul
 */
public class Setup {
    
        
    public static void createDir( String dirName)  {
       try{
       File dir=new File(dirName);
       if(dir.exists()){
           System.out.println("A folder with name "+ dirName + " is already exist in the path "+dir);
       }else{
           dir.mkdir();
       }
       }catch(Exception e){
           System.out.println(e.getMessage());
       }
    }
  public void createUserDir( String dirName)  {
       try{
       String path = System.getProperty("user.dir");
       File dir=new File(path+"/"+dirName);
       if(dir.exists()){
//           System.out.println("A folder with name "+ dirName + " is already exist in the path "+path);
       }else{
           dir.mkdir();
       }
       }catch(Exception e){
           System.out.println(e.getMessage());
       }
    }
    
    public static void fileCopy(String orig, String dest ) throws IOException{
        InputStream in = new FileInputStream(orig);
        OutputStream out = new FileOutputStream(dest);
        byte[] buf = new byte[1024];
        int len;
        while ((len = in.read(buf)) > 0) {
           out.write(buf, 0, len);
        }
        in.close();
        out.close(); 
    }

    public static void main(String[] args) throws IOException  {
       Setup s = new Setup();
//       s.createUserDir("results_models/JMTS");

               
                
       
       
    }
}
