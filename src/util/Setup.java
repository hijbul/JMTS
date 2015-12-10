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
