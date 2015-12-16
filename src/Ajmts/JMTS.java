
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

package Ajmts;


import org.kohsuke.args4j.CmdLineException;
import org.kohsuke.args4j.CmdLineParser;
import util.Setup;
import util.Timer;


public class JMTS {

    public static void runSingle(String args[]) throws Exception{
        JMTSCmdOption option = new JMTSCmdOption();
        CmdLineParser parser = new CmdLineParser(option);
        Timer timer = new Timer();
        timer.start();

        System.out.println(option.dir);
        int dataset = 0;

//        for (int run = 0; run < 1; ++run)
        {

//            System.out.println("run "+ run);
            try{
                if (args.length == 0){
                    showHelp(parser);
                    return;
                }
                parser.parseArgument(args);
//                String tmp = option.dir;
//                option.dfile = option.dfile +"-"+dataset+".txt";


//                String extension = "";
//
//                int i =  option.dfile.lastIndexOf('.');
//                String fname = option.dfile;
//                if (i > 0) {
//                    extension =  option.dfile.substring(i+1);
//                    fname = option.dfile.substring(0, i);
//                }
//                else fname = fname+"-";
//                String tmp1 =  tmp + "/" + fname +"/";
//

//                Setup.createDir(tmp1);

                System.out.println("run"+" "+option.glK+" "+option.locK);

//                option.dir=tmp1+"/"+Integer.toString(run);
//                Setup.createDir(option.dir);
//                Setup.fileCopy(tmp+"/"+option.dfile, option.dir+"/"+option.dfile);

                if (option.est || option.estc){
                    Estimator estimate = new Estimator();
                    estimate.init(option);
                    estimate.estimate(true);
                }
                else if (option.estp){
                    Estimator estimate = new Estimator();
                    estimate.init(option);
                    estimate.estimate(true);
                }
                else if (option.inf){
                    Inferencer inferencer = new Inferencer();
                    inferencer.init(option);
                    inferencer.inference();
                }
            }
            catch (CmdLineException cle){
                System.out.println("Command line error: " + cle.getMessage());
                showHelp(parser);
                return;
            }
            catch (Exception e){
                System.out.println("Error in main: LDA " + e.getMessage());
                return;
            }

        }


        timer.end();
        System.out.println("Elapsed time: " + timer.getElapsedTime());

    }
    public static void main(String args[]) throws Exception{
       runSingle(args);
    }

	public static void showHelp(CmdLineParser parser){
		System.out.println("LDA [options ...] [arguments...]");
		parser.printUsage(System.out);
	}
                        

}
