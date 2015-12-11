
/*
 * Copyright (C) 2015 by
 *
 * 	Md. Hijbul Alam
 *	hijbul@korea.ac.kr or hijbul@gmail.com
 * 	Dept. of Computer Science
 * 	Korea University University
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


import org.kohsuke.args4j.Option;

public class JMTSCmdOption {

    @Option(name="-est", usage="Specify whether we want to estimate model from scratch")
    public boolean est = false;

    @Option(name="-estp", usage="Specify whether we want to estimate model from scratch")
    public boolean estp = false;

    @Option(name="-estc", usage="Specify whether we want to continue the last estimation")
    public boolean estc = false;

    @Option(name="-inf", usage="Specify whether we want to do inference")
    public boolean inf = false;


    @Option(name="-dir", usage="Specify directory")
    //public String dir =  System.getProperty("user.dir")+ File.separator +"results_models/JMTS";
    public String dir =  "";

    @Option(name="-dfile", usage="Specify data file")
    public String dfile = "";

//    @Option(name="-doutfile", usage="Specify data file")
//    public String doutfile = "";

    @Option(name="-model", usage="Specify the model name")
    public String modelName = "model-final";


    @Option(name="-alphaGlobal", usage="Specify alpha")
    public double alphaGlobal = 0.5;

    @Option(name="-betaGlobal", usage="Specify beta")
    public double betaGlobal = 0.01;

    @Option(name="-alphaLocal", usage="Specify alpha")
    double alphaLocal = 0.5;

    @Option(name="-betaLocal", usage="Specify beta local")
    public double betaLocal = 0.001;

    @Option(name="-gamma", usage="Specifygamma")
    public double gamma = 0.001;

    @Option(name="-delta", usage="Specify delta")
    public double delta = 0.01;
    @Option(name="-adelta", usage="Specify delta")
    public double adeltaNeg = 5;

    @Option(name="-alpphamixl", usage="Specify delta")
    public double alphamixgl = 0.1;
    @Option(name="-alpphamixLoc", usage="Specify delta")
    public double alphaMixLocal = 0.4;

    @Option(name="-ntopics", usage="Specify the number of topics")
    public int K = 50;
    @Option(name="-nSenti", usage="Specify the number of topics")
    public int S = 2;
    @Option(name="-ngl", usage="Specify the number of topics")
    public int glK = 30;
    @Option(name="-nloc", usage="Specify the number of topics")
    public int locK=10;

    @Option(name="-niters", usage="Specify the number of iterations")
    public int niters = 500;

    @Option(name="-savestep", usage="Specify the number of steps to save the model since the last save")
    public int savestep = 1001;

    @Option(name="-twords", usage="Specify the number of most likely words to be printed for each topic")
    public int twords = 30;

    @Option(name="-withrawdata", usage="Specify whether we include raw data in the input")
    public boolean withrawdata = false;

    @Option(name="-wordmap", usage="Specify the wordmap file")
    public String wordMapFileName = "wordmap.txt";
}
