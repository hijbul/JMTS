
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

import it.unimi.dsi.fastutil.io.BinIO;
import util.Conversion;
import util.Pair;
import util.Timer;

import java.io.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Estimator {

	// output model
	Model trnModel;
	JMTSCmdOption option;
    int newTopic = 0, newSenti = 0,  newWindow=0;
    boolean newr;
    boolean priorAspect = false;

	public boolean init(JMTSCmdOption option){
		this.option = option;

		trnModel = new Model();
		if (option.est||option.estp){
			if (!trnModel.initNewModel(option))
				return false;
			trnModel.data.localDict.writeWordMap(option.dir + File.separator +  option.wordMapFileName);
		}
		else if (option.estc){
			if (!trnModel.initEstimatedModel(option))
				return false;
		}

		return true;
	}
  
    public void stepSave(){
                System.out.println("Saving the model at iteration " + trnModel.liter + " ...");
                trnModel.computePhiLoc();
                trnModel.computePhiGl();
                trnModel.saveModel("model-" + Conversion.ZeroPad(trnModel.liter, 5));

    }

    public double estimate(boolean prior) throws IOException{


        System.out.println("Sampling " + trnModel.niters + " iteration!");
        this.priorAspect = prior;
        Timer timer = new Timer();
        timer.start();
        Timer timer100 = new Timer();
        timer100.start();
        int lastIter = trnModel.liter;

        for (trnModel.liter = lastIter + 1; trnModel.liter < trnModel.niters + lastIter; trnModel.liter++){
            if(trnModel.liter%100==0) {
                System.out.print("Iteration " + trnModel.liter + " ...");
                timer100.end();
                System.out.println("elapsed time: " + timer100.getElapsedTime());// + "Remaming time ..."+trnModel.niters);
            }

            // for all z_i
            for (int m = 0; m < trnModel.M; m++){

                for (int n = 0; n < trnModel.data.docs[m].length; n++){
                    dec(m, n);
                    newWindow = samplingWindow(m, n);
                    newr = samplingR(m, n, newWindow);
                    samplingJoint(m,n,newWindow,newr);
                    inc(m, n, newWindow, newr, newTopic, newSenti);
                }// end for each word
            }// end for each document

            if (option.savestep > 0 && trnModel.liter % option.savestep == 0){
                //stepSave();
            }

        }// end iterations
        System.out.println("Gibbs sampling completed!\n");
        timer100.end();
        System.out.println("Sampling time: " + timer100.getElapsedTime());

        saveModel(option.dfile);
        return 0;


    }

    public void saveModel(String filename) throws IOException{


        System.out.println("Saving the final model!\n");
        trnModel.computePhiGl();
        trnModel.computePhiLoc();
        trnModel.liter--;
        String [] split= filename.split("\\.");
        filename = split[0];
        computePi();
        computeSentenceSenti();
        computePiLocSentence();
        computeOverallSentiLocSentence();
        trnModel.saveModel(filename);

    }

    public void computeOverallSentiLocSentence(){

        int offset = 0;
        double [] arr = new double[trnModel.locK + trnModel.locK];
        double [][] aspectProb = new double[trnModel.M-offset][trnModel.locK + trnModel.locK];
        double [][] glProb = new double[trnModel.M-offset][trnModel.glK + trnModel.glK];
        double [] garr = new double[trnModel.glK + trnModel.glK];
        try {



            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter (trnModel.dir +"/"+ trnModel.dfile + ".senTheta")));
            PrintWriter pwtpsmall = new PrintWriter(new BufferedWriter(new FileWriter (trnModel.dir +"/"+ trnModel.dfile + ".sentpsmall")));
            PrintWriter pw0 = new PrintWriter(new BufferedWriter(new FileWriter (trnModel.dir +"/"+ trnModel.dfile + ".senThetaSum")));
            PrintWriter pwOV = new PrintWriter(new BufferedWriter(new FileWriter (trnModel.dir +"/"+ trnModel.dfile + ".senThetaOverall")));
            PrintWriter pwAspect = new PrintWriter(new BufferedWriter(new FileWriter (trnModel.dir +"/"+ trnModel.dfile + ".aspect")));
            PrintWriter out = new PrintWriter(new FileWriter(new File(trnModel.dir + "/" +  trnModel.dfile + "-VisReviewsAspects.html")));

            int l;
            String [] sentiColors = {"green","red","black"};
            double posProb = 0, negProb = 0;
            for (int m = offset; m < trnModel.M; m++){
                pw.write(m+" ");
                pw0.write(m+" ");
                pwAspect.write(m+" ");
                posProb = 0;negProb = 0;
                for (int k = 0; k < arr.length ; k++)
                    aspectProb[m-offset][k] = 0;
                out.println("<h3>Document "+m+"</h3>");
                for (int s = 0; s < trnModel.data.docs[m].numberoflines; s++)
                {

                    int aspectno = 0;
                    int gk = 0;
                    l = 0;     double sumPosProb = 0, sumNegProb = 0;
                    for (int k = 0; k < trnModel.locK ; k++, aspectno++){
                        sumPosProb+= arr[aspectno] = this.sampleEquationLocSentence(m, s, l, k);
                        aspectProb[m-offset][aspectno] += arr[aspectno];
                    }
                    for (int k = 0; k < trnModel.glK ; k++, gk++){
                        sumPosProb+= garr[gk] = this.sampleEquationGlSentence(m, s, l, k);
                        glProb[m-offset][gk] += garr[gk];
                    }
                    l = 1;

                    for (int k = 0; k < trnModel.locK  ; k++, aspectno++){
                        sumNegProb += arr[aspectno] = this.sampleEquationLocSentence(m, s, l, k);
                        aspectProb[m-offset][aspectno] += arr[aspectno];
                    }

                    for (int k = 0; k < trnModel.glK ; k++, gk++){
                        sumNegProb+= garr[gk] = this.sampleEquationGlSentence(m, s, l, k);
                        glProb[m-offset][gk] += garr[gk];
                    }


                    List<Pair> p = Sort(arr);
                    Integer topic = 0;
                    int senti=0;
                    for (int k = 0; k < 1; k++){
                        topic = (Integer) p.get(k).first;
                        Double a1 = (Double) p.get(k).second;
                        pwAspect.write(s+":"+(topic)+":"+(a1)+" ");

                        if(topic > trnModel.locK){
                            pw.write(s+":1 ");
                            pwtpsmall.write("1");
                            negProb += (Double) p.get(k).second;
                        }
                        else {
                            pw.write(s+":0 ");
                            pwtpsmall.write("0");
                            posProb += (Double) p.get(k).second;
                        }

                        if(sumNegProb > sumPosProb ){
                            pw0.write(s + ":1 ");
                            senti = 1;

                        }
                        else {
                            pw0.write(s + ":0 ");
                            senti = 0;

                        }


                    }

                    out.print("<p style=\"color:"+sentiColors[senti]+";\">T"+topic+":");
                    out.print(" "+trnModel.data.docs[m].lines.get(s));
                    out.println("</p>");

                }
                if(posProb>negProb){
                    pwOV.println("0");
                }
                else  {
                    pwOV.println("1");
                }

                pw.write("\n");
                pwtpsmall.write("\n");
                pw0.write("\n");
                pwAspect.write("\n");


            }
            pw.close();
            pw0.close();
            pwOV.close();
            pwAspect.close();
            out.close();
            pwtpsmall.close();

            BinIO.storeDoubles(aspectProb, trnModel.dir+"/aspectSentiDist");

        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void computeSentenceSenti(){
        int offset = 0;

        try {

            ArrayList<ArrayList<Pair>> list = new ArrayList<ArrayList<Pair>>();

            String [] sentiColors = {"green","red","black"};
            System.out.println("Visualizing reviews...");


            PrintWriter pw0 = new PrintWriter(new BufferedWriter(new FileWriter (trnModel.dir +"/"+ trnModel.dfile + ".senSenti")));
            PrintWriter out = new PrintWriter(new FileWriter(new File(trnModel.dir + "/" +  trnModel.dfile + "-VisReviews.html")));

            int l;
            for (int m = offset; m < trnModel.M; m++){
                pw0.write(m+" ");
                out.println("<h3>Document "+m+"</h3>");
                for (int s = 0; s < trnModel.data.docs[m].numberoflines; s++)
                {
                    int positive = 0, negative = 1;
                    double posProb, negProb;

                    posProb =  this.sampleEquationPILoc(m, positive, s) + sampleEquationPIGL(m,positive);

                    negProb = this.sampleEquationPILoc(m, negative, s) +  sampleEquationPIGL(m,negative);
                    //System.out.println(m+" "+s+" "+posProb+" "+negProb);
                    if(negProb > posProb ){
                            out.print("<p style=\"color:"+sentiColors[1]+";\">T"+":");
//                            out.print("<p style=\"color:"+sentiColors[1]+";\">T"+sentence.getTopic()+":");
                            pw0.write(s+":1 ");
                        }
                        else {
                            out.print("<p style=\"color:"+sentiColors[0]+";\">T"+":");
//                            out.print("<p style=\"color:"+sentiColors[0]+";\">T"+sentence.getTopic()+":");
                            pw0.write(s+":0 ");
                        }
                    out.print(" "+trnModel.data.docs[m].lines.get(s));
                    out.println("</p>");
                }
                pw0.write("\n");


            }



            pw0.close();
            out.close();


        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void computePiLocSentence(){
        int K ;
        double [][] pi = new double[trnModel.M][trnModel.S];
        double [] arr = new double[trnModel.locK + trnModel.locK];
        String [] sentiColors = {"green","red","black"};
        try {

            PrintWriter out = new PrintWriter(new FileWriter(new File(trnModel.dir + "/" +  trnModel.dfile + "-VisReviews1.html")));
            int l;
            double posProb , negProb ;
            for (int m = 0; m < trnModel.M; m++){
                out.println("<h3>Document "+m+"</h3>");
                posProb = 0;negProb = 0;
                for (int s = 0; s < trnModel.data.docs[m].numberoflines; s++)
                {
                    l = 0; K = 0;
                    for (int k = 0; k < trnModel.locK ; k++)
                        posProb+=arr[K+k] = this.sampleEquationLocSentence(m, s, l, k);
                    l = 1;
                    K =  trnModel.locK;
                    for (int k = 0; k < trnModel.locK ; k++)
                        negProb+=arr[K+k] = this.sampleEquationLocSentence(m, s, l, k);

                    if(negProb > posProb ){
                        out.print("<p style=\"color:"+sentiColors[1]+";\">T"+":");
//                            out.print("<p style=\"color:"+sentiColors[1]+";\">T"+sentence.getTopic()+":");
                    }
                    else {
                        out.print("<p style=\"color:"+sentiColors[0]+";\">T"+":");
//                            out.print("<p style=\"color:"+sentiColors[0]+";\">T"+sentence.getTopic()+":");
                    }
                    out.print(" "+trnModel.data.docs[m].lines.get(s));
                    out.println("</p>");
                }
                pi[m][0]=posProb;
                pi[m][1]=negProb;

            }
            BinIO.storeDoubles( pi, trnModel.dir + File.separator + "jmts.pi" );
            out.close();

        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void computePi(){
        double [][] pi = new double[trnModel.M][trnModel.S];
        double [][] GLpi = new double[trnModel.M][trnModel.S];
        double [][] Locpi = new double[trnModel.M][trnModel.S];
        try {

            int senti;
            double posProb, negProb;
            for (int m = 0; m < trnModel.M; m++){
                senti = 0;
                posProb =  GLpi[m][senti]=  this.sampleEquationPIGL(m, senti);
                posProb +=  Locpi[m][senti] = this.sampleEquationPILoc(m, senti);

                senti = 1;

                negProb =  GLpi[m][senti] = this.sampleEquationPIGL(m, senti);
                negProb +=  Locpi[m][senti] = this.sampleEquationPILoc(m, senti);
                pi[m][0]=posProb;
                pi[m][1]=negProb;


            }
            BinIO.storeDoubles( pi, trnModel.dir + File.separator + "jmts.piCombine" );
            BinIO.storeDoubles( GLpi, trnModel.dir + File.separator + "jmts.GL" );
            BinIO.storeDoubles( Locpi, trnModel.dir + File.separator + "jmts.LoC" );

        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }


	/**
	 * Do sampling
	 * @param m document number
	 * @param n word number
	 * @return topic id
	 */
    public void dec(int mdoc, int n){
        try{
            int topic = trnModel.z[mdoc].get(n);
		    int v = trnModel.wv[mdoc].get(n);
		    int senti = trnModel.lv[mdoc].get(n);
		    int  word = trnModel.data.docs[mdoc].words[n];
            int s = trnModel.data.docs[mdoc].sentence[n];
            boolean r =  trnModel.rv[mdoc].get(n);
            double [] p;
            trnModel.ndl[mdoc][senti]--;
            if(r == true){
                trnModel.nwlGl[word][senti][topic] -= 1;
                trnModel.nwsumlGl[senti][topic] -= 1;
                trnModel.ndlGl[mdoc][senti][topic] -= 1; //ngld
                trnModel.ndsumGl[mdoc] -= 1;
                trnModel.data.docs[mdoc].ndvGl[v] -= 1;
                trnModel.data.docs[mdoc].ndvlGl [senti][v] -= 1;
                trnModel.ndlGlsum [mdoc][senti]--;
            }
            else if(r == false){ // local
                trnModel.nwlLoc[word][senti][topic] -= 1;
                trnModel.nwsumlLoc[senti][topic] -= 1;
                trnModel.ndlLoc[mdoc][senti][topic] -= 1; //ngld
                trnModel.ndsumLoc[mdoc] -= 1;
                trnModel.data.docs[mdoc].ndvLoc[v] -= 1;
                trnModel.data.docs[mdoc].ndvlLoc [senti][v] -= 1;
                trnModel.ndlLocsum [mdoc][senti] --;
                trnModel.data.docs[mdoc].ndvlLock[senti][v][topic]--;
                trnModel.data.docs[mdoc].ndvLock[v][topic] --;
                trnModel.ndLoc[mdoc][topic] --;
            }
            trnModel.data.docs[mdoc].ndsv[s][v] --;
            trnModel.data.docs[mdoc].ndv[v] --;
            trnModel.data.docs[mdoc].ndvl [senti][v] -= 1;
          
        }catch (Exception e){
			System.out.println("Error in sampling: dec " + e.getMessage());
			return;
        }
    }
    
    public int samplingWindow(int m, int n){
        try{
                
                double [] pv = new double [trnModel.data.docs[m].window];
                double se=0;
                int v;

                int s = trnModel.data.docs[m].sentence[n];
                
                if (trnModel.data.docs[m].numberoflines == 1)
                     return 0;
                if (trnModel.data.docs[m].numberoflines == 2){
                                    
                    
                    for (v = s ; v <= s+1; v++)
                        se += pv[v] = (trnModel.data.docs[m].ndsv[s][v]  + trnModel.gamma)/(trnModel.data.docs[m].nds[s]  + trnModel.T * trnModel.gamma);

                    double u1 = Math.random() * se;
                    for ( v = s+1; v <= s+1; v++)
                         pv[v] += pv[v - 1] ;
                    for ( v = s ; v <= s+1; v++)
                        if (pv[v] > u1) //sample topic w.r.t distribution p
				            break;

                    return v;
                    
                }
                    
                for (v = s ; v <= s+2; v++)
                    se += pv[v] = (trnModel.data.docs[m].ndsv[s][v]  + trnModel.gamma)/(trnModel.data.docs[m].nds[s]  + trnModel.T * trnModel.gamma);

                double u1 = Math.random() * se;
                for ( v = s+1; v <= s+2; v++)
                     pv[v] += pv[v - 1] ;
                for ( v = s ; v <= s+2; v++)
                     if (pv[v] > u1) //sample topic w.r.t distribution p
				break;
                return v;
        }catch (Exception e){
            System.out.println("Error in sampling: Window " + e.getMessage());
            return -1;
        }

    }

    public boolean samplingR(int m, int n, int v){
             try{
		        boolean r1;
                int s = trnModel.data.docs[m].sentence[n];
                double sumG = 0.0,sumL = 0.0;
                int Iv = windowForSentence(s);
                for ( v = Iv; v <= s; v++){
                       sumG =      (trnModel.data.docs[m].ndvGl [v] + trnModel.alphaMixGlobal)/(trnModel.data.docs[m].ndv[v]  + trnModel.alphaMixGlobal+trnModel.alphaMixLocal);
                       sumL =   (trnModel.data.docs[m].ndvLoc [v] + trnModel.alphaMixLocal)/(trnModel.data.docs[m].ndv[v]  + (trnModel.alphaMixGlobal+trnModel.alphaMixLocal));
                }
                r1 = false;
                double se= sumG + sumL;
                double u1 = Math.random() * se;
                if ( sumG > u1 ) r1 = true;

                return r1;
                          
             }catch (Exception e){
			System.out.println("Error in sampling: R" + e.getMessage());
			return false;
            }
    }

    public int windowForSentence(int s){
            int v;

                if (s == 0) v = 0;
                else if (s == 1) v = 0;
                else v = s - 2;
            return v;
        }



    public int inc(int m, int n, int v, boolean r, int topic, int senti){

                double []p;
                int w = trnModel.data.docs[m].words[n];
                int s = trnModel.data.docs[m].sentence[n];
                
                if(r == true){
                        trnModel.nwlGl[w][senti][topic] += 1;
                        trnModel.nwsumlGl[senti][topic] += 1;
                        trnModel.ndlGl[m][senti][topic] += 1; //ngld
                        trnModel.ndsumGl[m] += 1;
                        trnModel.data.docs[m].ndvGl[v] += 1;
                        trnModel.data.docs[m].ndvlGl [senti][v] += 1;
                        trnModel.ndlGlsum [m][senti]++;
                        if(topic>trnModel.glK) System.out.println(topic);
                }
                else {
                     if(topic > trnModel.locK) System.out.println(topic);
                        trnModel.nwlLoc[w][senti][topic] += 1;
                        trnModel.nwsumlLoc[senti][topic] += 1;
                        trnModel.ndlLoc[m][senti][topic] += 1; //ngld
                        trnModel.ndsumLoc[m] += 1;
                        trnModel.data.docs[m].ndvLoc[v] += 1;
                        trnModel.data.docs[m].ndvlLoc [senti][v] += 1;
                        trnModel.ndlLocsum [m][senti]++;
                        trnModel.data.docs[m].ndvlLock [senti][v][topic] ++;
                        trnModel.data.docs[m].ndvLock[v][topic] ++;
                        trnModel.ndLoc[m][topic] ++;
                }
                trnModel.ndl[m][senti]++;
                trnModel.data.docs[m].ndsv[s][v] ++;
                trnModel.data.docs[m].ndv[v] ++;
                trnModel.data.docs[m].ndvl [senti][v] += 1;
                trnModel.wv[m].set(n, v);
                trnModel.lv[m].set(n, senti);
                trnModel.rv[m].set(n, r);
                trnModel.z[m].set(n, topic);

            return 0;
        }


    public int samplingJoint(int m, int n,int v, boolean r){


        double sumProb = 0;newTopic = 0 ; newSenti = 0 ;int K = 0;
        double [][] probTable = null;

        if(r == true){

            K = trnModel.glK;
            probTable = new double[K][trnModel.S];

            for (int ti = 0; ti < K; ti++)
                for (int si = 0; si < trnModel.S; si++)
                    sumProb += probTable[ti][si] = sampleEquationGL( m,  n, si, ti, v);

        }
        else {
            K = trnModel.locK;
            probTable = new double[K][trnModel.S];

            for (int ti = 0; ti < K; ti++)
                for (int si = 0; si < trnModel.S; si++)
                    sumProb += probTable[ti][si] = sampleEquationLoc( m,  n, si, ti, v);

        }

        double randNo = Math.random() * sumProb;
        double tmpSumProb = 0;
        boolean found = false;



        for ( int  si = 0; si <  trnModel.S; si++) {
            for ( int ti = 0; ti <  K; ti++) {
                tmpSumProb += probTable[ti][si];
                if (randNo <= tmpSumProb) {
                    newTopic = ti;
                    newSenti = si;
                    found = true;
                }
                if (found) break;
            }
            if (found) break;
        }


        if(trnModel.data.docs[m].words[n] < 26) newSenti  = 0;
        else  if(trnModel.data.docs[m].words[n] < 49) newSenti = 1;

// required to activate for aspect sentiment classification experiments
//        if(trnModel.data.aspectVocab[w]&& priorAspect ) {
//            newTopic = trnModel.data.aspectWord2int.get(trnModel.data.localDict.getWord(w));
//        }




        return 0;
    }


    public double sampleEquationGL(int m, int n, int l, int k, int v){
        try{
            int w = trnModel.data.docs[m].words[n];
            int s = trnModel.data.docs[m].sentence[n];
            double g=0,sum=1,beta = 0.001, delta, alpha = 0.5;
            beta = trnModel.betaGlobal;
            delta = trnModel.delta;


            if ( l == 0 && w < 49 && w > 24) { beta = 0; }
            if ( l == 1 && w < 24) { beta = 0;    }
            if ( l == 0 && w < 24) { delta = 0.1;  }
            if ( l == 1 && w < 49 && w > 24) { delta =  trnModel.adeltaNeg;   }



            double p =
                    (trnModel.nwlGl [w][l][k] + beta )/(trnModel.nwsumlGl [l][k] + trnModel.V * beta) *
                            ( trnModel.data.docs[m].ndsv[s][v] + trnModel.gamma)/(trnModel.data.docs[m].nds[s] + trnModel.T * trnModel.gamma) *
                            (trnModel.data.docs[m].ndvGl [v] + trnModel.alphaMixGlobal)/(trnModel.data.docs[m].ndv[v] + trnModel.alphaMixGlobal + trnModel.alphaMixLocal) *
                            (trnModel.ndlGlsum [m][l] + delta  )/(trnModel.ndsumGl[m] + delta * trnModel.S) *
                            (trnModel.ndlGl [m][l][k] + trnModel.alphaGlobal    )/(trnModel.ndlGlsum[m][l] + trnModel.glK *trnModel.alphaGlobal ) ;

            return p;

        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationGL " );
            e.printStackTrace();
            return -1;
        }
    }

    public double sampleEquationPIGL(int m, int l){
        try{
            double delta = trnModel.delta;
            double p =      (trnModel.ndlGlsum [m][l] + delta  )/(trnModel.ndsumGl[m] + delta * trnModel.S) ;
            return p;

        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationGL " );
            e.printStackTrace();
            return -1;
        }
    }

    public double sampleEquationLoc(int m, int n, int l, int k, int v){
        try{
            int w = trnModel.data.docs[m].words[n];
            int s = trnModel.data.docs[m].sentence[n];
            double beta = trnModel.betaLocal;
            double p, delta = trnModel.delta;
            if ( l==0 && w < 49 && w > 24) { beta = 0; }
            if(l == 1 && w < 24) { beta = 0; }
            if(l == 0 && w < 24) { delta = 0.1;  }
            if(l == 1 && w < 49 && w > 24) { delta =  trnModel.adeltaNeg;  }

            p =     (trnModel.nwlLoc [w][l][k] + beta  )/(trnModel.nwsumlLoc [l][k] + trnModel.V *beta) *
                            ( trnModel.data.docs[m].ndsv[s][v] + trnModel.gamma )/(trnModel.data.docs[m].nds[s] + trnModel.T * trnModel.gamma) *
                            (trnModel.data.docs[m].ndvLoc [v] + trnModel.alphaMixLocal)/(trnModel.data.docs[m].ndv[v] + trnModel.alphaMixGlobal + trnModel.alphaMixLocal) *
                            (trnModel.data.docs[m].ndvlLoc [l][v] + delta    )/(trnModel.data.docs[m].ndvLoc [v] + delta * trnModel.S) *
                            (trnModel.data.docs[m].ndvlLock[l][v][k] + trnModel.alphaLocal )/(trnModel.data.docs[m].ndvlLoc [l][v] + trnModel.locK * trnModel.alphaLocal);
            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationLoc " );
            e.printStackTrace();
            return -1;
        }
    }

    public double sampleEquationPILoc(int m, int l){
        try{
            int nol = trnModel.data.docs[m].numberoflines;
            double p = 0;
            double delta = trnModel.delta;
            for (int v = 0; v < nol+2; v++)
                p +=             (trnModel.data.docs[m].ndvlLoc [l][v] + delta    )/(trnModel.data.docs[m].ndvLoc [v] + delta * trnModel.S) ;

            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationLoc " );
            e.printStackTrace();
            return -1;
        }
    }

    public double sampleEquationPILoc(int m, int l, int s){
        try{

            int v, end = 0;
            if (trnModel.data.docs[m].numberoflines == 1)
                end = 0;
            if (trnModel.data.docs[m].numberoflines == 2)
                end =  1;
            else end =  s+2;

            double p = 0;
            double delta = trnModel.delta;
            for( v = s; v <= end; v++)
                p +=             (trnModel.data.docs[m].ndvlLoc [l][v] + delta    )/(trnModel.data.docs[m].ndvLoc [v] + delta * trnModel.S) ;

            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationLoc " );
            e.printStackTrace();
            return -1;
        }
    }

    public double sampleEquationLocSentence(int m, int s, int l, int k ){ // m, s, v, w, l, k
        try{
            double p = 0; int v, end = 0;
            if (trnModel.data.docs[m].numberoflines == 1)end = 0;
            else if (trnModel.data.docs[m].numberoflines == 2)end =  1;
            else end =  s+2;


            for( v = s; v <= end; v++)
                for(int n=0;n<trnModel.data.docs[m].length;n++)
                    if(trnModel.data.docs[m].sentence[n]==s)
                        p+= sampleEquationLoc(m, n, l, k,  v);
            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationLoc " );
            e.printStackTrace();
            return -1;
        }
    }

    public double sampleEquationGlSentence(int m, int s, int l, int k ){ // m, s, v, w, l, k
        try{
            int v, end = 0;
            if (trnModel.data.docs[m].numberoflines == 1)
                end = 0;
            else if (trnModel.data.docs[m].numberoflines == 2)
                end =  1;
            else end =  s+2;

            double p = 0;
            for( v = s; v <= end; v++)
                for(int n=0;n<trnModel.data.docs[m].length;n++)
                    if(trnModel.data.docs[m].sentence[n]==s)
                        p+= sampleEquationGL(m, n, l, k,  v);
            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationGLsentence " );
            e.printStackTrace();
            return -1;
        }
    }

    public List<Pair> Sort(double [] arr){
        
        
        List<Pair> list = new ArrayList<Pair>();
        for(int k = 0; k < arr.length; k ++){
            Pair p = new Pair(k, arr[k], false);
            list.add(p);
        }
        Collections.sort(list);
        return list;
            
            
        
   }


}