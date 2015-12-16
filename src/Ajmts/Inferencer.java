
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

//import it.unimi.dsi.fastutil.io.BinIO;

import it.unimi.dsi.fastutil.io.BinIO;
import util.Dictionary;
import util.Pair;

import java.io.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Inferencer {
    // Train model
    public Model trainModel;
    public Dictionary globalDict;
    private JMTSCmdOption option;
    boolean priorAspect = true;

    private Model newModel;
    public int niters = 100, newTopic, newSenti;
//    double [][]sen ;

    //-----------------------------------------------------
    // Init method
    //-----------------------------------------------------
    public boolean init(JMTSCmdOption option){
            this.option = option;
            trainModel = new Model();

            if (!trainModel.initEstimatedModel(option))
                    return false;

            globalDict = trainModel.data.localDict;
		
//            computeTrnThetag();
//            computeTrnThetal();
            trainModel.computePhiGl();
            trainModel.computePhiLoc();

            return true;
    }
    
    public Model inference(){
            //System.out.println("inference");

            newModel = new Model();
            niters = option.niters;
            if (!newModel.initNewModel(option, trainModel)) return null;
            System.out.println("Sampling " + niters + " iteration for inference!");

   
            for (newModel.liter = 1; newModel.liter <= niters; newModel.liter++){
                    //System.out.println("Iteration " + newModel.liter + " ...");
                    // for all newz_i
                    for (int m = 0; m < newModel.M; ++m){
                            for (int n = 0; n < newModel.data.docs[m].length; n++){
                                    // (newz_i = newz[m][n]
                                    // sample from p(z_i|z_-1,w)
                                          dec(m, n);
                                          int newWindow = samplingWindow(m, n);
                                          boolean newr = samplingR(m, n, newWindow);
//                                          int topic = samplingTopic(m, n, v, r);
//                                          int senti = samplingSentiment(m, n, v, r, topic);
                                          this.samplingJoint(m, n, newWindow, newr);
                                          inc(m, n, newWindow, newr, newTopic, newSenti);
//                                    newModel.z[m].set(n, topic);
                            }
                    }//end foreach new doc
            }// end iterations

            System.out.println("Gibbs sampling for inference completed!");

        //newModel.computeThetaGlobal();
        //newModel.computeThetaLocal();
          computeNewPhiGl();
          computeNewPhiLoc();
//            computePerplexity();
        this.computeSentenceSenti();
        this.computeOverallSentiLocSentence();
        this.computeNewPi();
            newModel.liter--;
            System.out.println("Saving the inference outputs!");
            newModel.saveModel(newModel.dfile + "." + newModel.modelName);
            return newModel;
    }

    public void printList(List<Pair> list, String filename) throws IOException{

        PrintWriter pwf = new PrintWriter(new BufferedWriter(new FileWriter (trainModel.dir +"/"+ trainModel.dfile + filename)));
            for(Pair p: list){
                pwf.write(p.first+" "+p.second+"\n");
            }
            pwf.close();

    }


    public void computePerplexity() {

        double  sumnw=0, logProb=0, prob;
        int w;


        for (int m = 0; m < newModel.M; m++){

                sumnw += newModel.data.docs[m].length;
//                prob=0;
                for(int n = 0; n < newModel.data.docs[m].length; n++){
                    prob=0;
                    w = newModel.data.docs[m].words[n];
                    for (int senti = 0; senti < newModel.S; senti++){
                        for (int k = 0; k < newModel.glK; k++)
                            prob +=  newModel.glphi[senti][k][w] * newModel.gltheta[m][senti][k] ;
                    }
                    for (int senti = 0; senti < newModel.S; senti++){
                        for (int k = 0; k < newModel.locK; k++)
                            prob +=  newModel.locphi[senti][k][w]*newModel.loctheta[m][senti][k] ;
                    }
                    if(prob>0) prob =  Math.log(prob);
                    else       System.out.println(m+"\t"+prob);
                    logProb -= prob;
                }
        }
        System.out.println(logProb+ " "+sumnw);
        double p = logProb/sumnw;
        System.out.println(p);
        p = Math.exp(  p );
        System.out.println("Perplexity "+ (p));
    }

    //inference new model ~ getting data from a specified dataset
    public Model inference( Dataset newData){
            System.out.println("init new model");
            Model newModel = new Model();

            newModel.initNewModel(option, newData, trainModel);
            this.newModel = newModel;
            int sum =0;
             for (int m = 0; m < newModel.M; m++){
                            int p = newModel.data.docs[m].numberoflines;
                            System.out.println(m+" "+p);
                            sum+=p;
             }
               System.out.println(sum);
            System.out.println("Sampling " + niters + " iteration for inference!");
            for (newModel.liter = 1; newModel.liter <= niters; newModel.liter++){
                    //System.out.println("Iteration " + newModel.liter + " ...");

                    // for all newz_i
                    for (int m = 0; m < newModel.M; ++m){
                            for (int n = 0; n < newModel.data.docs[m].length; n++){
                                    // (newz_i = newz[m][n]
                                    // sample from p(z_i|z_-1,w)
//                                    int topic = infSampling(m, n);
                                          dec(m, n);
                                          int newWindow = samplingWindow(m, n);
                                          boolean newr = samplingR(m, n, newWindow);
//                                          int topic = samplingTopic(m, n, v, r);
//                                          int senti = samplingSentiment(m, n, v, r, topic);
                                           this.samplingJoint(m, n, newWindow, newr);
                                          inc(m, n, newWindow, newr, newTopic, newSenti);
//                                    newModel.z[m].set(n, topic);
                            }
                    }//end foreach new doc

            }// end iterations

            System.out.println("Gibbs sampling for inference completed!");

//            computeNewTheta();
            //computeNewLocPhi();
            //computeNewPi();
//            saveModelPI("a");
            //computeNewThetaLoc();

            newModel.liter--;
            return this.newModel;
    }


    //inference new model ~ getting dataset from file specified in option

    /**
     * do sampling for inference
     * m: document number
     * n: word number?
     */
    public void dec(int mdoc, int n){

        int topic = newModel.z[mdoc].get(n);
		int v = newModel.wv[mdoc].get(n);
		int senti = newModel.lv[mdoc].get(n);
		int  word = newModel.data.docs[mdoc].words[n];

        int s = newModel.data.docs[mdoc].sentence[n];
        boolean r =  newModel.rv[mdoc].get(n);
//        double [] p;
        newModel.ndl[mdoc][senti]--;

        if(r == true){
                        newModel.nwlGl[word][senti][topic] -= 1;
                        newModel.nwsumlGl[senti][topic] -= 1;
                        newModel.ndlGl[mdoc][senti][topic] -= 1; //ngld
                        newModel.ndsumGl[mdoc] -= 1;
                        newModel.data.docs[mdoc].ndvGl[v] -= 1;
                        newModel.data.docs[mdoc].ndvlGl [senti][v] -= 1;
                        newModel.ndlGlsum [mdoc][senti]--;
        }
        else if(r == false){ // local
                        newModel.nwlLoc[word][senti][topic] -= 1;
                        newModel.nwsumlLoc[senti][topic] -= 1;
                        newModel.ndlLoc[mdoc][senti][topic] -= 1; //ngld
                        newModel.ndsumLoc[mdoc] -= 1;
                        newModel.data.docs[mdoc].ndvLoc[v] -= 1;
                        newModel.data.docs[mdoc].ndvlLoc [senti][v] -= 1;
                        newModel.ndlLocsum [mdoc][senti] --;
                        newModel.data.docs[mdoc].ndvlLock[senti][v][topic]--;
                        newModel.data.docs[mdoc].ndvLock[v][topic] --;
                        newModel.ndLoc[mdoc][topic] --;
                }

                newModel.data.docs[mdoc].ndsv[s][v] --;
                newModel.data.docs[mdoc].ndv[v] --;
                newModel.data.docs[mdoc].ndvl [senti][v] -= 1;

      }



    public int samplingWindow(int m, int n){
        try{

            double [] pv = new double [newModel.data.docs[m].window];
            double se=0;
            int v;

            int s = newModel.data.docs[m].sentence[n];
            if (newModel.data.docs[m].numberoflines == 1)
                return 0;
            if (newModel.data.docs[m].numberoflines == 2){
                for (v = s ; v <= s+1; v++)
                    se += pv[v] = (newModel.data.docs[m].ndsv[s][v]  + newModel.gamma)/(newModel.data.docs[m].nds[s]  + newModel.T * newModel.gamma);

                double u1 = Math.random() * se;
                for ( v = s+1; v <= s+1; v++)
                    pv[v] += pv[v - 1] ;
                for ( v = s ; v <= s+1; v++)
                    if (pv[v] > u1) //sample topic w.r.t distribution p
                        break;
                return v;
            }
            for (v = s ; v <= s+2; v++)
                se += pv[v] = (newModel.data.docs[m].ndsv[s][v]  + newModel.gamma)/(newModel.data.docs[m].nds[s]  + newModel.T * trainModel.gamma);

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
            boolean r1; // = trainModel.rv[m].get(n);
            int s = newModel.data.docs[m].sentence[n];
            double sumG = 0.0,sumL = 0.0;
            int Iv = windowForSentence(s);
            for ( v = Iv; v <= s; v++){
                sumG =      (newModel.data.docs[m].ndvGl [v] + newModel.alphaMixGlobal)/(newModel.data.docs[m].ndv[v]  + newModel.alphaMixGlobal+newModel.alphaMixLocal);
                sumL =   (newModel.data.docs[m].ndvLoc [v] + newModel.alphaMixLocal)/(newModel.data.docs[m].ndv[v]  + (newModel.alphaMixGlobal+newModel.alphaMixLocal));
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
    public int samplingJoint(int m, int n,int v, boolean r){


        double sumProb = 0;newTopic = 0 ; newSenti = 0 ;
        int K;
        double [][] probTable = null;

        if(r == true){

            K = newModel.glK;
            probTable = new double[K][newModel.S];

            for (int ti = 0; ti < K; ti++)
                for (int si = 0; si < newModel.S; si++)
                    sumProb += probTable[ti][si] = sampleEquationGL( m,  n, si, ti, v);

        }
        else {
            K = newModel.locK;
            probTable = new double[K][newModel.S];

            for (int ti = 0; ti < K; ti++)
                for (int si = 0; si < newModel.S; si++)
                    sumProb += probTable[ti][si] = sampleEquationLoc( m,  n, si, ti, v);

        }

        double randNo = Math.random() * sumProb;
        double tmpSumProb = 0;
        boolean found = false;



        for ( int  si = 0; si <  newModel.S; si++) {
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

//                }

        int w = newModel.data.docs[m].words[n];
        //if(trainModel.data.docs[m].words[n] < 1028-3) newSenti  = 0;
        //else  if(trainModel.data.docs[m].words[n] < 2375-12) newSenti = 1;
            if(w < 26) newSenti  = 0;
          else  if(w < 49) newSenti = 1;

//
//        if(newModel.data.aspectVocab[w] && priorAspect)
//             newTopic = newModel.data.aspectWord2int.get(newModel.data.localDict.getWord(w));
//



        return 0;
    }

    public int inc(int m, int n, int v, boolean r, int topic, int senti){

                double []p;
                int w = newModel.data.docs[m].words[n];
                int s = newModel.data.docs[m].sentence[n];
                if(r == true){
                        newModel.nwlGl[w][senti][topic] += 1;
                        newModel.nwsumlGl[senti][topic] += 1;
                        newModel.ndlGl[m][senti][topic] += 1; //ngld
                        newModel.ndsumGl[m] += 1;
                        newModel.data.docs[m].ndvGl[v] += 1;
                        newModel.data.docs[m].ndvlGl [senti][v] += 1;
                        newModel.ndlGlsum [m][senti]++;
                }
                else {
                        newModel.nwlLoc[w][senti][topic] += 1;
                        newModel.nwsumlLoc[senti][topic] += 1;
                        newModel.ndlLoc[m][senti][topic] += 1; //ngld
                        newModel.ndsumLoc[m] += 1;
                        newModel.data.docs[m].ndvLoc[v] += 1;
                        newModel.data.docs[m].ndvlLoc [senti][v] += 1;
                        newModel.ndlLocsum [m][senti]++;
                        newModel.data.docs[m].ndvlLock [senti][v][topic] ++;
                        newModel.data.docs[m].ndvLock[v][topic] ++;
                        newModel.ndLoc[m][topic] ++;
                }
                newModel.ndl[m][senti]++;
                newModel.data.docs[m].ndsv[s][v] ++;
                newModel.data.docs[m].ndv[v] ++;
                newModel.data.docs[m].ndvl [senti][v] += 1;
                newModel.wv[m].set(n, v);
                newModel.lv[m].set(n, senti);
                newModel.rv[m].set(n, r);
                newModel.z[m].set(n, topic);

            return 0;
        }


    public double sampleEquationGL(int m, int n, int l, int k, int v){ // m, s, v, w, l,
        try{
        //    int w = newModel.data.docs[m].words[n];
            int s = newModel.data.docs[m].sentence[n];
            double beta = 0.001, delta;
            beta = trainModel.betaGlobal;
            delta = trainModel.delta;
            int _w = newModel.data.docs[m].words[n];
            int w = newModel.data.lid2gid.get(_w);
            if(l==0&&w < 49 && w > 24) { beta = 0; }
            if(l == 1 && w < 24) { beta = 0;    }
            if(l == 0 && w < 24) { delta = 0.1;  }
            if(l == 1 && w < 49 && w > 24) { delta =  trainModel.adeltaNeg;   }



            double p =
                    (trainModel.nwlGl [w][l][k] + newModel.nwlGl [_w][l][k] + beta )/(trainModel.nwsumlGl [l][k] + newModel.nwsumlGl [l][k] + trainModel.V * beta) *
                            ( newModel.data.docs[m].ndsv[s][v] + newModel.gamma)/(newModel.data.docs[m].nds[s] + newModel.T * newModel.gamma) *
                            (newModel.data.docs[m].ndvGl [v] + newModel.alphaMixGlobal)/(newModel.data.docs[m].ndv[v] + newModel.alphaMixGlobal + newModel.alphaMixLocal) *
                            (newModel.ndlGlsum [m][l] + delta  )/(newModel.ndsumGl[m] + delta * newModel.S) *
                            (newModel.ndlGl [m][l][k] + newModel.alphaGlobal    )/(newModel.ndlGlsum[m][l] + newModel.glK *newModel.alphaGlobal ) ;

            return p;

        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationGL " );
            e.printStackTrace();
            return -1;
        }
    }



    public double sampleEquationLoc(int m, int n, int l, int k, int v){ // m, s, v, w, l, k
        try{
            int _w = newModel.data.docs[m].words[n];
            int w = newModel.data.lid2gid.get(_w);
            int s = newModel.data.docs[m].sentence[n];
            double p = 0;

            double beta = trainModel.betaLocal;
            double delta = trainModel.delta;
            if ( l==0 && w < 49 && w > 24) { beta = 0; }
            if(l == 1 && w < 24) { beta = 0; }

            if(l == 0 && w < 24) { delta = 0.1;  }
            if(l == 1 && w < 49 && w > 24) { delta =  trainModel.adeltaNeg;  }

            p =     (trainModel.nwlLoc [w][l][k] + newModel.nwlLoc [_w][l][k]  + beta  )/(trainModel.nwsumlLoc [l][k] + newModel.nwsumlLoc [l][k] + trainModel.V *beta) *
                    ( newModel.data.docs[m].ndsv[s][v] + newModel.gamma )/(newModel.data.docs[m].nds[s] + newModel.T * newModel.gamma) *
                    (newModel.data.docs[m].ndvLoc [v] + newModel.alphaMixLocal)/(newModel.data.docs[m].ndv[v] + newModel.alphaMixGlobal + newModel.alphaMixLocal) *
                    (newModel.data.docs[m].ndvlLoc [l][v] + delta    )/(newModel.data.docs[m].ndvLoc [v] + delta * newModel.S) *
                    (newModel.data.docs[m].ndvlLock[l][v][k] + newModel.alphaLocal )/(newModel.data.docs[m].ndvlLoc [l][v] + newModel.locK * newModel.alphaLocal);
            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationLoc " );
            e.printStackTrace();
            return -1;
        }
    }


    public int randomGenerate(double [] p){

        int k;
        for ( k = 1; k < p.length; k++)
            p[k]  += p[k - 1];

        double u = Math.random() * p[p.length - 1];
        for ( k = 0; k < p.length-1; k++)
            if ( p[k] > u ){ //sample topic w.r.t distribution p
                    break;
            }

        return k;
        }
                      /*
    protected void computeNewPhi(){
        for (int k = 0; k < newModel.K; k++){
            for (int _w = 0; _w < newModel.V; _w++){
                Integer id = newModel.data.lid2gid.get(_w);

                if (id != null){
                    newModel.phi[k][_w] = (trnModel.nw[id][k] + newModel.nw[_w][k] + newModel.beta) / (newModel.nwsum[k] + newModel.nwsum[k] + trnModel.V * newModel.beta);
                }
            }//end foreach word
        }// end foreach topic
    }

    public void computePhiGl(){
        for (int senti = 0; senti < trnModel.S; senti++)
            for (int k = 0; k < trnModel.glK; k++){
                for (int w = 0; w < trnModel.V; w++){
                    trnModel.glphi [ senti ][k][ w ] = (trnModel.nwlGl [w][senti][k] + trnModel.betaGlobal) / (trnModel.nwsumlGl [senti][k] + trnModel.V * trnModel.betaGlobal);
                }
            }

    }

    public void computePhiLoc(){
        try{
            double [][] phi = new double [ trnModel.locK + trnModel.locK ][trnModel.V];
            for (int senti = 0; senti < trnModel.S; senti++)
                for (int k = 0; k < trnModel.locK; k++){
                    for (int w = 0; w < trnModel.V; w++){
                        phi[ senti + k ][ w ] = trnModel.locphi[senti][k][w] = ( trnModel.nwlLoc[w][senti][k] +  trnModel.betaLocal) / (trnModel.nwsumlLoc [senti][k]  + trnModel.V * trnModel.betaLocal);
                    }
                }

            BinIO.storeDoubles(phi, trnModel.dir + File.separator + "jmts.phi");
        }
        catch(IOException e){

        }
    }
                    */
    public void computeNewPhiGl(){
        for (int senti = 0; senti < newModel.S; senti++)
            for (int k = 0; k < newModel.glK; k++){
                for (int w = 0; w < newModel.V; w++){
                    Integer id = newModel.data.lid2gid.get(w);

                    if (id != null){
                        newModel.glphi [ senti ][ k ][ w ] = (trainModel.nwlGl[id][senti][k]  + newModel.nwlGl [w][senti][k] + newModel.betaGlobal) / (newModel.nwsumlGl [senti][k] + newModel.V * newModel.betaGlobal);
                    }
                }
            }

    }
    public void computeNewPhiLoc(){
    try {
        //double [][] phi = new double [ trainModel.locK + trainModel.locK ][newModel.V];

        for (int senti = 0; senti < newModel.S; senti++)
            for (int k = 0; k < newModel.locK; k++){
                for (int _w = 0; _w < newModel.V; _w++){

                    Integer id = newModel.data.lid2gid.get(_w);

                    if (id != null){
                        //	newModel.phi[k][_w] = (trainModel.nw[id][k] + newModel.nw[_w][k] + newModel.beta) / (newModel.nwsum[k] + newModel.nwsum[k] + trainModel.V * newModel.beta);

                        //phi[ senti + k ][ _w ] =
                                newModel.locphi[senti][k][_w] = ( trainModel.nwlLoc[id][senti][k]  + newModel.nwlLoc[ _w ][ senti ][ k ] +  newModel.betaLocal) / (newModel.nwsumlLoc [senti][k]  + newModel.V * newModel.betaLocal);
                    }
                }
            }
        //BinIO.storeDoubles(phi, trainModel.dir + File.separator + "jmtsInf.phi");
    }
    catch(Exception e){

    }
    }


    public void computeOverallSentiLocSentence(){
        int K =0;
        int locK = newModel.locK;
        int glK = newModel.glK;
        double [] arr = new double[locK +locK];
        double [][] aspectProb = new double[newModel.M][locK + locK];
        double [][] glProb = new double[newModel.M][glK + glK];
        double [] garr = new double[glK + glK];
        try {

            ArrayList<ArrayList<Pair>> list = new ArrayList<ArrayList<Pair>>();



            int l;
            double posProb = 0, negProb = 0;
            for (int m = 0; m < newModel.M; m++){

                posProb = 0;negProb = 0;
                for (int k = 0; k < arr.length ; k++)
                    aspectProb[m][k] = 0;
                for (int s = 0; s < newModel.data.docs[m].numberoflines; s++)
                {
                    int aspectno = 0;
                    int gk = 0;
                    l = 0; K = 0;    double sumPosProb = 0, sumNegProb = 0;
                    for (int k = 0; k < trainModel.locK ; k++, aspectno++){
                        sumPosProb+= arr[aspectno] = this.sampleEquationLocSentence(m, s, l, k);
                        aspectProb[m][aspectno] += arr[aspectno];
                    }
                    for (int k = 0; k < trainModel.glK ; k++, gk++){
                        sumPosProb+= garr[gk] = this.sampleEquationGlSentence(m, s, l, k);
                        glProb[m][gk] += garr[gk];
                    }
                    l = 1;

                    for (int k = 0; k < trainModel.locK  ; k++, aspectno++){
                        sumNegProb += arr[aspectno] = this.sampleEquationLocSentence(m, s, l, k);
                        aspectProb[m][aspectno] += arr[aspectno];
                    }

                    for (int k = 0; k < trainModel.glK ; k++, gk++){
                        sumPosProb+= garr[gk] = this.sampleEquationGlSentence(m, s, l, k);
                        glProb[m][gk] += garr[gk];
                    }


                    List<Pair> p = Sort(arr);
                    Integer a = 0;
                    for (int k = 0; k < 1; k++){
                        a = (Integer) p.get(k).first;
                        Double a1 = (Double) p.get(k).second;
                        //pwAspect.write(s+":"+(a)+":"+(a1)+" ");
                        if(a > trainModel.locK){
                            negProb += (Double) p.get(k).second;
                        }
                        else {
                            posProb += (Double) p.get(k).second;
                        }


                    }

                }


            }


            BinIO.storeDoubles(aspectProb, trainModel.dir+File.separator+newModel.dfile+".aspectSentiDist");
        /*    BinIO.storeDoubles(glProb, trnModel.dir+"/globalSentiDist");*/

        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public void computeOverallSentiLocSentenceExp(){
        int K =0;
        int locK = newModel.locK;
        int glK = newModel.glK;
        double [] arr = new double[locK +locK];
        double [][] aspectProb = new double[newModel.M][locK + locK];
        double [][] glProb = new double[newModel.M][glK + glK];
        double [] garr = new double[glK + glK];
        try {

            ArrayList<ArrayList<Pair>> list = new ArrayList<ArrayList<Pair>>();


            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter (newModel.dir +"/"+ newModel.dfile + ".senTheta")));
            PrintWriter pw0 = new PrintWriter(new BufferedWriter(new FileWriter (newModel.dir +"/"+ newModel.dfile + ".senThetaSum")));
            PrintWriter pwOV = new PrintWriter(new BufferedWriter(new FileWriter (newModel.dir +"/"+ newModel.dfile + ".senThetaOverall")));
            PrintWriter pwAspect = new PrintWriter(new BufferedWriter(new FileWriter (newModel.dir +"/"+ newModel.dfile + ".aspect")));

            int l;
            double posProb = 0, negProb = 0;
            for (int m = 0; m < newModel.M; m++){

                pw.write(m+" ");
                pw0.write(m+" ");
                pwAspect.write(m+" ");

                posProb = 0;negProb = 0;
                for (int k = 0; k < arr.length ; k++)
                    aspectProb[m][k] = 0;
                for (int s = 0; s < newModel.data.docs[m].numberoflines; s++)
                {
                    int aspectno = 0;
                    int gk = 0;
                    l = 0; K = 0;    double sumPosProb = 0, sumNegProb = 0;
                    for (int k = 0; k < trainModel.locK ; k++, aspectno++){
                        sumPosProb+= arr[aspectno] = this.sampleEquationLocSentence(m, s, l, k);
                        aspectProb[m][aspectno] += arr[aspectno];
                    }
                    for (int k = 0; k < trainModel.glK ; k++, gk++){
                        sumPosProb+= garr[gk] = this.sampleEquationGlSentence(m, s, l, k);
                        glProb[m][gk] += garr[gk];
                    }
                    l = 1;

                    for (int k = 0; k < trainModel.locK  ; k++, aspectno++){
                        sumNegProb += arr[aspectno] = this.sampleEquationLocSentence(m, s, l, k);
                        aspectProb[m][aspectno] += arr[aspectno];
                    }

                    for (int k = 0; k < trainModel.glK ; k++, gk++){
                        sumPosProb+= garr[gk] = this.sampleEquationGlSentence(m, s, l, k);
                        glProb[m][gk] += garr[gk];
                    }


                    List<Pair> p = Sort(arr);
                    Integer a = 0;
                    for (int k = 0; k < 1; k++){
                        a = (Integer) p.get(k).first;
                        Double a1 = (Double) p.get(k).second;
                        //pwAspect.write(s+":"+(a)+":"+(a1)+" ");
                        if(a > trainModel.locK){
                            pw.write(s+":1 ");
                            negProb += (Double) p.get(k).second;
                        }
                        else {
                            pw.write(s+":0 ");
                            posProb += (Double) p.get(k).second;
                        }

                        if(sumNegProb > sumPosProb ){
                            pw0.write(s+":1 ");
                        }
                        else {
                            pw0.write(s+":0 ");
                        }

                    }

                }

                if(posProb>negProb){
                    pwOV.println("0");
                }
                else  {
                    pwOV.println("1");
                }

                pw.write("\n");
                pw0.write("\n");
                pwAspect.write("\n");


            }

            pw.close();
            pw0.close();
            pwOV.close();
            pwAspect.close();

            BinIO.storeDoubles(aspectProb, trainModel.dir+File.separator+newModel.dfile+".aspectSentiDist");
        /*    BinIO.storeDoubles(glProb, trnModel.dir+"/globalSentiDist");*/

        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public double sampleEquationLocSentence(int m, int s, int l, int k ){ // m, s, v, w, l, k
        try{
            double p = 0; int v, end = 0;
            if (newModel.data.docs[m].numberoflines == 1)end = 0;
            else if (newModel.data.docs[m].numberoflines == 2)end =  1;
            else end =  s+2;


            for( v = s; v <= end; v++)
                for(int n=0;n<newModel.data.docs[m].length;n++)
                    if(newModel.data.docs[m].sentence[n]==s)
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
            if (newModel.data.docs[m].numberoflines == 1)
                end = 0;
            if (newModel.data.docs[m].numberoflines == 2)
                end =  1;
            else end =  s+2;

            double p = 0;
            for( v = s; v <= end; v++)
                for(int n=0;n<newModel.data.docs[m].length;n++)
                    if(newModel.data.docs[m].sentence[n]==s)
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
//        for(int k = 0; k < trnModel.locK; k++)
//            System.out.print( list.get(k).first+ ":"+ list.get(k).second);
        return list;



    }

    public void computeNewPi(){
        int K;
        double [][] pi = new double[newModel.M][newModel.S];
        double [][] GLpi = new double[newModel.M][newModel.S];
        double [][] Locpi = new double[newModel.M][newModel.S];
//        double [] arr = new double[trainModel.locK + newModel.locK];
        try {

            int senti;
            double posProb = 0, negProb = 0,a,b,c,d;
            for (int m = 0; m < newModel.M; m++){
                senti = 0; K = 0;
                posProb =  GLpi[m][senti]=  this.sampleEquationPIGL(m, senti);
                posProb +=  Locpi[m][senti] = this.sampleEquationPILoc(m, senti);

                senti = 1;

                negProb =  GLpi[m][senti] = this.sampleEquationPIGL(m, senti);
                negProb +=  Locpi[m][senti] = this.sampleEquationPILoc(m, senti);
                pi[m][0]=posProb;
                pi[m][1]=negProb;


            }
            BinIO.storeDoubles( pi, newModel.dir + File.separator +newModel.dfile+".jmts.pi" );
          //  BinIO.storeDoubles( GLpi, newModel.dir + File.separator + "jmts.GL" );
            //BinIO.storeDoubles( Locpi, newModel.dir + File.separator + "jmts.LoC" );

        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    public double sampleEquationPIGL(int m, int l){ // m, s, v, w, l,
            return     (newModel.ndlGlsum [m][l] + trainModel.delta  )/(newModel.ndsumGl[m] + trainModel.delta * trainModel.S) ;
    }

    public double sampleEquationPILoc(int m, int l){ // m, s, v, w, l, k
        try{
            int nol = newModel.data.docs[m].numberoflines;
            double p = 0,   gamma = 0.01;
            double delta = trainModel.delta;
            for (int v = 0; v < nol+2; v++)
                p +=             (newModel.data.docs[m].ndvlLoc [l][v] + delta    )/(newModel.data.docs[m].ndvLoc [v] + delta * trainModel.S) ;

            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: sampleEquationLoc " );
            e.printStackTrace();
            return -1;
        }
    }

    public void computeSentenceSenti(){
        try {
            PrintWriter pw0 = new PrintWriter(new BufferedWriter(new FileWriter (newModel.dir +"/"+ newModel.dfile + ".senSenti")));
            for (int m = 0; m < newModel.M; m++){
                pw0.write(m+" ");
                for (int s = 0; s < newModel.data.docs[m].numberoflines; s++)
                {
                    int positive = 0, negative = 1;
                    double posProb, negProb;
                    posProb =  this.sampleEquationPILoc(m, positive, s) + sampleEquationPIGL(m,positive);
                    negProb = this.sampleEquationPILoc(m, negative, s) +  sampleEquationPIGL(m,negative);
                    if(negProb > posProb )
                        pw0.write(s+":1 ");
                    else
                        pw0.write(s+":0 ");
                    pw0.write("\n");
                }
            }
            pw0.close();

        } catch (IOException ex) {
            Logger.getLogger(Estimator.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    public double sampleEquationPILoc(int m, int l, int s){ // m, s, v, w, l, k
        try{
            int v, end = 0;
            if (newModel.data.docs[m].numberoflines == 1)
                end = 0;
            else if (newModel.data.docs[m].numberoflines == 2)
                end =  1;
            else end =  s+2;

            double p = 0;
            double delta = newModel.delta;
            for( v = s; v <= end; v++)
                p +=             (newModel.data.docs[m].ndvlLoc [l][v] + delta    )/(newModel.data.docs[m].ndvLoc [v] + delta * newModel.S) ;

            return p;
        }catch (Exception e){
            System.out.println("Error in sampling: inferencer.sampleEquationPILoc " );
            e.printStackTrace();
            return -1;
        }
    }



}
