
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
import util.Constants;
import util.Pair;
import util.Dictionary;

import java.io.*;
import java.util.*;

public class Model {

    //---------------------------------------------------------------
    //	Class Variables
    //---------------------------------------------------------------

    public static String tassignSuffix;	//suffix for topic assignment file
    public static String thetaSuffix;		//suffix for theta (topic - document distribution) file
    public static String phiSuffix;		//suffix for phi file (topic - word distribution) file
    public static String piSuffix;		//suffix for pi file (topic - word distribution) file
    public static String winpiSuffix;		//suffix for phi file (topic - word distribution) file
    public static String glthetaSuffix;		//suffix for global theta (topic - document distribution) file
    public static String glphiSuffix;		//suffix for global phi file (topic - word distribution) file
    public static String locthetaSuffix;		//suffix for local theta (topic - document distribution) file
    public static String locphiSuffix;		//suffix for local  phi file (topic - word distribution) file
    public static String othersSuffix; 	//suffix for containing other parameters
    public static String twordsSuffix;		//suffix for file containing words-per-topics
    public static String gltwordsSuffix;		//suffix for file containing words-per-topics
    public static String ngltwordsSuffix, nloctwordsSuffix,piSuffixRating;		//suffix for file containing words-per-topics
    public static String pgltwordsSuffix, ploctwordsSuffix,locRatingSuffix;		//suffix for file containing words-per-topics
    public static String tdocsSuffix;
    //---------------------------------------------------------------
    //	Model Parameters and Variables
    //---------------------------------------------------------------

    public String wordMapFile; 		//file that contain word to id map
    public String trainlogFile; 	//training log file

    public String dir;
    public String dfile;
    public String modelName;
    public int modelStatus; 		//see Constants class for status of model
    public Dataset data;			// link to a dataset

    public int M; //dataset size (i.e., number of docs)
    public int V; //vocabulary size
    //	public int K; //number of topics
    public int T; //number of topics
    public int S; //number of topics max sentence and window for init
    public int glK; //number of global topics
    public int locK; //number of local topics
    public double gamma, delta, adelta, adeltaNeg; //LDA  hyperparameters
    public double alphaMixGlobal, alphaMixLocal; //LDA  hyperparameters
    public double alphaGlobal, betaGlobal; //LDA  hyperparameters
    public double alphaLocal, betaLocal; //LDA  hyperparameters
    public double perplexity;
    public int niters; //number of Gibbs sampling iteration
    public int liter; //the iteration at which the model was saved
    public int savestep; //saving period
    public int twords; //print out top words per each topic
    public int withrawdata;
    public int Rating;
    public  int toptopics = 3;

    // Estimated/Inferenced parameters
    public double [][] theta; //theta: document - topic distributions, size M x K
    public double [][][] gltheta; //theta: document - global topic distributions, size M x K
    public double [][][] glphi; // phi: gobal topic-word distributions, size K x V
    public double [][][] loctheta; //theta: document - local topic distributions, size M x K
    public double [][][] locphi; // phi: local topic-word distributions, size K x V
    public Vector<Integer> [] z; //topic assignments for words, size M x doc.size()
//    public int [][] nw; //nw[i][j]: number of instances of word/term i assigned to topic j, size V x K
    public int [][] nd; //nd[i][j]: number of words in document i assigned to topic j, size M x K
    public int [] nwsum; //nwsum[j]: total number of words assigned to topic j, size K


    public int [][][] nwlGl;
    public int [][] nwsumlGl;
    public int [][][] nwlLoc;
    public int [][] nwsumlLoc;
    public int [][][] ndlGl;
    public int [][] ndlGlsum;
    public int [] ndsumGl;
    public int [] ndsum;
    public int [][][] ndlLoc;
    public int [][] ndLoc;
    public int [][] ndl;
    public int [][] ndlLocsum;
    public int [] ndsumLoc;

    public Vector<Boolean> [] rv;
    public Vector<Integer> [] wv;
    public Vector<Integer> [] lv;
    public final boolean global = true;
    public final boolean local = false;
    public final int positiveSenti = 0;
    String [][] aspects;
    double [][] aspectsProb;
    // temp variables for sampling
    protected double [] p;

    //---------------------------------------------------------------
    //	Constructors
    //---------------------------------------------------------------

    public Model(){
        setDefaultValues();
    }


    /**
     * Set default values for variables
     */

    public void setDefaultValues(){
        wordMapFile = "wordmap.txt";
        trainlogFile = "trainlog.txt";
        tassignSuffix = ".tassign";
        glthetaSuffix = ".theta-gl";
        glphiSuffix = ".phi-gl";
        phiSuffix = ".phi";
        piSuffix = ".pi";
        winpiSuffix = ".winpi";
        locthetaSuffix = ".theta-loc";
        locphiSuffix = ".phi-loc";
        othersSuffix = ".others";
        twordsSuffix = ".twords";
        gltwordsSuffix = ".twords-gl";

        nloctwordsSuffix = ".twords-loc-neg";
        ploctwordsSuffix = ".twords-loc-pos";
        pgltwordsSuffix = ".twords-gl-pos";
        ngltwordsSuffix = ".twords-gl-neg";
        piSuffixRating = "-rating.csv";
        locRatingSuffix = "locRating";
        thetaSuffix = "-theta.csv";
        tdocsSuffix = ".tdocs";

        dir = "./";
        dfile = "trndocs.dat";
        modelName = "model-final";
        modelStatus = Constants.MODEL_STATUS_UNKNOWN;

        M = 0;
        V = 0;
        S = 2;
        T = 3;
        Rating = 5;
        glK = 30;
        locK = 10;
        niters = 1000;
        liter = 0;
        alphaMixGlobal = 0.1;
        alphaMixLocal = 0.4;
        alphaGlobal = 5.0 / glK;
        alphaLocal = 15.0 / locK;
        betaGlobal = 0.01;
        betaLocal = 0.01;

        gamma = 0.4;
        delta = 0.01;

        toptopics = 3;
        z = null;
        rv = null;
        wv = null;
        lv = null;
        nwlGl = null;
        nwsumlGl = null;
        nwlLoc = null;
        nwsumlLoc = null;
        ndlGl = null;//[m][l][k]
        ndlGlsum = null;
        ndsumGl = null;
        ndlLoc = null;//[m][l][k]
        ndlLocsum = null;
        ndsumLoc = null;
        nd = null;
        nwsum = null;
        ndsum = null;
        gltheta = null;
        glphi = null;
        loctheta = null;
        locphi = null;
        ndLoc = null;
        ndl =  null;

    }
    public void allocateMemoryIntializeZero()  {

        try{
            //       System.out.println(locK+"   "+glK);
            int m, n, w, k, s, v, l;
            glphi = new double[S][glK][V];
            locphi = new double[S][locK][V];

            nwlGl = new int[V][S][glK];
            nwsumlGl = new int[S][glK];
            nwlLoc = new int[V][S][locK];
            nwsumlLoc = new int [S][locK];
            ndsum = new int [M];


            for (l = 0; l < S; l++)
                for (w = 0; w < V; w++){
                    for (k = 0; k < glK; k++){
                        nwlGl[w][l][k] = 0;
                        nwsumlGl[l][k] = 0;
                    }
                    for (k = 0; k < locK; k++){
                        nwlLoc[w][l][k] = 0;
                        nwsumlLoc[l][k] = 0;
                    }
                }

            ndlGl = new int[M][S][glK];
            ndlGlsum = new int[M][S];
            ndsumGl = new int[M];

            ndlLoc = new int[M][S][locK];
            ndLoc = new int[M][locK];
            ndlLocsum = new int[M][S];
            ndl = new int[M][S];
            ndsumLoc = new int[M];

            for (m = 0; m < M; m++){
                for (l = 0; l < S; l++){

                    for (k = 0; k < glK; k++)  ndlGl[m][l][k] = 0;
                    for (k = 0; k < locK; k++) ndlLoc[m][l][k] = 0;

                    ndlGlsum[m][l] = 0;
                    ndlLocsum[m][l] = 0;
                    ndl[m][l]= 0;

                }
                ndsumGl[m] = 0;
                ndsumLoc [m] = 0;
                ndsum[m]=0;
            }
            for (m = 0; m < M; m++){
                for (  l = 0; l < S; l++)
                    for ( v = 0; v < data.docs[m].window; v++)
                    {
                        for (k = 0; k < locK; k++){
                            data.docs[m].ndvlLock[l][v][k] = 0;
                            data.docs[m].ndvLock[v][k] = 0;
                            ndLoc[m][k] =0;
                        }
                    }
                for ( v = 0; v < data.docs[m].window; v++){
                    for ( s = 0; s < data.docs[m].numberoflines; s++)
                        data.docs[m].ndsv[s][v] =0;
                        data.docs[m].ndv[v] =0;
                        data.docs[m].ndvLoc[v] = 0;
                        data.docs[m].ndvGl[v] = 0;

                    for (  l = 0; l < S; l++)  {
                        data.docs[m].ndvlLoc [l][v] = 0;
                        data.docs[m].ndvlGl [l][v] = 0;
                        data.docs[m].ndvl [l][v] = 0;
                    }

                }
                for ( s = 0; s < data.docs[m].numberoflines; s++)
                    data.docs[m].nds[s] = 0;
            }
        }catch(Exception e){
            System.out.println("["+this.getClass()+".allocateMemoryIntializeZero()]  " + e.getMessage());
            return;

        }
    }

    public void initiliazieWithRandomValues()  {

        try{
            int m, n, w, k, s, v, senti, R;
            z = new Vector[M];
            rv = new Vector[M];
            wv = new Vector[M];
            lv = new Vector[M];
            for (m = 0; m < data.M; m++){
                int N = data.docs[m].length;
                z[m] = new Vector<Integer>();
                rv[m] = new Vector<Boolean>();
                wv[m] = new Vector<Integer>();
                lv[m] = new Vector<Integer>();

                //initilize for z

                Random rb = new Random();
                for (n = 0; n < N; n++){
                    s =    data.docs[m].sentence[n];
                    int windowV = (int)Math.floor(Math.random() * T);
                    v = Math.abs(windowV);
                    v = v + s;

                    boolean r = rb.nextBoolean();
                    senti = (int)Math.floor(Math.random() * (S) );
                    senti = Math.abs(senti);
                    if(senti==2){System.out.print(senti+" ");senti=0;}
                    if(data.docs[m].words[n] < data.posWordNumber) senti  = positiveSenti;
                    else  if(data.docs[m].words[n] < data.negWordNumber) senti = 1;

                    rv[m].add(r);
                    wv[m].add(v);
                    lv[m].add(senti);
                    if(senti == 0)
                        R =   (int)Math.floor(Math.random() * 3 );
                    else      R =  2+ (int)Math.floor(Math.random() * 3 );


                    data.docs[m].nds[s]++;
                    data.docs[m].ndsv[s][v] ++;
                    data.docs[m].ndv[v] ++;
                    data.docs[m].ndvl [senti][v] += 1;
                    ndl[m][senti] ++;

                    if(r==true){
                        int topic = (int)Math.abs(Math.floor(Math.random() * glK));
                        z[m].add(topic);
                        nwlGl[data.docs[m].words[n]][senti][topic] += 1;
                        nwsumlGl[senti][topic] += 1;
                        ndlGl[m][senti][topic] += 1; //ngld
                        ndlGlsum[m][senti] += 1; //ngld
                        ndsumGl[m]++;
                        data.docs[m].ndvGl[v] ++;
                        data.docs[m].ndvlGl [senti][v] ++;
                    }
                    else{
                        int loctopic= (int)Math.abs(Math.floor(Math.random() * locK));
                        loctopic = Math.abs(loctopic);
                        z[m].add(loctopic);
                        nwlLoc[data.docs[m].words[n]][senti][loctopic] += 1;
                        nwsumlLoc[senti][loctopic] += 1;
                        ndlLoc[m][senti][loctopic] += 1;
                        ndlLocsum[m][senti] += 1;
                        ndsumLoc[m]++;
                        data.docs[m].ndvLoc[v] ++;
                        data.docs[m].ndvlLoc [senti][v] ++;
                        data.docs[m].ndvlLock[senti][v][loctopic] ++;
                        data.docs[m].ndvLock[v][loctopic] ++;
                        ndLoc[m][loctopic] ++;
                    }
                }
                // total number of words in document i
                ndsum[m] = N;
            }
        }catch(Exception e){
            System.out.println("["+this.getClass()+".initiliazieWithRandomValues()]  " + e.getMessage());
            return;

        }
    }

    //---------------------------------------------------------------
    //	I/O Methods
    //---------------------------------------------------------------
    /**
     * read other file to get parameters
     */
    protected boolean readOthersFile(String otherFile){
        //open file <model>.others to read:

        try {
            BufferedReader reader = new BufferedReader(new FileReader(otherFile));
            String line;
            while((line = reader.readLine()) != null){
                StringTokenizer tknr = new StringTokenizer(line,"= \t\r\n");

                int count = tknr.countTokens();
//				if (count != 11)
//					continue;

                String optstr = tknr.nextToken();
                String optval = tknr.nextToken();

                if (optstr.equalsIgnoreCase("alphaGlobal")){
                    alphaGlobal = Double.parseDouble(optval);
                }
                else if (optstr.equalsIgnoreCase("betaGlobal")){
                    betaGlobal = Double.parseDouble(optval);
                }
                if (optstr.equalsIgnoreCase("alphaLocal")){
                    alphaLocal = Double.parseDouble(optval);
                }
                else if (optstr.equalsIgnoreCase("betaLocal")){
                    betaLocal = Double.parseDouble(optval);
                }
                else if (optstr.equalsIgnoreCase("gamma")){
                    gamma = Double.parseDouble(optval);
                }

                else if (optstr.equalsIgnoreCase("alphaMixGlobal")){
                    alphaMixGlobal = Double.parseDouble(optval);
                }
                else if (optstr.equalsIgnoreCase("alphaMixLocal")){
                    alphaMixLocal = Double.parseDouble(optval);
                }
//				else if (optstr.equalsIgnoreCase("ntopics")){
//					K = Integer.parseInt(optval);
//				}
                else if (optstr.equalsIgnoreCase("ngltopics")){
                    glK = Integer.parseInt(optval);
                }
                else if (optstr.equalsIgnoreCase("nloctopics")){
                    locK = Integer.parseInt(optval);
                }
                else if (optstr.equalsIgnoreCase("liter")){
                    liter = Integer.parseInt(optval);
                }
                else if (optstr.equalsIgnoreCase("nwords")){
                    V = Integer.parseInt(optval);
                }
                else if (optstr.equalsIgnoreCase("ndocs")){
                    M = Integer.parseInt(optval);
                }
                else {
                    // any more?
                }
            }

            reader.close();
        }
        catch (Exception e){
            System.out.println("Error while reading other file:" + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;
    }

    protected boolean readTAssignFile(String tassignFile){
        try {
            int i,j, sentenceNumber=0;
            BufferedReader reader = new BufferedReader(new InputStreamReader(
                    new FileInputStream(tassignFile), "UTF-8"));

            String line;
            z = new Vector[M];
            rv = new Vector[M];
            wv = new Vector[M];
            lv = new Vector[M];
            data = new Dataset(M);
            data.V = V;
            Vector<Integer> indexofLine =new Vector<Integer>();
            for (i = 0; i < M; i++){
                line = reader.readLine();
                StringTokenizer tknr1 = new StringTokenizer(line, " \t\r\n");
                while(tknr1.hasMoreTokens()){
                    indexofLine.add( Integer.parseInt(tknr1.nextToken()));
                }

                line = reader.readLine();
                StringTokenizer tknr = new StringTokenizer(line, " \t\r\n");

                int length = tknr.countTokens();

                Vector<Integer> words = new Vector<Integer>();
                Vector<Integer> topics = new Vector<Integer>();
                Vector<Boolean> topicType = new Vector<Boolean>();
                Vector<Integer> windowType = new Vector<Integer>();
                Vector<Integer> sentiType = new Vector<Integer>();

                for (j = 0; j < length; j++){
                    String token = tknr.nextToken();

                    StringTokenizer tknr2 = new StringTokenizer(token, ":");
                    if (tknr2.countTokens() != 5){
                        System.out.println("Invalid word-topic assignment line\n");
                        return false;
                    }

                    words.add(Integer.parseInt(tknr2.nextToken()));
                    topics.add(Integer.parseInt(tknr2.nextToken()));
                    topicType.add(Boolean.parseBoolean(tknr2.nextToken()));
                    windowType.add(Integer.parseInt(tknr2.nextToken()));
                    sentiType.add(Integer.parseInt(tknr2.nextToken()));

                }//end for each topic assignment

                //allocate and add new document to the corpus
                Document doc = new Document(words, indexofLine );
                data.setDoc(doc, i);

                //assign values for z
                z[i] = new Vector<Integer>();
                rv[i] = new Vector<Boolean>();
                wv[i] = new Vector<Integer>();
                lv[i] = new Vector<Integer>();
//                                sen[i] = new Vector<Integer>();
                for (j = 0; j < topics.size(); j++){
                    z[i].add(topics.get(j));
                }
                for (j = 0; j < topicType.size(); j++){
                    rv[i].add(topicType.get(j));
                }
                for (j = 0; j < windowType.size(); j++){
                    wv[i].add(windowType.get(j));
                }
                for (j = 0; j < sentiType.size(); j++){
                    lv[i].add(sentiType.get(j));
                }
//                                for (j = 0; j < sentiType.size(); j++){
//					sen[i].add(sentence.get(j));
//				}

                indexofLine.clear();
            }//end for each doc


            reader.close();
        }
        catch (Exception e){
            System.out.println("Error while loading model: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;
    }

    /**
     * load saved model
     */
    public boolean loadModel(){
        if (!readOthersFile(dir + File.separator + modelName + othersSuffix))
            return false;

        if (!readTAssignFile(dir + File.separator + modelName + tassignSuffix))
            return false;

        // read dictionary
        Dictionary dict = new Dictionary();
        if (!dict.readWordMap(dir + File.separator + wordMapFile))
            return false;

        data.localDict = dict;

        return true;
    }

    /**
     * Save word-topic assignments for this model
     */

    public boolean saveModelTAssign(String filename){
        int i, j;

        try{
            BufferedWriter writer = new BufferedWriter(new FileWriter(filename));

            //write docs with topic assignments for words
            for (i = 0; i < data.M; i++){
                for(Integer k: data.docs[i].indexofLine)
                    writer.write(k+" ");
                writer.write("\n");

                for (j = 0; j < data.docs[i].length; ++j){
                    writer.write(data.docs[i].words[j] + ":" + z[i].get(j) + ":"+ rv[i].get(j).toString()+":"+ wv[i].get(j)+":"+lv[i].get(j) +" ");
                }
                writer.write("\n");
            }
            writer.close();
        }
        catch (Exception e){
            System.out.println("Error while saving model tassign: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;
    }

    /**
     * Save model
     */
    public boolean saveModel(String modelName) {

        if (!saveModelTAssign(dir + File.separator + modelName + tassignSuffix)){
            return false;
        }

        String filename = dir + File.separator + modelName + othersSuffix;
        if (!saveModelOthers(filename)){
            return false;
        }

        if (twords > 0){
            if (!saveModelTwords(dir + File.separator + modelName + twordsSuffix,  V ))
                return false;
        }
//       if (toptopics > 0){
//            if (!saveTopdocs(dir + File.separator + modelName + tdocsSuffix))
//               return false;
//        }

        try{

//                        BinIO.storeDoubles( pi, dir + File.separator + modelName + piSuffix);
//                        BinIO.storeDoubles( winpi, dir + File.separator + modelName + winpiSuffix);
//                        BinIO.storeDoubles( this.thetal, dir + File.separator + modelName + thetaSuffix);
//                        BinIO.storeDoubles( this.loctheta, dir + File.separator + modelName + thetaSuffix);
//
//                        BinIO.storeDoubles( locphi[positiveSenti], dir + File.separator + "pos" + phiSuffix);
//                        BinIO.storeDoubles( locphi[negativeSenti], dir + File.separator + "neg" + phiSuffix);

        }catch (Exception e){
            System.out.println("Error while saving word-topic distribution:" + e.getMessage());
            e.printStackTrace();
            return false;
        }


//                if (!saveModelLocRating(dir + File.separator + modelName + piSuffixRating, locRating, M, locK)){
//			return false;
//		}
        return true;
    }

    /**
     * Save theta (topic distribution) for this model
     */

    /**
     * Save word-topic distribution
     */

    public boolean saveModelLocRating(String outputFilePath, double [][][] phi, int numOfDoc, int numOfColumn){
        try {
            PrintWriter out = new PrintWriter(new FileWriter(new File(outputFilePath)));
            BufferedReader reader = new BufferedReader(new FileReader("ratingTrip.txt"));
//                 BufferedReader reader = new BufferedReader(new FileReader("ratingN.txt"));
            String s="i";
            for(int m = 0; m < numOfDoc ; m++){
                for(int col = 0; col < numOfColumn ; col++){
                    //   for(int l = 0; l < 2 ; l++)
                    if(phi[m][0][col] > phi[m][1][col])
                        s = "pos";
                    else s = "neg";
                    if(col == 0) out.print(s);
                    else out.print(","+s);
                }
                out.println();
            }

            out.close();
        }
        catch (Exception e){
            System.out.println("Error while saving word-topic distribution:" + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;
    }
    /**
     * Save other information of this model
     */
    public boolean saveModelOthers(String filename){
        try{
            BufferedWriter writer = new BufferedWriter(new FileWriter(filename));

            writer.write("alphaGlobal=" + alphaGlobal + "\n");
            writer.write("betaGlobal=" + betaGlobal + "\n");
            writer.write("alphaLocal=" + alphaLocal + "\n");
            writer.write("betaLocal=" + betaLocal + "\n");
            writer.write("alphaMixGlobal=" + alphaMixGlobal + "\n");
            writer.write("alphaMixLocal=" + alphaMixLocal + "\n");
            writer.write("gamma=" + gamma + "\n");
            writer.write("delta=" + delta + "\n");
            writer.write("nsenti=" + S + "\n");
            writer.write("ngltopics=" + glK + "\n");
            writer.write("nloctopics=" + locK + "\n");
            writer.write("ndocs=" + M + "\n");
            writer.write("nwords=" + V + "\n");
            writer.write("liters=" + liter + "\n");
            writer.write("perplexity=" + perplexity + "\n");

            writer.close();
        }
        catch(Exception e){
            System.out.println("Error while saving model others:" + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;
    }

    /**
     * Save model the most likely words for each topic
     */

    int ak =0 ;
    public boolean  sortTwords(double [][][] phi, int K, int V, int l){
        try{

            for (int k = 0; k < K; k++){
                List<Pair> wordsProbsList = new ArrayList<Pair>();
                int count =0;
                for (int w = 0; w < V; w++){
                    {
                        Pair p = new Pair(w, phi[l][k][w], false);
                        wordsProbsList.add(p);
                        count++;
                    }
                }
                Collections.sort(wordsProbsList);
                for (int i = 0; i < twords; i++){
                    if (data.localDict.contains((Integer)wordsProbsList.get(i).first)){
                        aspects[ak][i] = data.localDict.getWord((Integer)wordsProbsList.get(i).first );
                        aspectsProb[ak][i] = (Double) wordsProbsList.get(i).second;
                    }

                }
                ak++;
            } //end foreach topic



        }
        catch(Exception e){
            System.out.println("Error while saving model twords: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;


    }


    public List<Integer> globalRank(double[] countTopTopic, String filename) throws IOException{

        List<Integer> rank = new ArrayList<Integer>();
        PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(filename.replace("JMTS", "JMTS-global"))));
        List<Pair> wordsProbsList = new ArrayList<Pair>();

        for (int k = 0; k < glK + glK; k++)
        {
            Pair p = new Pair(k, countTopTopic[k], false);
            wordsProbsList.add(p);
        }//end foreach topic
        Collections.sort(wordsProbsList);
        //print topic
        //for (int i = 0; i < K; i++){
        for (int i = 0; i < glK + glK; i++){
            pw.write(wordsProbsList.get(i).first+" "+wordsProbsList.get(i).second);
            pw.write("\n");
            rank.add( (Integer) wordsProbsList.get(i).first  );
        }
        pw.close();
        return rank;

    }

    public List<Integer> localRank(double[] countTopTopic, String filename) throws IOException{

        List<Integer> rank = new ArrayList<Integer>();
        PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(filename.replace("JMTS", "JMTS-local"))));
        List<Pair> wordsProbsList = new ArrayList<Pair>();

        for (int k = glK + glK; k < glK + glK + locK + locK; k++)
        {
            Pair p = new Pair(k, countTopTopic[k], false);
            wordsProbsList.add(p);
        }//end foreach topic
        Collections.sort(wordsProbsList);
        //print topic
        for (int i = 0; i < locK + locK; i++){
            pw.write(wordsProbsList.get(i).first+" "+wordsProbsList.get(i).second);
            pw.write("\n");
            rank.add( (Integer) wordsProbsList.get(i).first  );
        }
        pw.close();
        return rank;

    }

    public void writetop10topics(List<Integer> rank, String filename)throws IOException{

        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename), "UTF-8"));
        int toptopic = 10;
        for (int k = 0; k < toptopic; k++){
            writer.write("Topic " + rank.get(k) + ":\n");
            for (int i = 0; i < twords -10 ; i++)
                writer.write("\t" + aspects[rank.get(k)][i] + " " + aspectsProb[rank.get(k)][i] + "\n");
        }
        writer.close();

    }
    public void writetop10topicsCSV(List<Integer> rank, String filename)throws IOException{

        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename), "UTF-8"));
        int toptopic = 10;
        for (int k = 0; k < toptopic; k++)
            if(k == 0)  writer.write("T"+rank.get( k  ));
            else    writer.write(",T"+rank.get( k  ));
        writer.write("\n");

        for(int i = 0; i < twords - 10 ; i++){
            for (int k = 0; k < toptopic; k++)
                if(k == 0)  writer.write( aspects[rank.get(k)][i] );
                else     writer.write(","+ aspects[rank.get(k)][i] );
            writer.write("\n");
        }
        writer.close();

    }
    public void TopTopicsInDocs(double[] countTopTopic, String filename, int K) throws IOException{

        List<Integer> rank  = globalRank(countTopTopic, filename);
        String fileTop10 = filename.replace(".rankedTopics", ".top10topics");
        String fileTop10csv = filename.replace(".rankedTopics", "-top10.csv");

        writetop10topics(rank, fileTop10.replace("JMTS", "JMTS-global"));
        writetop10topicsCSV(rank, fileTop10csv.replace("JMTS", "JMTS-global"));

        rank  = localRank(countTopTopic, filename);
        writetop10topics(rank, fileTop10.replace("JMTS", "JMTS-local"));
        writetop10topicsCSV(rank, fileTop10csv.replace("JMTS", "JMTS-local"));


    }
    public void TopTopicsInDocsLocal(int[] countTopTopic, String filename, int K) throws IOException{
//
//               // globalRank(countTopTopic, filename, K-this.locK-locK);
//                globalRank(countTopTopic, filename+"-local", K);
//
//                BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename.replace(".rankedTopics-local", ".top10topics-local")), "UTF-8"));
//
//                for (int k = 0; k < 10; k++){
//                writer.write("Topic " + rank.get(k) + ":\n");
//		for (int i = 0; i < twords -10 ; i++)
//                    writer.write("\t" + aspects[rank.get(k)][i] + " " + aspectsProb[rank.get(k)][i] + "\n");
//                }
//                 writer.close();
//
//                writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename.replace(".rankedTopics-local", "-top10-local.csv")), "UTF-8"));
//
//                for (int k = 0; k < 10; k++)
//                    if(k == 0)  writer.write("T"+rank.get( k  ));
//                    else    writer.write(",T"+rank.get( k  ));
//               writer.write("\n");
//
//               for(int i = 0; i < twords - 10 ; i++){
//                    for (int k = 0; k < 10; k++)
//                        if(k == 0)  writer.write( aspects[rank.get(k)][i] );
//                        else     writer.write(","+ aspects[rank.get(k)][i] );
//                    writer.write("\n");
//                }
//                writer.close();
//
//               writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename.replace(".rankedTopics-local", "-top20-local.csv")), "UTF-8"));
//
//               for (int k = 80; k < 100; k++)
//                    if(k == 80)  writer.write("T"+k);
//                    else    writer.write(",T"+k);
//               writer.write("\n");
//
//               for(int i = 0; i < twords -10 ; i++){
//                    for (int k = 80; k < 100; k++)
//                        if(k == 80)  writer.write( aspects[k][i] );
//                        else     writer.write(","+ aspects[k][i] );
//                    writer.write("\n");
//                }
//                writer.close();
//
//
//                writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename.replace(".rankedTopics-local", ".top10trends-local")), "UTF-8"));
//                Trends trends = new Trends(filename.replace(".rankedTopics-local", ".twords"));
//                List<String> listKeywords = trends.getTop10Trends();
//                if(listKeywords!=null)
//                for(String keywords : listKeywords){
//                   writer.write(keywords+"\n");
//                }
//                writer.close();
//
//                TopicAnalyzer ta = new TopicAnalyzer();
//                List<String> lst = ta.getTrendingTopics(filename.replace(".rankedTopics-local", ".twords"));
//                //System.out.println(filename);
//                writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename.replace(".rankedTopics-local", ".analyzer-local")), "UTF-8"));
//                if(lst != null)
//                for(String s:lst){
//                    writer.write(s+"\n");
//                }
//                writer.close();
//
    }
    public boolean saveTopdocs(String filename){
        try{
//                        PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(filename)));

            int K = glK+glK+locK+locK;
            if (toptopics > K){
                toptopics = K;
            }
            int [] countTopTopic = new int[K];
            double [] countTopTopicMass = new double[K];

            for (int k = 0; k < K; k++) countTopTopic[k] = 0;
            for (int k = 0; k < K; k++) countTopTopicMass[k] = 0.0;

            for (int m = 0; m < M; m++){
                List<Pair> wordsProbsList = new ArrayList<Pair>();
                for (int k = 0; k < K; k++)
                {

                    Pair p = new Pair(k, theta[m][k], false);

                    wordsProbsList.add(p);
                }//end foreach topic

                Collections.sort(wordsProbsList);
                //print topic
//				pw.print("Doc " + m + ": ");
                for (int i = 0; i < toptopics; i++){
//                                    pw.printf("(%d, %f) ",wordsProbsList.get(i).first, wordsProbsList.get(i).second);
                    int a = (Integer)wordsProbsList.get(i).first;
                    double d = (Double)wordsProbsList.get(i).second;
                    countTopTopic[a]++;
                    countTopTopicMass[a] +=d ;
                }
//                                pw.print("\n");
            } //end foreach doc
//			pw.close();
            TopTopicsInDocs(countTopTopicMass, filename.replace(".tdocs", ".rankedTopics"), K);
            //TopTopicsInDocs(countTopTopic, filename.replace(".tdocs", ".rankedTopics"), K);
//                        TopTopicsInDocs(countTopTopicMass, filename.replace(".tdocs", ".topicMassRank"));



        }
        catch(Exception e){
            System.out.println("Error while saving model top docs: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;
    }


    public void wrtieCSV(String filename, int K)throws Exception {
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename.replace(".twords", ".csv")), "UTF-8"));

        for (int k = 0; k < K; k++)
            if(k == 0)  writer.write("T"+k);
            else    writer.write(",T"+k);
        writer.write("\n");
        for(int i = 0; i < twords - 10; i++){
            for (int k = 0; k < K; k++)
                if(k == 0)  writer.write( aspects[k][i] );
                else     writer.write(","+ aspects[k][i] );
            writer.write("\n");
        }
        writer.close();
    }
    public void wrtieDistribution(String filename, int K)throws Exception {
        BufferedWriter  writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename), "UTF-8"));
        for (int k = 0; k < K; k++){
            writer.write("Topic " + k + ":\n");
            for (int i = 0; i < twords ; i++)
                writer.write("\t" + aspects[k][i] + " " + aspectsProb[k][i] + "\n");
        }
        writer.close();
        String s = ".jmts"+ Integer.toString(glK)+".topics.txt";
        writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename.replace(".twords", s)), "UTF-8"));
        for (int k = glK+glK; k < K; k++){
//                writer.write("Topic " + k + ":");
            for (int i = 0; i < twords ; i++)
                writer.write(aspects[k][i] + " " );
            writer.write("\n");
        }
        writer.close();
    }
    public boolean saveModelTwords(String filename,  int V){
        try{
            if (twords > V){
                twords = V;
            }
            int K = glK+glK+locK+locK;
            aspects = new String[K][twords];
            aspectsProb = new double[K][twords];
            sortTwords(glphi,   glK, V, 0);
            sortTwords(glphi,   glK, V, 1);
            sortTwords(locphi,   locK, V, 0);
            sortTwords(locphi,   locK, V, 1);

            wrtieDistribution(filename, K);
            filename = filename.replace(".twords", ".csv");
            wrtieCSV(filename, K);
        }
        catch(Exception e){
            System.out.println("Error while saving model twords: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
        return true;
    }


    //---------------------------------------------------------------
    //	Init Methods
    //---------------------------------------------------------------
    /**
     * initialize the model
     */
    protected boolean initializeFrom(JMTSCmdOption option){
        if (option == null)
            return false;

        modelName = option.modelName;
//		K = option.K;
        locK = option.locK;
        glK = option.glK;

        alphaGlobal = option.alphaGlobal;
        if (alphaGlobal < 0.0)
            alphaGlobal = 15.0 / glK;

        alphaLocal = option.alphaLocal;
        if (alphaLocal < 0.0)
            alphaLocal = 15.0 / locK;


        if (option.betaGlobal > 0.0)
            betaGlobal = option.betaGlobal;

        if (option.betaLocal > 0.0)
            betaLocal =option.betaLocal;


        if (option.alphamixgl > 0.0)
            alphaMixGlobal = option.alphamixgl;

        if (option.alphaMixLocal > 0.0)
            alphaMixLocal =option.alphaMixLocal;


        if (option.gamma >= 0)
            gamma = option.gamma;

        if (option.delta >= 0)
            delta = option.delta;

        if (option.adeltaNeg >= 0)
            adeltaNeg = option.adeltaNeg;

        niters = option.niters;

        dir = option.dir;
//		if (dir.endsWith(File.separator))dir = dir.substring(0, dir.length() - 1);

        dfile = option.dfile;
        twords = option.twords;
        wordMapFile = option.wordMapFileName;

        return true;
    }
    /**
     * Init parameters for estimation
     */
    public String getdirName(String filename){
        filename = new File(dfile).getName();
        String [] split= filename.split("\\.");
        filename = split[0];
        File ndir = new File(dir+"-global/"+filename);
        if(!ndir.exists()) ndir.mkdir();
        ndir = new File(dir+"-local/"+filename);
        if(!ndir.exists()) ndir.mkdir();

        return split[0];

    }
    public boolean initNewModel(JMTSCmdOption option){
        if (!initializeFrom(option))
            return false;

        int m, n, w, k;
//		p = new double[K];

        data = Dataset.readDataSet(dir + File.separator + dfile);
//    		data = Dataset.readDataSet(dfile);

        if (data == null){
            System.out.println("Fail to read training data!\n");
            return false;
        }

        //+ allocate memory and assign values for variables
        M = data.M;
        V = data.V;
        dir = option.dir;


        //               dir = option.dir +  File.separator + getdirName(dfile);


        savestep = option.savestep;

        // K: from command line or default value
        // alpha, beta: from command line or default values
        // niters, savestep: from command line or default values
        allocateMemoryIntializeZero();
        initiliazieWithRandomValues();



        return true;
    }

    /**
     * Init parameters for inference
     * @param newData DataSet for which we do inference
     */
    public boolean initNewModel(JMTSCmdOption option, Dataset newData, Model trnModel){
        if (!initializeFrom(option))
            return false;

        int m, n, w, k;

//		K = trnModel.K;
        locK = option.locK;
        glK = option.glK;

        alphaGlobal = trnModel.alphaGlobal;
        alphaLocal = trnModel.alphaLocal;
        betaGlobal = trnModel.betaGlobal;
        betaLocal =trnModel.betaLocal;
        gamma = trnModel.gamma;
        delta = trnModel.delta;


//		p = new double[K];
//		System.out.println("K:" + K);

        data = newData;

        //+ allocate memory and assign values for variables
        M = data.M;
        V = data.V;
        dir = option.dir;

//                dir = option.dir +  File.separator  + getdirName(dfile);


        savestep = option.savestep;
        System.out.println("M:" + M);
        System.out.println("V:" + V);

        // K: from command line or default value
        // alpha, beta: from command line or default values
        // niters, savestep: from command line or default values

        data.readParadigmAspectList("aspectsdic.txt");
        this.allocateMemoryIntializeZero();
        this.initiliazieWithRandomValues();;

        //theta = new double[M][K];
        //phi = new double[K][V];

        return true;
    }

    /**
     * Init parameters for inference
     * reading new dataset from file
     */
    public boolean initNewModel(JMTSCmdOption option, Model trnModel){


        System.out.println("Initializing new Model for new docs...");
        if (!initializeFrom(option))
            return false;

        Dataset dataset = Dataset.readDataSet(dir + File.separator + dfile, trnModel.data.localDict);
        if (dataset == null){
            System.out.println("Fail to read dataset!\n");
            return false;
        }

        return initNewModel(option, dataset , trnModel);
    }

    /**
     * init parameter for continue estimating or for later inference
     * Continue to estimate the model from a previously estimated model.
     */
    public boolean initEstimatedModel(JMTSCmdOption option){
        if (!initializeFrom(option))
            return false;

        int m, n, w, k;

//		p = new double[K];

        // load model, i.e., read z and trndata
        System.out.println("Loading train Model...");
        if (!loadModel()){
            System.out.println("Fail to load word-topic assignment file of the model!\n");
            return false;
        }


        System.out.println("\talpha local:" + this.alphaGlobal);
        System.out.println("\tbeta Global:" + this.betaGlobal);
        System.out.println("\talpha local:" + this.alphaLocal);
        System.out.println("\tbeta local:" + this.betaLocal);
        System.out.println("\tgamma:" + gamma);


        System.out.println("\tglobal topic:" + glK);
        System.out.println("\tlocal topic:" + locK);

        System.out.println("\tM:" + M);
        System.out.println("\tV:" + V);

        this.allocateMemoryIntializeZero();
        this.initiliazieWithEstimatedValues();


        dir = option.dir;

        savestep = option.savestep;

        System.out.println("done.");
        return true;
    }
    public void initiliazieWithEstimatedValues()  {

        try{
            int m, n, w, k, s, v, senti, R;
            for (m = 0; m < data.M; m++){
                int N = data.docs[m].length;

                //initilize for z

                Random rb = new Random();
                for (n = 0; n < N; n++){
                    s =    data.docs[m].sentence[n];
                    //    System.out.println(s);

                    boolean r = rv[m].get(n);
                    v = wv[m].get(n);
                    senti = lv[m].get(n);
                    if(senti == 0)
                        R =   (int)Math.floor(Math.random() * 3 );
                    else      R =  2+ (int)Math.floor(Math.random() * 3 );

                    data.docs[m].nds[s]++;
                    data.docs[m].ndsv[s][v] ++;
                    data.docs[m].ndv[v] ++;
                    data.docs[m].ndvl [senti][v] += 1;
                    ndl[m][senti] ++;

                    if(r==true){
                        int topic = z[m].get(n);
                        nwlGl[data.docs[m].words[n]][senti][topic] += 1;
                        nwsumlGl[senti][topic] += 1;
                        ndlGl[m][senti][topic] += 1; //ngld
                        ndlGlsum[m][senti] += 1; //ngld
                        ndsumGl[m]++;
                        data.docs[m].ndvGl[v] ++;
                        data.docs[m].ndvlGl [senti][v] ++;
                    }
                    else{
                        int loctopic = z[m].get(n);

                        nwlLoc[data.docs[m].words[n]][senti][loctopic] += 1;
                        nwsumlLoc[senti][loctopic] += 1;
                        ndlLoc[m][senti][loctopic] += 1;
                        ndlLocsum[m][senti] += 1;
                        ndsumLoc[m]++;
                        data.docs[m].ndvLoc[v] ++;
                        data.docs[m].ndvlLoc [senti][v] ++;
                        data.docs[m].ndvlLock[senti][v][loctopic] ++;
                        data.docs[m].ndvLock[v][loctopic] ++;
                        ndLoc[m][loctopic] ++;

                    }
                }
                // total number of words in document i
                ndsum[m] = N;

            }

        }catch(Exception e){
            System.out.println("["+this.getClass()+".initiliazieWithRandomValues()]  " + e.getMessage());
            return;

        }
    }

   public void computePhiLoc(){
            for (int senti = 0; senti < S; senti++)
                for (int k = 0; k < locK; k++){
                    for (int w = 0; w < V; w++){
                        locphi[senti][k][w] = ( nwlLoc[w][senti][k] +  betaLocal) / (nwsumlLoc [senti][k]  + V * betaLocal);
                    }
                }
   }

   public void computePhiGl(){
        for (int senti = 0; senti < S; senti++)
            for (int k = 0; k < glK; k++){
                for (int w = 0; w < V; w++){
                    glphi [ senti ][k][ w ] = (nwlGl [w][senti][k] + betaGlobal) / (nwsumlGl [senti][k] + V * betaGlobal);
                }
            }

   }

   public void computeThetaGlobal(){
        gltheta = new double[M][S][glK];
        for (int l = 0; l < 2; l++){
            for (int m = 0; m < M; m++){
                for (int k = 0; k < glK; k++){
                    gltheta[m][l][k] = (ndlGl[m][l][k] + alphaGlobal) / (ndlGlsum[m][l] + glK * alphaGlobal);
                }
            }
        }
   }

   public void computeThetaLocal(){
        loctheta = new double[M][S][locK];
        for (int l = 0; l < 2; l++){
            for (int m = 0; m < M; m++){
                for ( int k=0;k < locK; k++){
                    loctheta[m][l][k] = (ndlLoc[m][l][k] + alphaLocal) / (ndlLocsum[m][l] + locK * alphaLocal);
                }
            }
        }
        //saveAspectThetaSentiDist();

   }

   public void saveAspectThetaSentiDist(){
        double [][] aspectProb = new double[M][locK + locK];

        int offset = 0;
        for (int l = 0; l < 2; l++){
            for (int m = 0; m < M; m++){
                for ( int k=0;k < locK; k++){
                    if (l ==1) offset = locK;
                    else offset = 0;
                    aspectProb[m][offset+k]  = loctheta[m][l][k];
                }
            }
        }
        try {
            BinIO.storeDoubles(aspectProb, dir+"/aspectThetaSentiDist");
        } catch (IOException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }
   }

    public void computeTheta(){
                    /*
        trnModel.theta = new double[trnModel.M][trnModel.glK+trnModel.glK+trnModel.locK+trnModel.locK];
        int K=0;
        for (int l = 0; l < 2; l++){
            if(l==1)K += trnModel.glK;
            for (int m = 0; m < trnModel.M; m++){
                for (int k = 0; k < trnModel.glK; k++){
                    trnModel.theta[m][K+k] = (trnModel.ndlGl[m][l][k] + trnModel.alphaGlobal) / (trnModel.ndlGlsum[m][l] + trnModel.glK * trnModel.alphaGlobal);
                }
            }
        }
        K += trnModel.glK;
        for (int  l = 0; l < 2; l++){
            if(l==1)K += trnModel.locK;
            for (int m = 0; m < trnModel.M; m++){
                for (int k = 0; k < trnModel.locK; k++){
                    trnModel.theta[m][K + k] = (trnModel.ndlLoc[m][l][k] + trnModel.alphaLocal) / (trnModel.ndlLocsum[m][l] + trnModel.locK * trnModel.alphaLocal);
                }
            }
        }
              */
    }

}
