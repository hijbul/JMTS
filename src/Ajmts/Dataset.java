
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

import util.Dictionary;
import util.Porter;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

public class Dataset {
	//---------------------------------------------------------------
	// Instance Variables
	//---------------------------------------------------------------
	
	public Dictionary localDict;			// local dictionary	
	public Document[] docs; 		// a list of documents
	public int M; 			 		// number of documents
	public int V;			 		// number of words
	public int posWordNumber;			 		// number of words
	public int negWordNumber;			 		// number of words
    public boolean [] aspectVocab;
    public  Map<String, Integer> aspectWord2int;
       
    public static Map<String, Integer> stopWord2int;
	// map from local coordinates (id) to global ones 
	// null if the global dictionary is not set
	public Map<Integer, Integer> lid2gid; 
	
	//link to a global dictionary (optional), null for train data, not null for test data
	public Dictionary globalDict;	 		
	

	
	public Dataset(int M){
		localDict = new Dictionary();
		this.M = M;
		this.V = 0;
		docs = new Document[M];
		globalDict = null;
		lid2gid = null;
	}
	
	public Dataset(int M, Dictionary globalDict){
		localDict = new Dictionary();	
		this.M = M;
		this.V = 0;
		docs = new Document[M];
		this.globalDict = globalDict;
		lid2gid = new HashMap<Integer, Integer>();
	}
	
	//-------------------------------------------------------------
	//Public Instance Methods
	//-------------------------------------------------------------
	/**
	 * set the document at the index idx if idx is greater than 0 and less than M
	 * @param doc document to be set
	 * @param idx index in the document array
	 */	
	public void setDoc(Document doc, int idx){
		if (0 <= idx && idx < M){
			docs[idx] = doc;
		}
	}
	/**
	 * set the document at the index idx if idx is greater than 0 and less than M
	 * @param str string contains doc
	 * @param idx index in the document array
	 */
	public void setDoc(String str, int idx,  Vector<Integer> index, String rate, String author, String originalStr,  ArrayList<String> lines){
             
		if (0 <= idx && idx < M){
			String [] words = str.split("[ \\t\\n]");

			Vector<Integer> ids = new Vector<Integer>();

			for (String word : words){

                if (word.isEmpty())continue;
                if (word.equalsIgnoreCase("not_good"))
                    System.out.println(word);

				int _id = localDict.word2id.size();

				if (localDict.contains(word))
					_id = localDict.getID(word);

				if (globalDict != null){
					//get the global id
					Integer id = globalDict.getID(word);
					//System.out.println(id);

					if (id != null){
						localDict.addWord(word);

						lid2gid.put(_id, id);
						ids.add(_id);
					}
					else { //not in global dictionary
						//do nothing currently
					}
				}
				else {
					localDict.addWord(word);
					ids.add(_id);
				}
			//System.out.print(word+" -");
                        }
            originalStr = null;
            //System.out.println(index);
			Document doc = new Document(ids, str, index, rate,  author,  originalStr, lines);
			docs[idx] = doc;

			V = localDict.word2id.size();
		}
	}

	//---------------------------------------------------------------
	// I/O methods
	//---------------------------------------------------------------

	/**
	 *  read a dataset from a stream, create new dictionary
	 *  @return dataset if success and null otherwise
	 */

    public  void readParadigmList(String filename) throws IOException {

           BufferedReader reader = new BufferedReader(new InputStreamReader(
					new FileInputStream(filename), "UTF-8"));
           int f = 0; 
           int i = 0;
           int max = -1;
           while(reader.ready())
            {
                String line = reader.readLine();
                String[] str = line.split(" ");
               
                for(String s:str){
                    localDict.addWord(s);
                    i = localDict.getID(s);
                    if(i > max )max =i;
                }
                if (f == 0)posWordNumber = max;
                else if (f == 1) negWordNumber = max;
                f = 1;

            }
        }
        
                
                
                
        
    public  Map<String, Integer> readParadigmAspectList(String filename) {


        try{
                 
              BufferedReader reader = new BufferedReader (new InputStreamReader(
                                new FileInputStream(filename),"UTF-8"));

              int t=0;
              String line;
              aspectVocab = new boolean[V];
              aspectWord2int =  new HashMap<String, Integer>();
              Porter  strPorter = new Porter();
              while( (line = reader.readLine())!=null){

                    String[] str = line.split("\\s+");
                
                    for(String s:str){
                        s = strPorter.stripAffixes(s);
                        if(localDict.contains(s))
                            this.aspectVocab[localDict.getID(s)]=true;
                        if(!aspectWord2int.containsKey(s))
                                aspectWord2int.put(s, t);
                    }
                    t++;
              }
              reader.close();
              }
		catch (Exception e){
			System.out.println("Read Dataset Error: " + e.getMessage());
			e.printStackTrace();
		}
             return aspectWord2int;

    }

	public static Dataset readDataSet(String filename){
		try {
			BufferedReader reader = new BufferedReader(new InputStreamReader(
					new FileInputStream(filename), "UTF-8"));

			Dataset data = readDataSet(reader);
            File file = new File(filename);
            String dir = filename.substring(0,filename.indexOf(file.getName()));
			data.readParadigmAspectList(dir+"aspectsdic.txt");
            reader.close();
			return data;
		}
		catch (Exception e){
			System.out.println("Read Dataset Error: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
	
	/**
	 * read a dataset from a file with a preknown vocabulary
	 * @param filename file from which we read dataset
	 * @param dict the dictionary
	 * @return dataset if success and null otherwise
	 */
	public static Dataset readDataSet(String filename, Dictionary dict){
		try {
			BufferedReader reader = new BufferedReader(new InputStreamReader(
					new FileInputStream(filename), "UTF-8"));
			Dataset data = readDataSet(reader, dict);
			File file = new File(filename);
            String dir = filename.substring(0,filename.indexOf(file.getName()));
			//data.readParadigmAspectList(dir+"aspectsdic.txt");

			reader.close();
			return data;
		}
		catch (Exception e){
			System.out.println("Read Dataset Error: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
	
	/**
	 *  read a dataset from a stream, create new dictionary
	 *  @return dataset if success and null otherwise
	 */
         static int k=0;
         
//         public static Dataset readDataSet(BufferedReader reader){ // boolean trackline
//             
//         }
	

    public static Dataset readDataSet(BufferedReader reader, Dictionary dict){ // boolean trackline
		try {
                        //read number of document
			String line;
			line = reader.readLine();
			int M = Integer.parseInt(line);

            StringBuilder sb;
            StringBuilder originalStr;
			Dataset data = new Dataset(M, dict);
            data.readStopWords();
            data.readParadigmList("ppolar.txt");
            Vector<Integer>  index;
            ArrayList<String> lines = new ArrayList<String>();

            for (int i = 0; i < M; ++i){
                index = new Vector<Integer>();
                sb = new StringBuilder(50);
                originalStr = new StringBuilder(50);
                String author ="",rate="";
                                 
				while( (line = reader.readLine()) != null){
                    if (line.startsWith("<author")){
                    author = line;
                        continue;
                    }

                    if (line.startsWith("<content")){
                        continue;
                    }

                    if (line.startsWith("<rating")){
                        rate = line;
                        break;
                    }

                    originalStr.append(line).append("\n");
                    line = removeStopWordandStemming(line);
                    if(line.isEmpty())
                        continue;

                    sb.append(line).append(" ");
                    String [] str = line.split(" ");
                    index.add( str.length  );
                    lines.add(line);
                }
                data.setDoc(sb.toString(), i, index, rate, author, originalStr.toString(), lines);
                index.clear();

			}

			return data;
		}
		catch (Exception e){
			System.out.println("Read Dataset Error: " + e.getMessage());
			return null;
		}
	}
          
          
    public static Dataset readDataSet(BufferedReader reader){
		try {
			//read number of document
			String line;
			line = reader.readLine();
			int M = Integer.parseInt(line);
            StringBuilder sb;
            StringBuilder originalStr;

			Dataset data = new Dataset(M);
            data.readStopWords();
            data.readParadigmList("ppolar.txt");
            //data.readParadigmList("E:\\TKDE\\ppolar.txt");
            Vector<Integer>  index;

			for (int i = 0; i < M; ++i){
//
                index = new Vector<Integer>();
                sb = new StringBuilder(50);
                originalStr = new StringBuilder(50);
                ArrayList<String> lines = new ArrayList<String>();

                String author ="",rate="";

				while( (line = reader.readLine()) != null)
                {

                    String line1 = line;
                    line = line.toLowerCase();

                    if (line.startsWith("<author")){
                        author = line;
                        continue;
                    }

                    if (line.startsWith("<content")){
                        continue;
                    }

                    if (line.startsWith("<rating")){
                        rate = line;
                        break;
                    }


                    originalStr.append(line).append("\n");
                    line = removeStopWordandStemming(line);
                    if(line.isEmpty())continue;
                    sb.append(line).append(" ");
                    String [] str = line.split(" ");
                    index.add( str.length);
                    lines.add(line1);
                }

/* // single sentence reviews
                line = reader.readLine();
                System.out.println(line);
                String line1 = line;
                originalStr.append(line).append("\n");
                line = removeStopWordandStemming(line);
//                if(line.isEmpty())continue;
                sb.append(line).append(" ");
                String [] str = line.split(" ");
                index.add( str.length);
                lines.add(line1);
*/



                data.setDoc(sb.toString(), i, index, rate, author, originalStr.toString(),lines);
//              data.setDocPreDict(sb.toString(), i, index, rate, author, originalStr.toString());
                index.clear();
			}

			return data;
		}
		catch (Exception e){
			System.out.println("Read Dataset Error: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
        

    public  static String removeStopWordandStemming(String line){
        try {

            StringBuilder result;
            Porter strPorter = new Porter(); // stemming
            boolean first=false;

            result = new StringBuilder(line.length());
            String[] splitString = (line.split("\\s+|\\p{Punct}")); // remove punctuation
            first = false;
            for (String string : splitString) {

                if(first==true)  result.append(" ");
                first = false;
                String tmp = string.toLowerCase();
                if(tmp.contains("n\'t"))tmp = "not";
                if(!stopWord2int.containsKey(tmp)&&!tmp.isEmpty()) // remove stopwords
                {
                    tmp=strPorter.stripAffixes(tmp);              // stemmming words
                    if(!stopWord2int.containsKey(tmp)&&!tmp.isEmpty()){
                        result.append(tmp);
                        first=true;
                    }
                }
            }
            return result.toString();

        }
        catch (Exception e){
            System.out.println("Read Dataset Error: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }

    public  void readStopWords() {

             try{
              BufferedReader reader = new BufferedReader (new InputStreamReader(
                                new FileInputStream("stop_words.txt"),"UTF-8"));

              int t=0;
              String line;

              stopWord2int =  new HashMap<String, Integer>();
              while( (line = reader.readLine())!=null){

                    stopWord2int.put(line.toString(), t++);

              }
              reader.close();
            //   System.out.println(stopWord2int.size());
              }
		catch (Exception e){
			System.out.println("Read Dataset Error: " + e.getMessage());
			e.printStackTrace();
		}


    }
        


}
