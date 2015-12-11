
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


import java.util.ArrayList;
import java.util.Vector;

public class Document {

    //----------------------------------------------------
    //Instance Variables
    //----------------------------------------------------

    public int [] words;
    public int [] sentence;
    public int [][] ndsv;
    public int [] ndv;

    public int [] nds;
    public int [][] ndvlGl;
    public int [][] ndvl;
    public int [][][] ndvlR;
    public int [][] ndvlLoc;
    public int [] ndvGl;
    public int [] ndvLoc;
    public int [][][] ndvlLock;
    public int [][] ndvLock;
    public String rawStr;
    public int length;
    public int numberoflines;
    public int window;
    public String rate;
    public String  author;
    public String originalStr;
    public Vector<Integer> indexofLine = new Vector<Integer>();
    ArrayList<String> lines;

    //----------------------------------------------------
    //Constructors
    //----------------------------------------------------



    public Document(Vector<Integer> doc, String rawStr,  Vector<Integer> indexofLine, String rate, String author, String originalStr,  ArrayList<String> lines){

        this.length = doc.size();
        this.rawStr = rawStr;
        this.words = new int[length];
        this.sentence = new int[length];
        this.rate = rate;
        this.author = author;
        this.originalStr = originalStr;
        this.words = new int[length];
        this.sentence = new int[length];
        this.numberoflines=indexofLine.size();
        this.window = this.numberoflines+2;
        this.nds = new int[this.numberoflines];
        this.ndv = new int[this.window];
        this.ndsv = new int[this.numberoflines][this.window];
        this.ndvlGl = new int[3][this.window];
        this.ndvl = new int[3][this.window];
        this.ndvlLoc = new int[3][this.window];
        this.ndvGl = new int[this.window];
        this.ndvLoc = new int[this.window];
        this.ndvlLock = new int[3][this.window][30];
        this.ndvLock = new int[this.window][30];
        this.ndvlR = new int[3][this.window][30];
        for (Integer kk:indexofLine)
            this.indexofLine.add(kk);

        int t =0;
        for (int j = 0; j < numberoflines; ++j){
            for (int k = 0; k < indexofLine.get(j); ++k)
                if(t < length){
                    this.words[t] = doc.get(t);
                    this.sentence[t] = j;
                    t++;
                }
        }
        this.lines = lines;
    }
    public Document(Vector<Integer> doc, Vector<Integer> indexofLine){

        this.length = doc.size();
        this.words = new int[length];
        this.sentence = new int[length];
        this.words = new int[length];
        this.sentence = new int[length];
        this.numberoflines=indexofLine.size();
        this.window = this.numberoflines+2;
        this.nds = new int[this.numberoflines];
        this.ndv = new int[this.window];
        this.ndsv = new int[this.numberoflines][this.window];
        this.ndvlGl = new int[3][this.window];
        this.ndvl = new int[3][this.window];
        this.ndvlLoc = new int[3][this.window];
        this.ndvGl = new int[this.window];
        this.ndvLoc = new int[this.window];
        this.ndvlLock = new int[3][this.window][30];
        this.ndvLock = new int[this.window][30];
        this.ndvlR = new int[3][this.window][30];
        for (Integer kk:indexofLine)
            this.indexofLine.add(kk);

        int t =0;
        for (int j = 0; j < numberoflines; ++j){
            for (int k = 0; k < indexofLine.get(j); ++k)
                if(t < length){
                    this.words[t] = doc.get(t);
                    this.sentence[t] = j;
                    t++;
                }
        }
    }

}
