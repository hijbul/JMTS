JMTS (Joint Multi-grain topic sentiment) in Java (based on JGibbLDA)
This is a Java implementation of JMTS based on the popular JGibbLDA package. The code has been heavily refactored for JMTS and a few additional options have been added. See sections below for more details.

Data Format

The input data format is similar to the JGibbLDA input data format, with some minor changes to provide sentences of reviews as input.

Number of documents
<author>
<content>
line 1
line 2
line 3
...
...
line n
<docend>

<author>
<content>
line 1
...
...
<docend>

where each document is a set of lines, and each line consists of a set of terms

line_i = term_1 term_2 ... term_n
Usage

Please see the JGibbLDA usage, noting the following changes:


New options have been added:

-alphaGlobal : Hyperparameter

-alphaLocal : Hyperparameter

-betaGlobal: Hyperparameter

-betaLocal:Hyperparameter


-gamma : Hyperparameter

-delta : Hyperparameter

-ngl: Number of global topics

-nloc: Number of local topics


Outputfile description:
jmts.piCombine : Sentiment distribution for review sentiment classification
[inputfilename.csv] all topics in csv file


Contact

Please direct questions to Md. Hijbul Alam .

License

Following JGibbLDA, this code is licensed under the GPLv2. Please see the LICENSE file for the full license.

JMTS in Java Copyright (C) 2015 Md. Hijbul Alam, SangKeun Lee (JMTS), Xuan-Hieu Phan and Cam-Tu Nguyen (JGibbLDA)

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.


JMTS softwate is built upon the framework of JGibbLDA http://jgibblda.sourceforge.net/

