[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Arc-Flag Trip-Based Public Transit Routing (Arc-Flag TB)

This repository is based on the ULTRA framework. For more information, see [ULTRA on GitHub](https://github.com/kit-algo/ULTRA). 

This repository contains the code for 
* *Arc-Flags Meet Trip-Based Public Transit Routing*
  Ernestine Großmann, Jonas Sauer, Christian Schulz, Patrick Steil
  [arXiv](https://arxiv.org/abs/2302.07168)

but also for the following publications:
* *UnLimited TRAnsfers for Multi-Modal Route Planning: An Efficient Solution*
  Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf
  In: Proceedings of the 27th Annual European Symposium on Algorithms (ESA'19), Leibniz International Proceedings in Informatics, pages 14:1–14:16, 2019
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2019/11135/pdf/LIPIcs-ESA-2019-14.pdf) [arXiv](https://arxiv.org/abs/1906.04832)
* *Integrating ULTRA and Trip-Based Routing*
  Jonas Sauer, Dorothea Wagner, Tobias Zündorf
  In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 4:1–4:15, 2020
  [pdf](http://i11www.ira.uka.de/extra/publications/swz-iultr-20.pdf)

* *Fast Multimodal Journey Planning for Three Criteria*
  Moritz Potthoff, Jonas Sauer
  In: Proceedings of the 24th Workshop on Algorithm Engineering and Experiments (ALENEX'22), SIAM, pages 145–157, 2022
  [pdf](https://epubs.siam.org/doi/epdf/10.1137/1.9781611977042.12) [arXiv](https://arxiv.org/abs/2110.12954)

* *Efficient Algorithms for Fully Multimodal Journey Planning*
  Moritz Potthoff, Jonas
  Accepted for publication at the 22nd Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'22)

## Usage
To use the Arc-Flag TB algorithm, compile the executables in the  ``Runnables`` folder (using the ``Makefile``). Next to the ``ULTRA`` and ``Network`` executables, there is the ``ARCTB`` executable and also two scripts ``prepLayoutGraph.scirpt`` and ``arcFlagTB.script``. 
When executing ``ARCTB``, you can call ``runScript`` followed by the script name. For any command, you can call ``help``followed by the command name to get information about the usage.

* First Step: Prepare the GTFS Data by running ``prepLayoutGraph.scirpt``. 
**Do not forget to change the directories to the GTFS Data in the script and the place where you want to store the computed binaries.** The script does the following automatically:
	* ``parseGTFS``simply parses the GTFS directory into a GTFS binary
	* ``gtfsToIntermediate`` uses the GTFS binary and computes an Intermediate binary (Note that the day-extraction happens here as well).
	* Then some data reductions are used to e.g., only use the largest connected component or remove any degree two vertices.
	* ``intermediateToRAPTOR`` transforms the Intermediate binary into the RAPTOR binary
	* ``createLayoutGraph`` computes the layout graph for the given RAPTOR binary and stores it in the METIS file format (readable by [KaHIP](https://github.com/KaHIP/KaHIP)).
* Second Step: Partition the layout graph. We recommend using [KaHIP](https://github.com/KaHIP/KaHIP), since we used this tool in all of our experiments. Note that the partitioning step has to be performed outside of this repo.
* Third Step: Compute the Flags. After partitioning the layout graph, you can now compute the Flags by running the ``arcFlagTB.script`` script. **Again, make sure that the binary- and 'partitioned text files' - paths are correct**
	* As a first step, ``raptorToTripBased``computes the TripBased Data from the RAPTOR binary. Note that as a third parameter, one can pass the text file with the partition values.
	* Finally, ``computeArcFlagTB``computes the Flags for the given Trip-Based data. Note that as a third parameter, one can pass the text file with the partition values (not necessary if you already passed it before during ``raptorToTripBased``). Here are optional arguments such as ``Fixing Departure Time``or ``Buffering`` as well as the additional Flag Compression. 

Additionally, query performance can be evaluated by using the following commands:
* ``runTransitiveTripBasedQueries`` runs Trip-Based queries. Note that some *transferedges* are deleted (during the Flag Computation), since unnecessary edges are not needed for correct queries. Hence by comparing the performance of Arc-Flag TB and the original TB, make sure to not pass the already 'smaller' Trip-Based binary to test the original algorithm.
* ``runTransitiveArcTripBasedQueries`` runs Arc-Flag TB queries. Note that an additional third parameter can be used to switch from / to the compressed Arc-Flag TB variant.
