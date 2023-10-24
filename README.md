
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Arc-Flag Trip-Based Public Transit Routing (Arc-Flag TB)
This repository is based on the ULTRA framework. For more information, see [ULTRA on GitHub](https://github.com/kit-algo/ULTRA). To get actual GTFS data, see [gtfs.de](https://gtfs.de/) or [transit.land](https://www.transit.land/). In addition, it is worth stopping by [here](https://www.youtube.com/watch?v=dQw4w9WgXcQ).

This repository contains the code for

* *Arc-Flags Meet Trip-Based Public Transit Routing (Arc-Flag TB)* 
Ernestine Großmann, Jonas Sauer, Christian Schulz, Patrick Steil
In: Proceedings of the 21st International Symposium on Experimental Algorithms (SEA 2023), Schloss Dagstuhl - Leibniz-Zentrum für Informatik, pages 16:1-16:18, 2023
[pdf](https://drops.dagstuhl.de/opus/volltexte/2023/18366/pdf/LIPIcs-SEA-2023-16.pdf)

If you use this repository, please cite our work using

```
@inproceedings{gromann_et_al:LIPIcs.SEA.2023.16,
	title        = {{Arc-Flags Meet Trip-Based Public Transit Routing}},
	author       = {Gro{\ss}mann, Ernestine and Sauer, Jonas and Schulz, Christian and Steil, Patrick},
	year         = 2023,
	booktitle    = {21st International Symposium on Experimental Algorithms (SEA 2023)},
	publisher    = {Schloss Dagstuhl -- Leibniz-Zentrum f{\"u}r Informatik},
	address      = {Dagstuhl, Germany},
	series       = {Leibniz International Proceedings in Informatics (LIPIcs)},
	volume       = 265,
	pages        = {16:1--16:18},
	doi          = {10.4230/LIPIcs.SEA.2023.16},
	isbn         = {978-3-95977-279-2},
	issn         = {1868-8969},
	url          = {https://drops.dagstuhl.de/opus/volltexte/2023/18366},
	editor       = {Georgiadis, Loukas},
	urn          = {urn:nbn:de:0030-drops-183664},
	annote       = {Keywords: Public transit routing, graph algorithms, algorithm engineering}
}
```

This repo also contains the code for the following publications:

* *UnLimited TRAnsfers for Multi-Modal Route Planning: An Efficient Solution*
Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf
In: Proceedings of the 27th Annual European Symposium on Algorithms (ESA'19), Leibniz International Proceedings in Informatics, pages 14:1–14:16, 2019 [pdf](https://drops.dagstuhl.de/opus/volltexte/2019/11135/pdf/LIPIcs-ESA-2019-14.pdf) [arXiv](https://arxiv.org/abs/1906.04832)

* *Integrating ULTRA and Trip-Based Routing*
Jonas Sauer, Dorothea Wagner, Tobias Zündorf
In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 4:1–4:15, 2020 [pdf](http://i11www.ira.uka.de/extra/publications/swz-iultr-20.pdf)

* *Fast Multimodal Journey Planning for Three Criteria*
Moritz Potthoff, Jonas Sauer
In: Proceedings of the 24th Workshop on Algorithm Engineering and Experiments (ALENEX'22), SIAM, pages 145–157, 2022 [pdf](https://epubs.siam.org/doi/epdf/10.1137/1.9781611977042.12) [arXiv](https://arxiv.org/abs/2110.12954)

* *Efficient Algorithms for Fully Multimodal Journey Planning*
Moritz Potthoff, Jonas Sauer
Accepted for publication at the 22nd Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'22)

## Clone from Github

This repository contains a submodule (see ``ExternalLibs/sparsehash``), so clone this repo using the ``--recurse-submodules ...`` argument.
To install sparsehash (see https://github.com/sparsehash/sparsehash), call
```
>> cd ExternalLibs/sparsehash
>> ./configure --prefix=[the path to ExternalLibs/sparsehash]
>> make
>> make install
```
## Usage
To use the Arc-Flag TB algorithm, compile the executables in the  ``Runnables`` folder (using the ``Makefile``). Next to the ``ULTRA`` and ``Network`` executables are the ``ARCTB`` executable and two scripts called ``prepLayoutGraph.script`` and ``arcFlagTB.script``.

When executing ``ARCTB``, you can call ``runScript`` followed by the script name. For any command, you can call ``help`` followed by the command name to get information about the usage.

1. Prepare the GTFS Data by running ``prepLayoutGraph.script``. **Do not forget to change the directories to the GTFS Data in the script and the place where you want to store the computed binaries.** The script does the following automatically:

	* ``parseGTFS``simply parses the GTFS directory into a GTFS binary

	* ``gtfsToIntermediate`` uses the GTFS binary and computes an Intermediate binary (Note that the day-extraction also happens here).

	* Then some data reductions are used, e.g., only use the largest connected component or remove any degree two vertices.

	* ``intermediateToRAPTOR`` transforms the Intermediate binary into the RAPTOR binary

	* ``createLayoutGraph`` computes the layout graph for the given RAPTOR binary and stores it in the METIS file format (readable by [KaHIP](https://github.com/KaHIP/KaHIP)).

2. Partition the layout graph. We recommend using [KaHIP](https://github.com/KaHIP/KaHIP) since we used this tool in our experiments. Note that the partitioning step has to be performed outside of this repo. An example call for KaHIP could look like this: 
```
./KaHIP/deploy/kaffpaE Datasets/germany/raptor.layout.graph --k=128 --imbalance=5 --preconfiguration=ssocial --time_limit=600 --output_filename=Datasets/germany/raptor.partition128.txt
```

3. Compute the Flags. After partitioning the layout graph, you can compute the Flags by running the ``arcFlagTB.script`` script. **Again, make sure that the binary- and 'partitioned text files' - paths are correct**
	* As a first step, ``raptorToTripBased``computes the TripBased Data from the RAPTOR binary. Note that one can pass the text file with the partition values as a third parameter.

	* Finally, ``computeArcFlagTB``computes the Flags for the given Trip-Based data. Note that as a third parameter, one can pass the text file with the partition values (not necessary if you already passed it before during ``raptorToTripBased``). Here are optional arguments such as ``Fixing Departure Time``or ``Buffering``and the additional Flag Compression.

Additionally, query performance can be evaluated by using the following commands:

* ``runTransitiveTripBasedQueries`` runs Trip-Based queries. Note that some *transfer-edges* are deleted (during the Flag Computation) since unnecessary edges are unnecessary for correct queries. Hence by comparing the performance of Arc-Flag TB and the original TB, make sure not to pass the already 'smaller' Trip-Based binary to test the original algorithm.

* ``runTransitiveArcTripBasedQueries`` runs Arc-Flag TB queries. Note that an additional third parameter can be used to switch from/to the compressed Arc-Flag TB variant.

## Example

In the directory ``test`` is an example GTFS datasets (IC/ICE from [gtfs.de](https://gtfs.de/de/feeds/de_fv/)). You can run the executable ``ARCTB`` and, inside, call ``runScript prepLayoutGraph.script``. The last command executed by the script should produce the following output:

```
> createLayoutGraph ../test/raptor.binary ../test/exports/raptor.layout.graph true
Loading static graph from ../test/raptor.binary.graph
RAPTOR public transit data:
   Number of Stops:                   828
   Number of Routes:                2,755
   Number of Trips:                 8,408
   Number of Stop Events:         100,389
   Number of Connections:          91,981
   Number of Vertices:                828
   Number of Edges:                   160
   First Day:                           0
   Last Day:                            2
   Bounding Box:             [(2.35912, 44.5061) | (22.7764, 56.1501)]
   Number of partitions:                1
METIS-Graph creating with:
	trip weighted
	transfer weighted
100.00% (1ms)                                   
Graph has been created!
Number of vertices:	828
Number of edges:	4024
100.00% (0ms)                                   
Finished creating metis file ../test/exports/raptor.layout.graph
```

After partitioning the layout graph (there is an example with k=32 inside the ``test/exports`` folder), you can run ``ARCTB`` again and call ``runScript arcFlagTB.script``. This yields the following:

```
> computeArcFlagTB ../test/trip.binary ../test/trip.binary None false false
Loading static graph from ../test/trip.binary.raptor.graph
Loading static graph from ../test/trip.binary.graph
Trip-Based public transit data:
   Number of Stops:                   828
   Number of Routes:                2,755
   Number of Trips:                 8,408
   Number of Stop Events:         100,389
   Number of Connections:          91,981
   Number of Transfers:         1,145,788
   Number of Vertices:                828
   Number of Edges:                   160
   First Day:                           0
   Last Day:                            2
   Bounding Box:             [(2.35912, 44.5061) | (22.7764, 56.1501)]
   Number Of Partitions:               32
Computing ARCFlags with 6 threads.
Collecting all departure times
100.00% (2ms)                                   
Collecting done, we now sort all the departure times!
Now starting the preprocessing!
100.00% (446ms)                                 
Done! Now bitwise or-ing all the edges from the threads!
Preprocessing done!
Now deleting unnecessary edges
ARC-FLAG Stats:
Number of Flags set:          332051 (7%)
Number of removed edges:      1003183 (87%)
100.00% (45ms)                                  
Saving the compressed flags!
Done with compressed flags!
```
To now test the query performance, one can call the following:
```
> runTransitiveArcTripBasedQueries ../test/trip.binary 1000
Loading static graph from ../test/trip.binary.raptor.graph
Loading static graph from ../test/trip.binary.graph
Trip-Based public transit data:
   Number of Stops:                   828
   Number of Routes:                2,755
   Number of Trips:                 8,408
   Number of Stop Events:         100,389
   Number of Connections:          91,981
   Number of Transfers:           142,605
   Number of Vertices:                828
   Number of Edges:                   160
   First Day:                           0
   Last Day:                            2
   Bounding Box:             [(2.35912, 44.5061) | (22.7764, 56.1501)]
   Number Of Partitions:               32
Rounds: 5.09
Scanned trips: 140.52
Scanned stops: 770.43
Relaxed transfers: 1,933.84
Enqueued trips: 1,982.10
Added journeys: 2.18
Scan initial transfers: 0µs
Evaluate initial transfers: 3µs
Scan trips: 20µs
Total time: 24µs
Avg. journeys: 1.30
```

**Note** Testing the (original) TB performance will result in faster TB queries than before computing Arc-Flags since we deleted unnecessary edges in the process. So if you want to compare the Arc-TB performance against the (orignal) TB, you should test the performance of the (original) **before** computing the Arc-Flags.
