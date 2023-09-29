#include "../Helpers/Console/CommandLineParser.h"
#include "../Shell/Shell.h"
#include "Commands/ArcTBPreprocessing.h"
#include "Commands/NetworkIO.h"
#include "Commands/NetworkTools.h"
#include "Commands/QueryBenchmark.h"

using namespace Shell;

int main(int argc, char** argv)
{
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;

    new ParseGTFS(shell);
    new GTFSToIntermediate(shell);
    new WriteTripBasedToCSV(shell);
    new WriteLayoutGraphToGraphML(shell);
    new WriteTripBasedToGraphML(shell);
    // new ComputePageRank(shell);

    new IntermediateMakeTransitive(shell);
    new ReduceGraph(shell);
    new ReduceToMaximumConnectedComponent(shell);
    new MakeOneHopTransfers(shell);
    new IntermediateToRAPTOR(shell);
    new IntermediateToCSA(shell);

    new RAPTORToTripBased(shell);
    new ComputeTransitiveEventToEventShortcuts(shell);
    new CreateLayoutGraph(shell);
    new ApplyPartitionToTripBased(shell);
    new ComputeArcFlagTB(shell);
    new ComputeArcFlagTBRAPTOR(shell);

    new RunTransitiveRAPTORQueries(shell);
    new RunTransitiveCSAQueries(shell);
    new RunTransitiveProfileCSAQueries(shell);
    new RunTransitiveTripBasedQueries(shell);

    /* new RunTransitiveTripBasedQueryExplizitly(shell); */
    
    new RunTransitiveProfileOneToAllTripBasedQueries(shell);
    new RunTransitiveProfileTripBasedQueries(shell);
    new RunTransitiveArcTripBasedQueries(shell);
    new RunTransitiveProfileArcTripBasedQueries(shell);

    new TestTransitiveArcTripBasedQueries(shell);

    shell.run();
    return 0;
}
