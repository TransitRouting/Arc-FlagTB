#include "Commands/NetworkIO.h"
#include "Commands/NetworkTools.h"
#include "Commands/QueryBenchmark.h"
#include "Commands/ArcTBPreprocessing.h"

#include "../Helpers/Console/CommandLineParser.h"

#include "../Shell/Shell.h"
using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;

    new ParseGTFS(shell);
    new GTFSToIntermediate(shell);

    new IntermediateMakeTransitive(shell);
    new ReduceGraph(shell);
    new ReduceToMaximumConnectedComponent(shell);
    new MakeOneHopTransfers(shell);
    new IntermediateToRAPTOR(shell);

    new RAPTORToTripBased(shell);
    new CreateLayoutGraph(shell);
    new ComputeArcFlagTB(shell);

    new RunTransitiveRAPTORQueries(shell);
    new RunTransitiveTripBasedQueries(shell);
    new RunTransitiveArcTripBasedQueries(shell);

    shell.run();
    return 0;
}
