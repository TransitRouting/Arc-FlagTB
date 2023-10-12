#include "../Helpers/Console/CommandLineParser.h"
#include "../Shell/Shell.h"
#include "Commands/TransferPatternsPreprocessing.h"

using namespace Shell;

int main(int argc, char** argv)
{
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;

    new RAPTORToTripBased(shell);
    new RAPTORToTransferPattern(shell);
    new ComputeTransitiveEventToEventShortcuts(shell);
    new CreateLayoutGraph(shell);

    shell.run();
    return 0;
}
