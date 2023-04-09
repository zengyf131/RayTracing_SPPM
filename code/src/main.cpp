#include "SPPM.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    double start = omp_get_wtime( );

    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        std::cout << "Usage: ./bin/render <input scene file> <output bmp file>" << std::endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.

    SPPM* sppm = new SPPM(inputFile, outputFile);
    sppm->rayIterate();
    delete sppm;

    double end = omp_get_wtime( );
    std::cout << "done in " << end - start << "s." << std::endl;
    return 0;
}