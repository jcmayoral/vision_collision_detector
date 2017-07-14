#include <stdio.h>
#include <featuredetection/FD.h>

using namespace std;

int main(int argc, char *argv[]){
    FaultDetection fd;
    fd.start();
    fd.run();
    fd.stop();
    std::cout << "inside" <<std::endl;
    return 0;
}
