#include <iostream>
#include <string>
#include <map>

#include "kf.h"

using namespace std;
using namespace Eigen;

int main() {
    
    KF filter;
    
    filter.Update(-455.366,4614.99);
    
    filter.Predict(0.024);

    filter.Update(-455.336,4614.79);
    
    
    filter.Predict(0.024);

    filter.Update(-455.265,4614.43);

    filter.Predict(0.024);

    filter.Update(-455.251,4614.3);
    
    cout << filter.GetState();


}