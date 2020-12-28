#include <iostream>
#include <string>
#include <vector>

#include "mo-tracker.h"

using namespace std;
using namespace Eigen;

int getTypeIndex(string type) {
    switch(type[0]) {
        case 'P': //Pedestrian
            return 0;
        case 'V': //Vehicle
            return 1;
        case 'T': //TrafficLight
            return 2;
        default:
            return -1;
    }       
}


int main() {
    int N = 0;
    int timeStamp;
    string type;
    double x,y;
    vector<bool> firstLineOfData = {true, true, true};
    vector<MultipleObjectTracker> multipleTrackers(3);
    pair<int,pair<double,double>> res;
    IdFactory factory;
    
    while ( cin >> timeStamp >> type >> x >> y) {
        int typeIndex = getTypeIndex(type);
        
        if (typeIndex == -1) {
            cout << "Unknown Type" << endl;
            return -1;
        }
        
        if (firstLineOfData[typeIndex]) {          
            multipleTrackers[typeIndex] = MultipleObjectTracker(timeStamp, & factory); 
            firstLineOfData[typeIndex] = false;
        }
                        
        res = multipleTrackers[typeIndex].RegisterEvent(timeStamp, x, y);
        
        cout << timeStamp << " " << type << " " << x << " " << y << " "
             << res.first << " " << res.second.first << " " << res.second.second << endl;
        ++N;
    }
}