#pragma once

#include "so-tracker.h"
#include <forward_list>


class IdFactory {
private:
    int N;
public:
    IdFactory(): N(0) {        
    }
    int GetNewId() {
        ++N;
        return N;
    }
};

class MultipleObjectTracker
{
public:
    static constexpr double ThreshholdDistance = 10.0;
private: 
    forward_list<SingleObjectTracker> Trackers;
    int CurrentTimeStamp;
    IdFactory * IdFactoryHandle;
public:
    MultipleObjectTracker(int timeStamp = 0, IdFactory* handle = NULL): 
    CurrentTimeStamp(timeStamp), IdFactoryHandle(handle)
    {}
    
    pair<double, forward_list<SingleObjectTracker>::iterator> 
        GetBest(double x, double y) { 
        // Assumes that Trackers is not empty
            double best = Trackers.begin() -> GetDistance(x,y);
            auto bestIterator = Trackers.begin();
            for (forward_list<SingleObjectTracker>::iterator it = next(Trackers.begin());
                    it != Trackers.end(); ++it) {
                if ((it -> GetDistance(x,y)) < best ) {
                    bestIterator = it;
                    best = it -> GetDistance(x,y);
                }                    
            }
            //cout << "                                              best is " << best << endl;
            return make_pair(best, bestIterator);
        }
        
    auto CreateNewSingleObjectTracker
                        (int timeStamp, double x, double y) {
        SingleObjectTracker tempTracker(timeStamp, x, y, IdFactoryHandle -> GetNewId() );
        Trackers.push_front(tempTracker);
        return Trackers.begin();
    }
    
    int GetTSize() {
        int count = 0;
        for (auto it = Trackers.begin(); it != Trackers.end(); ++it) {
            ++ count;
        }
        return count;
    }
        
    pair<int, pair<double,double>>  RegisterEvent
                       (int timeStamp, double x, double y) {
        
        if (timeStamp == CurrentTimeStamp) {
            if ( Trackers.empty() ) {
                return CreateNewSingleObjectTracker(timeStamp, x, y) ->
                                        GetIdAndPolarVelocity();
            } else {
                pair<double, forward_list<SingleObjectTracker>::iterator> best
                    = GetBest(x,y);
                double bestDistance = best.first;
                auto bestSingleTrackerIterator = best.second;
                if (bestDistance < ThreshholdDistance) 
                {
                    if (bestSingleTrackerIterator -> IsOpen() ) {
                        bestSingleTrackerIterator -> RegisterEvent(timeStamp, x, y);
                        return bestSingleTrackerIterator -> GetIdAndPolarVelocity();
                    } else {
                        return make_pair(0, make_pair(0,0));
                    }
                } else {
                    return CreateNewSingleObjectTracker(timeStamp, x, y) ->         
                                         GetIdAndPolarVelocity();
                }                
            }
        } else {
            
            // Traverse singly-linked list of s-o-trackers and remove
            // those for which Update function returns true. 
            
            // This code is crazy because you use different function 
            // for removing head- and non-head- elements.
            
            //cout << "        # trackers before erasing " << GetTSize() << endl;
            
            while (Trackers.begin() != Trackers.end() 
                     &&  Trackers.begin() -> Update(timeStamp)) {
            //    cout << "             erasing something at the beginning" << endl;
                Trackers.pop_front();
            }
            auto prev = Trackers.begin();
            if (prev != Trackers.end() ) {
                auto it = next(prev);
                while (it != Trackers.end()) {
                    if (it -> Update(timeStamp) ) {                    
            //             cout << "             erasing something in the middle" << endl;
                        Trackers.erase_after(prev);
                    } else {
                        prev = next(prev);
                        if (prev == Trackers.end()) {
                            break;
                        }
                    }
                    it = next(prev);                
                }
            }
            // cout << "        # trackers after erasing " << GetTSize() << endl;            
            CurrentTimeStamp = timeStamp;
            return RegisterEvent(timeStamp, x, y);
        }
    }
};
