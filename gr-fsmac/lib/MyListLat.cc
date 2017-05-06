
#include <list>
#include <bits/c++config.h>

#include "MyListLat.h"

//using namespace pmt;

std::list<double> sendListLat;
static MyListLat *instance;
bool flagLat = false;

std::list<double> getListLat() {
    return sendListLat;
}

void MyListLat::push_back(double pack){
    sendListLat.push_back(pack);
}

bool MyListLat::empty(){
    return sendListLat.empty();
}

std::list<double>::iterator MyListLat::begin(){
    return sendListLat.begin();
}

std::list<double>::iterator MyListLat::end(){
    return sendListLat.end();
}

void MyListLat::remove(double pack){
    sendListLat.remove(pack);
}

std::size_t MyListLat::size(){
    return sendListLat.size();
}

double MyListLat::front(){
    return sendListLat.front();
}

void MyListLat::pop_front(){
    sendListLat.pop_front();
}

void MyListLat::clear(){
    sendListLat.clear();
}

MyListLat& MyListLat::Instance()
{
    if(!flagLat){
        instance = new MyListLat();
    }
    
    return *instance;
}
