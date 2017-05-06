
#include <list>

#include "MyList.h"

//using namespace pmt;

std::list<SendPackage*> sendList;
static MyList *instance;
bool flag = false;

std::list<SendPackage*> getList() {
    return sendList;
}

void MyList::push_back(SendPackage* pack){
    sendList.push_back(pack);
}

bool MyList::empty(){
    return sendList.empty();
}

std::list<SendPackage*>::iterator MyList::begin(){
    return sendList.begin();
}

std::list<SendPackage*>::iterator MyList::end(){
    return sendList.end();
}

void MyList::remove(SendPackage* pack){
    sendList.remove(pack);
}

size_t MyList::size(){
    return sendList.size();
}

SendPackage* MyList::front(){
    return sendList.front();
}

void MyList::pop_front(){
    sendList.pop_front();
}

MyList& MyList::Instance()
{
    if(!flag){
        instance = new MyList();
    }
    
    return *instance;
}
