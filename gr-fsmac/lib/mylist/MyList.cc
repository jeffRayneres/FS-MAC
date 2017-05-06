
#include <list>

#include "MyList.h"

//using namespace pmt;

std::list<SendPackage*> sendList;
static MyList *instance;
bool flag = false;

std::list<SendPackage*> getList() {
    return sendList;
}

MyList& MyList::Instance()
{
    if(!flag){
        instance = new MyList();
    }
    
    return *instance;
}
