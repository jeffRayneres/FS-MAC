/* 
 * File:   MyList.h
 * Author: jefferson
 *
 * Created on 27 de Novembro de 2016, 22:20
 */

#ifndef MYLIST_H
#define MYLIST_H

#include <SendPackage.h>
//#include <pmt/pmt.h>

class MyList {
public:
    static MyList& Instance();
    std::list<SendPackage*> getList();
private:
//  std::list<SendPackage*> sendList;
//  MyList();
//  ~MyList();

}; 

#endif /* MYLIST_H */

