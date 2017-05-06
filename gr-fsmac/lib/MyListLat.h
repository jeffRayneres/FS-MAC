/* 
 * File:   MyListLat.h
 * Author: jefferson
 *
 * Created on 27 de Novembro de 2016, 22:20
 */

#ifndef MyListLat_H
#define MyListLat_H

//#include <SendPackage.h>
//#include <pmt/pmt.h>

class MyListLat {
public:
    static MyListLat& Instance();
    std::list<double> getListLat();
    void push_back(double pack);
    std::size_t size();
    bool empty();
    std::list<double>::iterator begin();
    std::list<double>::iterator end();
    void remove(double pack);
    double front();
    void pop_front();
    void clear();
    
private:
//  std::list<double> sendListLat;
//  MyListLat();
//  ~MyListLat();

}; 

#endif /* MyListLat_H */

