/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SendPackage.h
 * Author: jefferson
 *
 * Created on 18 de Março de 2016, 17:10
 */

#ifndef SENDPACKAGE_H
#define SENDPACKAGE_H

#include <pmt/pmt.h>

class SendPackage {
public:
    SendPackage();
    SendPackage(pmt::pmt_t pack, unsigned char idPack, bool hasAckPack);
//    SendPackage(const SendPackage& orig);
//    virtual ~SendPackage();
    short getResends();
    pmt::pmt_t getPackage();
    void setResends(int resends);
    void setPackage(pmt::pmt_t pack);
    void setTime(long int timeSent);
    long int getTimeSent();
    unsigned char getId();
    void increaseResends(int limit);
    bool hasAckPackage();
    void setHasAckPackage(bool hAck);
    void setCanRemove(bool canR);
    bool getCanRemove();
        
private:
    long int time;
    short resends;
    pmt::pmt_t package;
    unsigned char idPackage;
    bool hasAck;
    bool canRemove;
};

#endif /* SENDPACKAGE_H */

