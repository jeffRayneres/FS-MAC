/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SendPackage.cpp
 * Author: jefferson
 * 
 * Created on 18 de Março de 2016, 17:10
 */

#include "SendPackage.h"

using namespace pmt;

SendPackage::SendPackage() {
}

SendPackage::SendPackage(pmt::pmt_t pack, unsigned char idPack, bool hasAckPack){
    package = pack;
    resends = 0;
    idPackage = idPack;
    hasAck = hasAckPack;
    canRemove = false;
}

//SendPackage::SendPackage(const SendPackage& orig) {
//}

//SendPackage::~SendPackage() {
//}

pmt::pmt_t SendPackage::getPackage(){
    return package;
}

short SendPackage::getResends(){
    return resends;
}

void SendPackage::increaseResends(int limit){
    resends = resends + 1;
    if(resends > limit){
        this->setCanRemove(true);
    }
}

void SendPackage::setPackage(pmt::pmt_t pack){
    package = pack;
}

void SendPackage::setResends(int resd){
    resends = resd;
}

void SendPackage::setTime(long int timeSent){
    time = timeSent;
}

long int SendPackage::getTimeSent(){
    return time;
}

unsigned char SendPackage::getId(){
    return idPackage;
}

bool SendPackage::hasAckPackage(){
    return hasAck;
}

void SendPackage::setHasAckPackage(bool hAck){
    hasAck = hAck;
}

void SendPackage::setCanRemove(bool canR){
    canRemove = canR;
}

bool SendPackage::getCanRemove(){
    return canRemove;
}

