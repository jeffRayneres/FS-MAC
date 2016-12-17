/* -*- c++ -*- */
/* 
 * Copyright 2016 <Jefferson Rayneres Silva Cordeiro {jeff@dcc.ufmg.br} - DCC/UFMG>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
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

