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

#ifdef HAVE_CONFIG_H
//#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <list>
#include <pmt/pmt.h>
#include "fsmac/sens_num_senders.h"
#include <stdio.h>

using namespace gr::fsmac;
    
class sens_num_senders_impl : public sens_num_senders{
    
    std::list<char*> sendersSet;
    boost::shared_ptr<gr::thread::thread> waitToSend;
    bool sendValueToDecision = true;
    
    public:
    /*
     * The private constructor
     */
    sens_num_senders_impl()
      : gr::block("sens_num_senders",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0))
    {        
        message_port_register_in(pmt::mp("pdu in"));
        set_msg_handler(pmt::mp("pdu in"), boost::bind(&sens_num_senders_impl::pdu_in, this, _1));
        
        message_port_register_out(pmt::mp("dec out"));
    }

    /*
     * Our virtual destructor.
     */
    ~sens_num_senders_impl()
    {
    }

    void waitSendingTime() {
        long int timeToWait = 5000;
        boost::posix_time::millisec workTime(timeToWait);
        boost::this_thread::sleep(workTime);
        sendValueToDecision = true;
    }
    
    void pdu_in(pmt::pmt_t msg) {
        if(sendValueToDecision){
            int numSenders = sendersSet.size();
            pmt::pmt_t numSenders_pmt = pmt::from_uint64(numSenders);
            message_port_pub(pmt::mp("dec out"), numSenders_pmt);
            sendersSet.clear();
            sendValueToDecision = false;
            
            waitToSend = boost::shared_ptr<gr::thread::thread>
                            (new gr::thread::thread(boost::bind(&sens_num_senders_impl::waitSendingTime, this)));
        }
        
        pmt::pmt_t blob;

        if (pmt::is_pair(msg)) {
            blob = pmt::cdr(msg);
        } else {
            assert(false);
        }

        size_t data_len = pmt::blob_length(blob);
        if (data_len < 11 && data_len != 6) {
            return;
        }

        char* recPackage = (char*) pmt::blob_data(blob);
        recPackage[data_len - 1] = '\0';
        data_len = data_len - 1;
        
        recPackage = (char*) pmt::blob_data(blob);
        uint16_t crc = crc16(recPackage, data_len);
        
        if (crc == 0 && recPackage[0] == 97) {
            bool elementIsPresent = false;  
            
            if (!sendersSet.empty()) {
                std::list<char*>::iterator it = sendersSet.begin();
                while (it != sendersSet.end()) {
                    
                    if((*it)[0] == recPackage[7] && (*it)[1] == recPackage[8]){
                        elementIsPresent = true;
                        break;
                    }
                    
                    it++;                    
                }

            }
            
            if(!elementIsPresent){
                char* sender = (char*)malloc(sizeof(char)*2);
                sender[0] = recPackage[7];
                sender[1] = recPackage[8];

                sendersSet.push_back(sender);                
            }
        }

    }
    
    uint16_t crc16(char *buf, int len) {
        uint16_t crc = 0;

        for (int i = 0; i < len; i++) {
            for (int k = 0; k < 8; k++) {
                int input_bit = (!!(buf[i] & (1 << k)) ^ (crc & 1));
                crc = crc >> 1;
                if (input_bit) {
                    crc ^= (1 << 15);
                    crc ^= (1 << 10);
                    crc ^= (1 << 3);
                }
            }
        }

        return crc;
    }

    void printPack(char* pack, int data_len) {
        int j = 0;
        for (j = 0; j < data_len; j++) {
            printf("%d-", pack[j]);
        }
        printf("\n");
    }

    void printPackChar(char* pack, int data_len) {
        int j = 0;
        for (j = 0; j < data_len; j++) {
            printf("%c-", pack[j]);
        }
        printf("\n");
    }

};

sens_num_senders::sptr
sens_num_senders::make()
{
  return gnuradio::get_initial_sptr
    (new sens_num_senders_impl());
}
