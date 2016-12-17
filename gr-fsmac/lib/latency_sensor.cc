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
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <fsmac/latency_sensor.h>
#include <stdio.h>

using namespace gr::fsmac;

class latency_sensor_impl : public latency_sensor{    
    boost::shared_ptr<gr::thread::thread> latencySender;
    long int timeToWait = 10000;
    
    std::list<double> latencyListSum;
    float latency_counter = 0.0;
    
public:
    #define EXCHANGE_COMMAND_SEND_INFO 0
    #define EXCHANGE_COMMAND_INFO_SENT 1
    #define EXCHANGE_COMMAND_STOP 2
    #define EXCHANGE_COMMAND_DONE 3
    #define LATENCY_SENSOR_COMMAND_SEND 4

    /*
     * The private constructor
     */
    latency_sensor_impl(bool is_coord): gr::block("latency_sensor",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
            is_coordinator(is_coord)
    {                
        
        message_port_register_in(pmt::mp("pdu in"));
        set_msg_handler(pmt::mp("pdu in"), boost::bind(&latency_sensor_impl::pdu_in, this, _1));
        
        message_port_register_out(pmt::mp("dec out"));
    }

    /*
     * Our virtual destructor.
     */
    ~latency_sensor_impl(void)
    {
    }
    
    bool start() {
        latencySender = boost::shared_ptr<gr::thread::thread>
                (new gr::thread::thread(boost::bind(&latency_sensor_impl::sendInformation, this)));


        return block::start();
    }

    bool stop(){
        latencySender->interrupt();        
        return block::stop();
    }
    
    void sendInformation() {
        while (true) {
            if(!is_coordinator){
                pmt::pmt_t latency_command = pmt::from_uint64(LATENCY_SENSOR_COMMAND_SEND);
                message_port_pub(pmt::mp("dec out"), latency_command);
            } else if (is_coordinator) {
                float sum = 0.0;
                
                std::list<double>::iterator it = latencyListSum.end();
                float instant_latency_counter = latency_counter;
                
                while (it != latencyListSum.begin()) {
                    sum = sum + (*it);
                    it--;
                }

                float latencyAv = sum / instant_latency_counter;
                latencyListSum.clear();
                latency_counter = 0.0;
                
                pmt::pmt_t final_latency_av_pmt = pmt::from_float(latencyAv);
                message_port_pub(pmt::mp("dec out"), final_latency_av_pmt);
            }
            
            boost::posix_time::millisec timeToSendoNewInformation(timeToWait);
            boost::this_thread::sleep(timeToSendoNewInformation);                
        }
    }

    void printPackChar(char* pack, int data_len) {
        int j = 0;
        for (j = 0; j < data_len; j++) {
            printf("%c-", pack[j]);
        }
        printf("\n");
    }

    void pdu_in(pmt::pmt_t msg) {
        if (is_coordinator) {
            pmt::pmt_t blob;
            pmt::pmt_t latency_counter_pmt;

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

            if (crc == 0 && recPackage[0] == 0x41 && recPackage[9] == 'L') {
                int strSize = data_len - 10 - 2;
                
                char* count_plus_latency_chars = (char*)malloc(sizeof(char)*strSize);
                int i = 0;
                for(i=10; i<10 + strSize; i++){
                    count_plus_latency_chars[i-10] = recPackage[i];
                }                
                
                int j = 0;
                int split_index = 0;
                for (j=0; j<strSize; j++){
                    if(count_plus_latency_chars[j] == '&'){
                        split_index = j;
                        break;
                    }
                }
                
                char count_char[split_index + 1]; 
                char latency_char[strSize - split_index];
                
                for(i=0; i<split_index; i++){
                    count_char[i] = count_plus_latency_chars[i];
                }
                count_char[split_index] = '\0';        
                
                               
                
                for(j=split_index + 1; j<strSize; j++){
                    latency_char[j - (split_index + 1)] = count_plus_latency_chars[j];
                }
                
                latency_char[strSize] = '\0';
                
                std::string local_latency_str(latency_char);
                std::string local_latency_counter_str(count_char);

                double latency;

                std::istringstream s(local_latency_str);
                s >> latency;
                
                
                int local_latency_counter;         
                
                std::istringstream ss(local_latency_counter_str);
                ss >> local_latency_counter;
                               
                float latency_sum = latency * local_latency_counter;
                
                latency_counter = latency_counter + local_latency_counter;
                latencyListSum.push_back(latency_sum);

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
    
private:
    bool is_coordinator;

};

latency_sensor::sptr
latency_sensor::make(bool is_coord) {
    return gnuradio::get_initial_sptr
            (new latency_sensor_impl(is_coord));
}

