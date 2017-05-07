/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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
#include <fsmac/exchanger.h>
#include <pmt/pmt.h>
#include <list>
#include <stdio.h>

using namespace gr::fsmac;

class exchanger_impl : public exchanger {
    int active_protocol = 1;
    int next_protocol = 1;
    int state = 0;
    bool sending = false;
    boost::shared_ptr<gr::thread::thread> activeProtSender;
    long int timeToWait = 5000;
    boost::mutex mut;

public:
#define EXCHANGE_COMMAND_SEND_INFO 0
#define EXCHANGE_COMMAND_INFO_SENT 1
#define EXCHANGE_COMMAND_STOP 2
#define EXCHANGE_COMMAND_DONE 3
#define LATENCY_SENSOR_COMMAND_SEND 4

#define NORMAL_STATE 0
#define EXCHANGE_STATE 1

    /*
     * The private constructor
     */
    exchanger_impl(bool is_coord)
    : gr::block("exchanger",
    gr::io_signature::make(0, 0, 0),
    gr::io_signature::make(0, 0, 0)),
    is_coordinator(is_coord)
    {
        message_port_register_in(pmt::mp("dec in"));
        set_msg_handler(pmt::mp("dec in"), boost::bind(&exchanger_impl::dec_in, this, _1));
        message_port_register_out(pmt::mp("dec out"));

        message_port_register_in(pmt::mp("app in"));
        set_msg_handler(pmt::mp("app in"), boost::bind(&exchanger_impl::app_in, this, _1));
        message_port_register_out(pmt::mp("app out"));

        message_port_register_in(pmt::mp("mac in"));
        set_msg_handler(pmt::mp("mac in"), boost::bind(&exchanger_impl::mac_in, this, _1));
        message_port_register_out(pmt::mp("mac out"));

        message_port_register_in(pmt::mp("p1_app in"));
        set_msg_handler(pmt::mp("p1_app in"), boost::bind(&exchanger_impl::p1_app_in, this, _1));
        message_port_register_out(pmt::mp("p1_app out"));

        message_port_register_in(pmt::mp("p1_mac in"));
        set_msg_handler(pmt::mp("p1_mac in"), boost::bind(&exchanger_impl::p1_mac_in, this, _1));
        message_port_register_out(pmt::mp("p1_mac out"));

        message_port_register_in(pmt::mp("p2_app in"));
        set_msg_handler(pmt::mp("p2_app in"), boost::bind(&exchanger_impl::p2_app_in, this, _1));
        message_port_register_out(pmt::mp("p2_app out"));

        message_port_register_in(pmt::mp("p2_mac in"));
        set_msg_handler(pmt::mp("p2_mac in"), boost::bind(&exchanger_impl::p2_mac_in, this, _1));
        message_port_register_out(pmt::mp("p2_mac out"));

        message_port_register_in(pmt::mp("p1_ctrl in"));
        set_msg_handler(pmt::mp("p1_ctrl in"), boost::bind(&exchanger_impl::p1_ctrl_in, this, _1));
        message_port_register_out(pmt::mp("p1_ctrl out"));

        message_port_register_in(pmt::mp("p2_ctrl in"));
        set_msg_handler(pmt::mp("p2_ctrl in"), boost::bind(&exchanger_impl::p2_ctrl_in, this, _1));
        message_port_register_out(pmt::mp("p2_ctrl out"));
    }

    /*
     * Our virtual destructor.
     */
    ~exchanger_impl(void) {
    }
    
    bool start(){
        pmt::pmt_t pack_list = pmt::make_dict();
        if(active_protocol == 1){
            message_port_pub(pmt::mp("p1_ctrl out"), pack_list);            
        }else if(active_protocol == 2){
            message_port_pub(pmt::mp("p2_ctrl out"), pack_list);
        }
        
        activeProtSender = boost::shared_ptr<gr::thread::thread>
                (new gr::thread::thread(boost::bind(&exchanger_impl::sendInformation, this)));
        
        return block::start();
    }
    
    void sendInformation() {
        while (is_coordinator) {
            if (!sending) {         
//                printf("Enviando PERIODIC_SEND_INFO - PROT ATIVO\n");
                pmt::pmt_t exch_command = pmt::cons(pmt::from_uint64(EXCHANGE_COMMAND_SEND_INFO), pmt::from_uint64(active_protocol));
                if (active_protocol == 1) {
                    message_port_pub(pmt::mp("p1_ctrl out"), exch_command);
                } else if (active_protocol == 2) {
                    message_port_pub(pmt::mp("p2_ctrl out"), exch_command);
                }
            }

            boost::posix_time::millisec timeToSendoNewInformation(timeToWait);
            boost::this_thread::sleep(timeToSendoNewInformation);
        }
    }

    void dec_in(pmt::pmt_t msg) {
//        printf("EXCH: Entrou no DEC IN\n");
        if (pmt::is_dict(msg) && state == NORMAL_STATE) {
            boost::unique_lock<boost::mutex> lock(mut);
//            pmt::print(msg);
            pmt::pmt_t best_prot = pmt::dict_ref(msg, pmt::from_uint64(1), pmt::get_PMT_NIL());
            pmt::pmt_t secound_prot = pmt::dict_ref(msg, pmt::from_uint64(2), pmt::get_PMT_NIL());
//            printf("Protocolo ativo no momento: %d\n", active_protocol);
            int prot1_number = pmt::to_uint64(pmt::car(best_prot));
            float prot1_adapt = pmt::to_float(pmt::cdr(best_prot));
//            std::cout << "Best number:" << prot1_number << std::endl;
//            std::cout << "Best adapt:" << prot1_adapt << std::endl;

            int prot2_number = pmt::to_uint64(pmt::car(secound_prot));
            float prot2_adapt = pmt::to_float(pmt::cdr(secound_prot));
//            std::cout << "Sec number:" << prot2_number << std::endl;
//            std::cout << "Sec adapt:" << prot2_adapt << std::endl;

            float diff = prot1_adapt - prot2_adapt;
            
            if(diff < 0){
                diff = diff * (-1);
            }
            
            if ((prot1_number != active_protocol) && (diff > 5) && state == NORMAL_STATE) {
//                std::cout << "Deve trocar o protocolo" << std::endl;
                next_protocol = prot1_number;
                if (is_coordinator) {
                    notify_nodes(next_protocol);
                }
            }
            lock.unlock();
        }else if(pmt::is_uint64(msg) && !is_coordinator){
//            printf("EXCH: Entrou no IF do envio de latências\n");
            int command = pmt::to_uint64(msg);
//            printf("EXCH: Enviou comando periodico de latencia\n");
            if(command == LATENCY_SENSOR_COMMAND_SEND) {
                if (active_protocol == 1) {
                    message_port_pub(pmt::mp("p1_ctrl out"), msg);
                } else if (active_protocol == 2) {
                    message_port_pub(pmt::mp("p2_ctrl out"), msg);
                } 
            }
        }
    }

    void notify_nodes(int new_prot) {
        if(!sending){
            sending = true;
//            printf("Enviando SEND_INFO\n");
            pmt::pmt_t exch_command = pmt::cons(pmt::from_uint64(EXCHANGE_COMMAND_SEND_INFO), pmt::from_uint64(new_prot));
//            pmt::pmt_t exch_command = pmt::from_uint64(EXCHANGE_COMMAND_SEND_INFO);
            if (active_protocol == 1) {
                message_port_pub(pmt::mp("p1_ctrl out"), exch_command);
            } else if (active_protocol == 2) {
                message_port_pub(pmt::mp("p2_ctrl out"), exch_command);
            }            
        }        
    }

    void exchange_protocol(int new_prot) {
        if (is_coordinator) {
            state = EXCHANGE_STATE;
            pmt::pmt_t exch_command = pmt::from_uint64(EXCHANGE_COMMAND_STOP);
            if (active_protocol == 1) {
                message_port_pub(pmt::mp("p1_ctrl out"), exch_command);
            } else if (active_protocol == 2) {
                message_port_pub(pmt::mp("p2_ctrl out"), exch_command);
            }
        }

        //enviar mensagem para os outros nós
        //enviar mensagem de controle pedindo a fila
        //desativar protocolo atual
        //montar a fila do novo protocolo
        //ativar novo protocolo
    }

    void p1_ctrl_in(pmt::pmt_t msg) {
        control_in(msg);
    }

    void p2_ctrl_in(pmt::pmt_t msg) {
        control_in(msg);
    }
    
    bool stop(){
        activeProtSender->interrupt();
        return block::stop();
    }

    void control_in(pmt::pmt_t msg) {
        if (pmt::is_uint64(msg)) {
            int command = pmt::to_uint64(msg);
            if (command == EXCHANGE_COMMAND_INFO_SENT) {//stop active protocol                 
                sending = false;
                state = EXCHANGE_STATE;
                pmt::pmt_t exch_command = pmt::from_uint64(EXCHANGE_COMMAND_STOP);
                if (active_protocol == 1) {
                    message_port_pub(pmt::mp("p1_ctrl out"), exch_command);
                } else if (active_protocol == 2) {
                    message_port_pub(pmt::mp("p2_ctrl out"), exch_command);
                }
            }else if(command == EXCHANGE_COMMAND_DONE){
				printf("\nExchanging...\n\n");
                active_protocol = next_protocol;
//                printf("EXCH: Novo protocolo ativo: %d\n", active_protocol);
//                printf("EXCH: Recebeu DONE\n");
                state = NORMAL_STATE;
                pmt::pmt_t active_prot_info_dec = pmt::from_uint64(active_protocol);
//                pmt::print(active_prot_info_dec);
                message_port_pub(pmt::mp("dec out"), active_prot_info_dec);
//              printf("EXCH: Enviou comunicado ao módulo de Decisão\n");
				printf("New protocol activated!\n\n");
            }
        } else if (pmt::is_dict(msg)) {//start new protocol
//            printf("EXCH: Recebeu fila para repassar\n");
            if (next_protocol == 1) {
//                printf("EXCH: Antes do envio da fila\n");
                message_port_pub(pmt::mp("p1_ctrl out"), msg);
//                printf("EXCH: Lista: ");
//                pmt::print(msg);
//                printf("\n");
//                printf("EXCH: Depois do envio da fila\n");
            } else if (next_protocol == 2) {
//                printf("EXCH: Antes do envio da fila\n");
                message_port_pub(pmt::mp("p2_ctrl out"), msg);
//                printf("EXCH: Lista: ");
//                pmt::print(msg);
//                printf("\n");
//                printf("EXCH: Depois do envio da fila\n");
            }
        }
    }

    void app_in(pmt::pmt_t msg) {
        if (state == NORMAL_STATE) {
            if (active_protocol == 1) {
                message_port_pub(pmt::mp("p1_app out"), msg);
            } else if (active_protocol == 2) {
                message_port_pub(pmt::mp("p2_app out"), msg);
            }
        }
    }

    void mac_in(pmt::pmt_t msg) {
        pmt::pmt_t blob;

        if (pmt::is_pair(msg)) {
            blob = pmt::cdr(msg);
        } else {
            assert(false);
        }

        size_t data_len = pmt::blob_length(blob);
        //LOG printf("Tamanho do pacote recebido: %d\n", data_len);
        if (data_len < 11 && data_len != 6) {
            return;
        }

        char* recPackage = (char*) pmt::blob_data(blob);
        recPackage[data_len - 1] = '\0';
        data_len = data_len - 1;
        
        recPackage = (char*) pmt::blob_data(blob);
        uint16_t crc = crc16(recPackage, data_len);
        
        if(crc == 0){
            if(recPackage[0] == 0x41 && !is_coordinator && recPackage[9] != 'L'){
//                pmt::print(msg);
                int prot1_number;
                if(recPackage[9] == '1'){
                    prot1_number = 1;                    
                }else if(recPackage[9] == '2'){
                    prot1_number = 2;
                }

                if(prot1_number != active_protocol){
                    state = EXCHANGE_STATE;
//                    std::cout << "EXCH: Deve trocar o protocolo" << std::endl;
//                    printf("Ativo: %d - Novo: %d\n", active_protocol, prot1_number);
                    next_protocol = prot1_number;

                    pmt::pmt_t exch_command = pmt::from_uint64(EXCHANGE_COMMAND_STOP);
                    if (active_protocol == 1) {
                        message_port_pub(pmt::mp("p1_ctrl out"), exch_command);
                    } else if (active_protocol == 2) {
                        message_port_pub(pmt::mp("p2_ctrl out"), exch_command);
                    }                    
                }
                
            }else if (state == NORMAL_STATE) {
                if (active_protocol == 1) {
                    message_port_pub(pmt::mp("p1_mac out"), msg);
                } else if (active_protocol == 2) {
                    message_port_pub(pmt::mp("p2_mac out"), msg);
                }
            }
        }
    }

    void p1_app_in(pmt::pmt_t msg) {
        if (state == NORMAL_STATE) {
            if (active_protocol == 1) {
                message_port_pub(pmt::mp("app out"), msg);
            }
        }
    }

    void p2_app_in(pmt::pmt_t msg) {
        if (state == NORMAL_STATE) {
            if (active_protocol == 2) {
                message_port_pub(pmt::mp("app out"), msg);
            }
        }
    }

    void p1_mac_in(pmt::pmt_t msg) {
        if (state == NORMAL_STATE) {
//            printf("Protocolo ativo no MAC IN: %d\n", active_protocol);
            if (active_protocol == 1) {
                message_port_pub(pmt::mp("mac out"), msg);
            }
        }
    }

    void p2_mac_in(pmt::pmt_t msg) {
        if (state == NORMAL_STATE) {
            if (active_protocol == 2) {
                message_port_pub(pmt::mp("mac out"), msg);
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


    //    void
    //    exchanger_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    //    {
    //        /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    //    }
    //
    //    int
    //    exchanger_impl::general_work (int noutput_items,
    //                       gr_vector_int &ninput_items,
    //                       gr_vector_const_void_star &input_items,
    //                       gr_vector_void_star &output_items)
    //    {
    //        const <+ITYPE*> *in = (const <+ITYPE*> *) input_items[0];
    //        <+OTYPE*> *out = (<+OTYPE*> *) output_items[0];
    //
    //        // Do <+signal processing+>
    //        // Tell runtime system how many input items we consumed on
    //        // each input stream.
    //        consume_each (noutput_items);
    //
    //        // Tell runtime system how many output items we produced.
    //        return noutput_items;
    //    }
};

exchanger::sptr
exchanger::make(bool is_coord) {
    return gnuradio::get_initial_sptr
            (new exchanger_impl(is_coord));
}

