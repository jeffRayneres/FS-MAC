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

#include <fsmac/csma.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
//#include <list>
#include <cstdlib>

#include <iostream>
#include <iomanip>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <stdio.h>
#include <stddef.h>
#include <sys/time.h>
#include <time.h>

#include "SendPackage.h"
#include "MyList.h"
#include "MyListLat.h"

using namespace gr::fsmac;

class csma_impl : public csma {
    float lastAvPower = -1000.0;
    float referenceValueChannelBusy = -80;
    MyList& sendList = MyList::Instance();
    //    std::list<SendPackage*> sendList;
    //    MyList& mylist = MyList::Instance();
    std::list<SendPackage*> commandList;
    long int slotSize = 2; //milisecounds/2
    long int cw_backoff_min = 4; //number of slots/4
    long int cw_backoff_max = 64; //number of slots/64
    long int cw_current_backoff = 4; //number of slots/4
    long int real_backoff = 4; //number of slots/4
    long int difs = 5; //milisecounds/5
    long int sifs = 2; //milisecounds/2
    long int resend_waiting = 40; //milisecounds/80

    long int numPackEnviados = 0;
    long int numPackConfirmados = 0;
    long int numRetransmissoes = 0;
    int control_pack_len;

    short max_retr = 5;

    //Endereço MAC local. Os endereços possuem 2 bytes, assim, essas duas 
    //variáveis representam apenas um endereço. Para testes, esse endereço deve 
    //ser mudado para cada máquina. Deve-se lembrar de manter a coerência com 
    //os endereços de outras máquinas atribuidos na função start() e com a 
    //chamada da função generate_mac() feita na função app_in().
    char mac_addr_1; // = 0x41;
    char mac_addr_2; // = 0xe8;

    //    int indexDestNode;

    //Endereço de broadcast
    char addr_bc_1 = 0xff;
    char addr_bc_2 = 0xff;

    //Endereços de outras máquinas para simulações
    char addr0[2];
    char addr1[2];
    char addr2[2];
    char addr3[2];
    char addr4[2];
    char addr5[2];
    char addr6[2];

    //array que vai conter os endereços das outras máquinas
    char* addrs[7];

    boost::shared_ptr<gr::thread::thread> exec;
    boost::shared_ptr<gr::thread::thread> waitSending;
    boost::shared_ptr<gr::thread::thread> waitToComputeTb;  
    boost::shared_ptr<gr::thread::thread> waitToExchange;
    boost::condition_variable cond;
    boost::condition_variable cond2;
    boost::condition_variable cond3;
    boost::mutex mut;
    boost::mutex mut2;
    boost::mutex mut3;
    bool data_ready = false;
    bool comm_ready = false;
    bool answer_sending = false;
    bool canCompute = false;
    bool lastPackAckedOrTimeToResendFinished = false;
    struct timeval t1, t2;
    struct timeval myComT1, myComT2;
    struct timeval myComT3, myComT4;
    std::list<double> listaLatencias;
   // MyListLat& listaLatencias = MyListLat::Instance();
    std::list<double> latencyListForSensor;
    double elapsedTime = 0;
    bool waitingAck = false;
    long int timeToStart = 60000;
    long int timeToCalculateLatency = 60000;

    bool exchanging = true;
    float latencyAv = 0.0;
    int latency_counter = 0;
    long int num_fsmac_packs = 0;


public:
#define EXCHANGE_COMMAND_SEND_INFO 0
#define EXCHANGE_COMMAND_INFO_SENT 1
#define EXCHANGE_COMMAND_STOP 2
#define EXCHANGE_COMMAND_DONE 3
#define LATENCY_SENSOR_COMMAND_SEND 4

    //#define dout d_debug && std::cout

    csma_impl(int mac_addr, int dest_node, bool debug) :
    block("csma",
    gr::io_signature::make(0, 0, 0),
    gr::io_signature::make(0, 0, 0)),
    d_msg_offset(0),
    d_seq_nr(0),
    d_debug(debug),
    d_num_packet_errors(0),
    d_num_packets_received(0) {

        //        mac_addr_1 = addrs[mac_addr][0];//mac_addr[0];
        //        mac_addr_2 = addrs[mac_addr][1];//mac_addr[1];
        indexDestNode = dest_node;
        indexLocalMac = mac_addr;

        //        sendList = mylist.getList();

        message_port_register_in(pmt::mp("cs in"));
        set_msg_handler(pmt::mp("cs in"), boost::bind(&csma_impl::cs_in, this, _1));
        message_port_register_in(pmt::mp("app in"));
        set_msg_handler(pmt::mp("app in"), boost::bind(&csma_impl::app_in, this, _1));
        message_port_register_in(pmt::mp("pdu in"));
        set_msg_handler(pmt::mp("pdu in"), boost::bind(&csma_impl::mac_in, this, _1));
        message_port_register_in(pmt::mp("ctrl in"));
        set_msg_handler(pmt::mp("ctrl in"), boost::bind(&csma_impl::ctrl_in, this, _1));

        message_port_register_out(pmt::mp("app out"));
        message_port_register_out(pmt::mp("pdu out"));
        message_port_register_out(pmt::mp("ctrl out"));
    }

    ~csma_impl(void) {
    }

    void ctrl_in(pmt::pmt_t msg) {
        if (pmt::is_pair(msg)) {
            int command; // = pmt::to_uint64(msg);
            pmt::pmt_t command_pmt = pmt::car(msg);
            command = pmt::to_uint64(command_pmt);

            if (command == EXCHANGE_COMMAND_SEND_INFO) {
                int new_protocol;
                pmt::pmt_t new_protocol_pmt = pmt::cdr(msg);
                new_protocol = pmt::to_uint64(new_protocol_pmt);

                char new_prot;

                if (new_protocol == 1) {
                    new_prot = '1';
                } else if (new_protocol == 2) {
                    new_prot = '2';
                }

//                printf("CSMA: Recebeu comando de envio.\n");
                char commandFsmac[2];
                commandFsmac[1] = 0x01;

                char* request = (char*) malloc(sizeof (char)*1);
                request[0] = 'Z';

                char packRequest[256];
                char addr_d[2];
                addr_d[0] = addr_bc_1;
                addr_d[1] = addr_bc_2;

                generateControlFsmacPack(request, 0, new_prot, packRequest, EXCHANGE_COMMAND_SEND_INFO, addr_d);

                pmt::pmt_t packReq = pmt::cons(pmt::PMT_NIL, pmt::make_blob(packRequest, control_pack_len));
                SendPackage* packageFsmac = new SendPackage(packReq, commandFsmac[1], false);

//                printf("ANTES da notificacao para thread de envios\n");
                
                comm_ready = true;
                commandList.push_back(packageFsmac);
                data_ready = true;
                cond.notify_all();
//                printf("DEPOIS da notificacao para thread de envios\n");

                if (new_protocol != 1) {                    
                    answer_sending = true;
                }
            }
        } else if (pmt::is_uint64(msg)) {
            int command = pmt::to_uint64(msg);

            if (command == EXCHANGE_COMMAND_STOP) {
//                printf("CSMA: Parar atuação CSMA\n");
                exchanging = true;
                pmt::pmt_t pack_list = pmt::make_dict();

                //TODO passar a fila
                //                std::list<SendPackage*>::iterator it = sendList.begin();
                //                int index = 0;
                //                while (it != sendList.end()) {
                //                    pack_list = pmt::dict_add(pack_list, pmt::from_uint64(index), (*it)->getPackage());
                //                    it++;
                //                    index++;
                //                }         

                message_port_pub(pmt::mp("ctrl out"), pack_list);
                //                exec->interrupt();
            } else if (command == LATENCY_SENSOR_COMMAND_SEND) {
                calculateLatencyAv();

                if (latency_counter > 0) {

                    std::ostringstream ss;
                    ss << latencyAv;
                    std::string latency_str(ss.str());
                    
//                    std::cout<<"Latencia média: "<<latencyAv<<std::endl;
//                    std::cout<<"Número de latencias: "<<latency_counter<<std::endl;
//                    char latency_char[1024];
//                    strncpy(latency_char, latency_str.c_str(), sizeof (latency_char));
//                    latency_char[sizeof (latency_char) - 1] = 0;

                    char commandFsmac[2];
                    commandFsmac[1] = 0x01;

                    //                char* request = (char*) malloc(sizeof (char)*1);
                    //                request[0] = 'Z';

                    char packRequest[256];
                    char addr_d[2];
                    addr_d[0] = addr_bc_1;
                    addr_d[1] = addr_bc_2;

                    char comm = 'L';
//                    printf("CSMA: Gerando comando de latência\n");
//                    generateControlFsmacPack(latency_char, (int) (ssize_t) latency_str.length(), comm, packRequest, EXCHANGE_COMMAND_SEND_INFO, addr_d);

                    std::stringstream ss2;
                    ss2 << latency_counter;
                    std::string latency_counter_str = ss2.str();

//                    char latency_counter_char[1024];
//                    strncpy(latency_counter_char, latency_counter_str.c_str(), sizeof (latency_counter_char));
//                    latency_counter_char[sizeof (latency_counter_char) - 1] = 0;

                    std::string couter_plus_av(latency_counter_str + "&" + latency_str);
                    int sizeCounterPlusAv = ((int) (ssize_t) latency_counter_str.length()) + ((int) (ssize_t) latency_str.length());

                    char couter_plus_av_char[1024];
                    strncpy(couter_plus_av_char, couter_plus_av.c_str(), sizeof (couter_plus_av_char));
                    couter_plus_av_char[sizeof (couter_plus_av_char) - 1] = 0;
                    
                    generateControlFsmacPack(couter_plus_av_char, sizeCounterPlusAv , comm, packRequest, EXCHANGE_COMMAND_SEND_INFO, addr_d);
                    
//                    pmt::pmt_t latency_counter_blob = pmt::make_blob(latency_counter_char, (int) (ssize_t) latency_counter_str.length());

                    //                    pmt::pmt_t packReq = pmt::cons(latency_counter_blob, pmt::make_blob(packRequest, control_pack_len));
                    pmt::pmt_t packReq = pmt::cons(pmt::get_PMT_NIL(), pmt::make_blob(packRequest, control_pack_len));

                    SendPackage* packageFsmac = new SendPackage(packReq, commandFsmac[1], false);

                    comm_ready = true;
                    commandList.push_back(packageFsmac);
                    data_ready = true;
                    cond.notify_all();
                }
            }
        } else if (pmt::is_dict(msg)) {
//            printf("CSMA: Recebeu fila para trocar protocolo.\n");

            //TODO receber a fila

            pmt::pmt_t exch_command = pmt::from_uint64(EXCHANGE_COMMAND_DONE);
            message_port_pub(pmt::mp("ctrl out"), exch_command);
            exchanging = false;

            exec = boost::shared_ptr<gr::thread::thread>
                    (new gr::thread::thread(boost::bind(&csma_impl::executeM, this)));
        }
    }

    /**
     * Função que trata as entradas da conexão "pdu in". 
     * Recebe mensagens que contêm pacotes MAC
     * @param msg Mensagem com pacotes MAC (Dados ou acks)
     */
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
            //  dout << "MAC: frame too short. Dropping! === " << std::endl;
            //            printPack((char*) pmt::blob_data(blob), data_len);
            return;
        }

        char* recPackage = (char*) pmt::blob_data(blob);
        recPackage[data_len - 1] = '\0';
        data_len = data_len - 1;

        recPackage = (char*) pmt::blob_data(blob);
        uint16_t crc = crc16(recPackage, data_len);

        //LOG std::cout << "Número de pacotes recebidos: " << d_num_packets_received << std::endl;
        if (crc) {
            d_num_packet_errors++;
//            std::cout << "MAC: wrong crc. Dropping packet!" << std::endl;
            if (d_debug) {
                printPack(recPackage, data_len);
            }
            return;
        } else {
            if (isAckPack(recPackage)) {
                //LOG printf("Package %u acked.\n\n", (unsigned char)recPackage[2]);
                removePackAcked(recPackage);
            } else if ((mac_addr_1 != recPackage[7] || mac_addr_2 != recPackage[8]) &&
                    (mac_addr_1 == recPackage[5] && mac_addr_2 == recPackage[6])) {
                //Verifica se o endereço de destino confere com o endereço MAC 
                //local, ou seja, "é ednereçado a mim" e se o endereço de origem 
                //é diferente do número MAC local, ou seja, "não foi enviado por
                //mim". Só trata o pacote se as duas condições forem verdadeiras.                

                d_num_packets_received++;
                //LOG dout << "MAC: correct crc. Propagate packet to APP layer." << std::endl;
                //LOG printf("Pacote recebido - ID: %u - Endereco de origem: %u%u\n", (unsigned char)recPackage[2], 
                //(unsigned char)recPackage[7], (unsigned char)recPackage[8]);

                pmt::pmt_t mac_payload = pmt::make_blob((char*) pmt::blob_data(blob) + 9, data_len - 9 - 2);
                message_port_pub(pmt::mp("app out"), pmt::cons(pmt::PMT_NIL, mac_payload));

                //Só envia pacote ack se a mensagem recebida não for de broadcast                
                if (addr_bc_1 != recPackage[5] || addr_bc_2 != recPackage[6]) {
                    char dAck[6];
                    generateAck(recPackage, dAck);
                    pmt::pmt_t packAck = pmt::cons(pmt::PMT_NIL, pmt::make_blob(dAck, 6));
                    SendPackage* packageAck = new SendPackage(packAck, recPackage[2], true);
                    send(packageAck);
                } else {
//                    printf("TIPO PACOTE: %x\n", recPackage[0] & 0xff);
                    printf("\n");
                }
            } else {
                //LOG printf("Pacote dropado. Mesmo endereço\n\n");
            }
        }

    }

    void printPack(char* pack, int data_len) {
        int j = 0;
        for (j = 0; j < data_len; j++) {
            printf("%x-", pack[j] & 0xff);
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

    void iniciaContagemLatencia() {
        gettimeofday(&myComT1, NULL);
    }

    void finalizaContagemLatencia() {
        gettimeofday(&myComT2, NULL);

        double timeCom = 0;

        timeCom = (myComT2.tv_sec - myComT1.tv_sec) * 1000.0; // sec to ms
        timeCom += (myComT2.tv_usec - myComT1.tv_usec) / 1000.0; // us to ms

        listaLatencias.push_back(timeCom);

        if (d_debug) {
            printf("Elapsed time: %f\n", timeCom);
        }
        
//        printf("Latencia normal: %f\n", timeCom);
    }
    
    /**
     * Inicia a contagem de tempo para medição periódica 
     * de latência como sensoriamento para decisão de troca.
     */
    void iniciaContagemLatenciaParaSensor() {
        gettimeofday(&myComT3, NULL);
    }
    
    /**
     * Finaliza a contagem de tempo para medição periódica 
     * de latência como sensoriamento para decisão de troca.
     */
    void finalizaContagemLatenciaParaSensor() {
        gettimeofday(&myComT4, NULL);

        double timeCom = 0;

        timeCom = (myComT4.tv_sec - myComT3.tv_sec) * 1000.0; // sec to ms
        timeCom += (myComT4.tv_usec - myComT3.tv_usec) / 1000.0; // us to ms

        latencyListForSensor.push_back(timeCom);
        if (d_debug) {
            printf("Elapsed time sensor: %f\n", timeCom);
        }
        
//        printf("Latencia para sensor: %f\n", timeCom);
    }

    bool isAckPack(char* recPack) {
        bool isA = recPack[0] == 0x02;
        return isA;
    }

    /**
     * Marca o pacote como confirmado e diz que ele pode ser removido. Não 
     * remove o pacote da fila diretamente para evitar problemas de 
     * produtor/consumidor.
     */
    void removePackAcked(char* ackPack) {
        unsigned char packId;
        packId = ackPack[2];

        if (!sendList.empty()) {
            std::list<SendPackage*>::iterator it = sendList.begin();
            //        while (it != sendList.end()) {
            if ((*it)->getId() == packId && (*it)->getCanRemove() == false) {
                (*it)->setCanRemove(true);
                lastPackAckedOrTimeToResendFinished = true;

                //                waitingAck = false;
                
                finalizaContagemLatenciaParaSensor();
                finalizaContagemLatencia();

                if (canCompute) {
                    numPackConfirmados++;
                }

                cond2.notify_all();

                if (d_debug) {
                    printf("Package %u acked.\n\n", (unsigned char) ackPack[2]);
                }
                //                break;
            }
            //            it++;
            //        }
        }


    }

    /**
     * Função que trata a entrada da conexção "app in".
     */
    void app_in(pmt::pmt_t msg) {
        if (!exchanging) {
            pmt::pmt_t blob;
            if (pmt::is_eof_object(msg)) {
                // dout << "MAC: exiting" << std::endl;
                detail().get()->set_done(true);
                return;
            } else if (pmt::is_blob(msg)) {
                blob = msg;
            } else if (pmt::is_pair(msg)) {
                blob = pmt::cdr(msg);
            } else {
                // dout << "MAC: unknown input" << std::endl;
                return;
            }
            //            printf("CSMA: Recebeu pacote\n");
            //LOG printf("Preparando pacote para o endereco: %u%u\n\n", addr0[0], addr0[1]);

            //Neste caso, todos os pacotes estão sendo enviados para o endereço 
            //addr0, para melhorar os testes, este endereço poderia ser escolhido 
            //aleatoriamente entre os endereços disponíveis.
            generate_mac((const char*) pmt::blob_data(blob), pmt::blob_length(blob), addrs[indexDestNode]);
            pmt::pmt_t pack = pmt::cons(pmt::PMT_NIL, pmt::make_blob(d_msg, d_msg_len));
            SendPackage* package = new SendPackage(pack, d_msg[2], false);
            package->setTime(0);
            send(package);
        }
    }

    /**
     * Esta função inicia o processo de envio da mensagem. Ela coloca o pacote 
     * na fila para envio, notifica a thread principal que há dados prontos para 
     * serem enviados.
     * 
     * A função que implementa essa espera é a executeM().
     */
    void send(SendPackage* pack) {
        //        printf("CSMA: Colocou pacote na fila\n");
        //        pmt::print(pack->getPackage());
        if (pack->hasAckPackage()) {
            sendAckPackage(pack);
        } else if (sendList.size() < 100) {
            sendList.push_back(pack);
            //            mylist.push_back(pack);
            //            printf("Tamanho da my liste %d\n", mylist.size());
            //            printf("Tamanho da fila: %d\n", sendList.size());
        }

        data_ready = true;
        cond.notify_all();
    }

    /**
     * Envia confirmação dos pacotes recebidos. Diferente dos outros pacotes, 
     * o ack não é colocado em fila de envio, é enviado diretamente.
     */
    void sendAckPackage(SendPackage* pack) {
        //LOG std::cout << "Waiting Sifs: " << sifs << std::endl;
        boost::posix_time::millisec workTime(sifs);
        boost::this_thread::sleep(workTime);

        message_port_pub(pmt::mp("pdu out"), pack->getPackage());
        //LOG printf("Enviando ack ID: %u\n\n", pack->getId());

    }

    void waitToCompute() {
        boost::posix_time::millisec workTimeAloc(timeToStart);
        boost::this_thread::sleep(workTimeAloc);

        canCompute = true;
        
        listaLatencias.clear();
    }

    void calculateLatencyAv() {
        float sum = 0.0;
        latency_counter = 0;
        long unsigned int tam_lista = latencyListForSensor.size();
        
//        std::list<double>::iterator it = latencyListForSensor.end();
//        while (it != latencyListForSensor.begin()) {
//            sum = sum + (*it);
//            latency_counter++;
//            it--;
//        }
        
        int index = 0;
        std::list<double>::iterator it = latencyListForSensor.begin();
        while (index < tam_lista) {
            sum = sum + (*it);
            latency_counter++;
            it++;
            index++;
        }

        latencyAv = sum / latency_counter;
        latencyListForSensor.clear();
    }

    /**
     * Esta função sobrescreve a função nativa do GNU Radio que ativa o bloco. 
     * Foi necessário usá-la para iniciar a thread principal. 
     */
    bool start() {
        //Cria a thread principal de gerenciamento. Esta thread é criada para 
        //evitar usar a thread (realmente) principal do bloco. Durante a 
        //execução ela precisa ser pausada e retomada várias vezes, quando 
        //fazemos isso com a thread do bloco, ela interrompe outras operações 
        //como o carrier sense que é feito na função cs_in().
        exec = boost::shared_ptr<gr::thread::thread>
                (new gr::thread::thread(boost::bind(&csma_impl::executeM, this)));

        //Estes endereços estão sendo alocados manualmente para testes. Em um
        //cenário onde a rede funcione de forma completa, estes endereços são 
        //fornecidos por um nó de coordenação, que informa de tempos em tempos
        //quais são os nós presentes na rede.
        addr0[0] = 0x40;
        addr0[1] = 0xe8;

        addr1[0] = 0x41;
        addr1[1] = 0xe8;

        addr2[0] = 0x42;
        addr2[1] = 0xe8;

        addr3[0] = 0x43;
        addr3[1] = 0xe8;

        addr4[0] = 0x44;
        addr4[1] = 0xe8;

        addr5[0] = 0x45;
        addr5[1] = 0xe8;

        addr6[0] = 0x46;
        addr6[1] = 0xe8;

        addrs[0] = addr0;
        addrs[1] = addr1;
        addrs[2] = addr2;
        addrs[3] = addr3;
	addrs[4] = addr4;
        addrs[5] = addr5;
        addrs[6] = addr6;

        mac_addr_1 = addrs[indexLocalMac][0]; //mac_addr[0];
        mac_addr_2 = addrs[indexLocalMac][1]; //mac_addr[1];

        gettimeofday(&t1, NULL);

        return block::start();
    }
    

    /**
     * Esta função sobrescreve a função nativa do GNU Radio que ativa o bloco. 
     * ela garante que a thread principal será fechada apenas quando o bloco for 
     * desativado.
     */
    bool stop() {
          exchanging = true;
          exec->interrupt();
//        exec->join();

  //      waitToComputeTb->join();
        waitToComputeTb->interrupt();

        if (numPackEnviados != 0) {
            
            boost::posix_time::millisec workTime(2000);
            boost::this_thread::sleep(workTime);

            gettimeofday(&t2, NULL);

            // compute and print the elapsed time in millisec
            elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0; // sec to ms
            elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0; // us to ms
            long int finalTime = elapsedTime - timeToStart;
            printf("\n");
            std::cout << "RESULTADOS_CSMA:" << std::endl;
            std::cout << "Tempo:" << finalTime << ":ms" << std::endl;
            std::cout << "Enviados:" << numPackEnviados << std::endl;
            std::cout << "Confirmados:" << numPackConfirmados << std::endl;
            std::cout << "Retransmissoes: " << numRetransmissoes << std::endl;
            std::cout << "pacotesFsmac: " << num_fsmac_packs << std::endl;
            std::cout << "VazaoPs: " << numPackConfirmados / (finalTime / 1000) << " pacotes/s" << std::endl;
            std::cout << "VazaoBs:" << numPackConfirmados * 110 / (finalTime / 1000) << " bytes/s" << std::endl;
            std::cout << "Taxa:" << double(numPackConfirmados) / numPackEnviados * 100 << " porcento" << std::endl;
            std::cout << "Latencias:"; // << finalTime/numPackConfirmados << " ms" << std::endl;

            std::list<double>::iterator it = listaLatencias.begin();
            while (it != listaLatencias.end()) {
                std::cout << (*it) << ";";
                it++;
            }
        }        

        return block::stop();
    }

    bool canExecute() {
        return data_ready;
    }

    /**
     * Função chamada pela thread principal, a partir da qual todo o 
     * gerenciamento acontece. Ela espera que os dados fiquem prontos, quando é 
     * notificada de que há dados a enviar, envia todos os dados e volta ao 
     * modo de espera.
     * 
     * A função que faz notificação de dados prontos é a função send().
     */
    void executeM() {
        if (d_debug) {
            timeToStart = 1000;
        }

        //        printf("CSMA: Entrou no executeM\n");

        waitToComputeTb = boost::shared_ptr<gr::thread::thread>
                (new gr::thread::thread(boost::bind(&csma_impl::waitToCompute, this)));

        boost::unique_lock<boost::mutex> lock(mut);
        while (true && !exchanging) {

            while (!data_ready) {
                //std::cout << "Loop" << std::endl;
                cond.wait(lock);
            }

            //Essa atribuição é necessária nesse ponto para evitar que dados 
            //fiquem perdidos na fila por problemas de sincronização. Essa 
            //variável é usada também pela função send() para notificar se há 
            //dados a serem enviados.
            data_ready = false;
            if (!exchanging) {
                runSending();
            }

        }
    }

    /**
     * Envia de fato os pacotes e incrementa o número de reenvios.
     */
    bool sendPackageNow(SendPackage* pack) {
        pmt::pmt_t pmt_pack = pack->getPackage();
        //        pmt::print(pmt_pack);
        struct timeval now;
        message_port_pub(pmt::mp("pdu out"), pmt_pack);

//        if (!comm_ready) {
//            iniciaContagemLatenciaParaSensor();
//        }


        if (!comm_ready) {
            if (pack->getResends() == 0) {
                if (canCompute) {
                    numPackEnviados++;
                }
                iniciaContagemLatencia();
                iniciaContagemLatenciaParaSensor();
            } else {
                if(canCompute){
                    numRetransmissoes++;                    
                }
            }
        } else if (comm_ready) {
            if (canCompute) {
                num_fsmac_packs++;
            }
        }
        

        lastPackAckedOrTimeToResendFinished = false;
        if (d_debug) {
            printf("CSMA: Enviou pacote dados %u\n", pack->getId());
        }
        gettimeofday(&now, NULL);
        pack->setTime((now.tv_sec * 1000) + (now.tv_usec / 1000));

        pack->increaseResends(max_retr);

        return true;
    }

    /**
     * Funçao que gerencia toso as operaçoes enquanto houver dados para serem 
     * enviados.
     */
    bool runSending() {
        while ((!sendList.empty() || !commandList.empty()) && !exchanging) {
//            printf("CSMA: Entrou no PRIMEIRO while\n");
            std::list<SendPackage*>::iterator it = sendList.begin();
            while ((it != sendList.end() || !commandList.empty()) && !exchanging) {
                it = sendList.begin();
//                printf("CSMA: Entrou no SEGUNDO while\n");
                //Remove os pacotes confirmados e que excederam a quantidade de reenvios
                if (!sendList.empty() && (*it)->getCanRemove()) {
                    SendPackage* packToRemove = *it;
                    it++;
                    sendList.remove(packToRemove);
                    cw_current_backoff = cw_backoff_min;
                } else {
//                    std::cout << "Waiting Diff: " << difs << std::endl;
                    boost::posix_time::millisec workTimeDifs(difs);
                    boost::this_thread::sleep(workTimeDifs);

//                    printf("INICIO da verificacao de backoff\n");
                    if (!sendList.empty() && (*it)->getResends() > 0) {
                        cw_current_backoff = cw_current_backoff * 2;
                    }
//                    printf("FIM da verificacao de backoff\n");

                    real_backoff = (std::rand() % cw_current_backoff) + 1;
                    boost::posix_time::millisec workTime(slotSize);

//                    printf("INICIO da espera de backoff\n");
                    //tratamento do backoff
//                    std::cout << "Real backoff inicial: " << real_backoff << std::endl;
                    while (real_backoff >= 0) {
//                        printf("INICIO SLEEP\n");
                        boost::this_thread::sleep(workTime);
//                        printf("FIM SLEEP\n");
//                        std::cout << "POTENCIA DO MEIO: " <<lastAvPower << std::endl;
                        if (!isChannelBusy(referenceValueChannelBusy)) {
                            real_backoff = real_backoff - slotSize;
//                            std::cout << "Real backoff: " << real_backoff << std::endl;
                        }
                    }
//                    printf("FIM da espera de backoff\n");

                    if (!commandList.empty()) {
//                        printf("Envio de comando %d vez.\n", commandList.front()->getResends());
                        sendPackageNow(*(commandList.begin()));
                        //                        pmt::print((*(commandList.begin()))->getPackage());
                        //printPackChar(d_com, 12);

                        if (commandList.front()->getResends() == 2) {
                            comm_ready = false;
                            commandList.pop_front();
                           
                            if(answer_sending){
                                pmt::pmt_t exch_command = pmt::from_uint64(EXCHANGE_COMMAND_INFO_SENT);
                                message_port_pub(pmt::mp("ctrl out"), exch_command);                                
                            }
                            
                            answer_sending = false;                            
//                            printf("Comando enviado com sucesso. Vida que segue.\n");
                        }

                    } else {
                        sendPackageNow(*it);
                        //                    }

                        //epera para reenvio
                        waitSending = boost::shared_ptr<gr::thread::thread>
                                (new gr::thread::thread(boost::bind(&csma_impl::waitResendingTime, this)));

                        boost::unique_lock<boost::mutex> lock2(mut2);

                        // a variável lastPackAckedOrTimeToResendFinished é 
                        //manipulada em duas funções diferentes para garantir que 
                        //essa espera durará até o recebimento da confirmação do 
                        //pacote ou até o fim do tempo de espera para reenvio, o 
                        //que acontecer primeiro. 
                        while (!lastPackAckedOrTimeToResendFinished) {
                            cond2.wait(lock2);
                            //                        printf("Loop de confirmacao\n");
                        }

                        lock2.unlock();
                        waitSending->interrupt();
                        //                    printf("Saiu do Loop de confirmacao\n");
                        //                    waitSending->interrupt();
                        //                    waitSending->join();
                    }
                }

            }

        }

        return true;
    }

    /**
     * Função que faz a espera para reenvio.
     */
    void waitResendingTime() {
        boost::posix_time::millisec workTimeResend(resend_waiting);
        boost::this_thread::sleep(workTimeResend);
        if (d_debug) {
            printf("Acabou a espera de timeout\n");
        }
        lastPackAckedOrTimeToResendFinished = true;
        cond2.notify_all();
    }

    /**
     * Faz o carrier sense. 
     * Com intervalo de tempo definido no parêmetro do bloco gráfico de 
     * eventstream, faz a leitura do ambiente, verificando qual a potência do 
     * sinal em uma certa frequência definida. Guarda esse valor em uma variável 
     * que será usada internamente para decisões de canal livre ou ocupado.
     * 
     * É importante ter atenção ao manipular threads nesse arquivo, pois 
     * frequentemente quando se interrompe uma thread, um efeito colateral é 
     * interromper a execução dessa função.
     */
    void cs_in(pmt::pmt_t msg) {
        pmt::pmt_t blob;

        if (pmt::is_blob(msg)) {
            blob = msg;
            std::cout << "Is blob" << std::endl;
        } else if (pmt::is_pair(msg)) {
            blob = pmt::car(msg);
            //LOG std::cout << "Is pair" << std::endl;
        }

        float avPowerChannel = 0;
        float power = 0;

        //In this case, the blob is a dictionary, we are getting the value of power using the key "power"
        pmt::pmt_t pmtPower = pmt::dict_ref(blob, pmt::string_to_symbol("power"), pmt::get_PMT_NIL());
        power = pmt::to_float(pmtPower);

        //LOG std::cout << "Power: ";

        //LOG std::cout << power << std::endl;

        avPowerChannel = power;

        lastAvPower = avPowerChannel;
    }

    /**
     * Usa o valor de referência para determinar se o canal está livre.
     */
    bool isChannelBusy(float refValue) {
        return (lastAvPower > refValue);
    }

    /**
     * Função da implementação original. Funciona bem, mas tem um problema de 
     * retorno, quando o crc falha, retorna true, no caso de sucesso, retorna 
     * false.
     * @param buf
     * @param len
     * @return 
     */
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

    /**
     * Gera os pacotes de confirmação. Implementada por mim, está em acordo com 
     * o 802.15.4.
     * @param buf Usado apenas para verificação do id do pacote
     * @param dAck o pacote ack a ser configurado
     * @return Retorna o pacote ack configurado
     */
    char* generateAck(const char *buf, char* dAck) {
        unsigned char packId;
        packId = (unsigned char) buf[2];

        // ack frame: type 010
        dAck[0] = 0x02;
        dAck[1] = 0x00;

        // seq nr
        dAck[2] = packId;

        uint16_t crc = crc16(dAck, 3);

        dAck[3] = crc & 0xFF;
        dAck[4] = crc >> 8;
        dAck[5] = 0x7f;

        return dAck;
    }

    void generateControlFsmacPack(char *buf, int bufSize, char new_prot, char* pack, int type, char* addr_dest) {
        int len = 0;
        char packFunc[1];

        int i = 0;

        if (type == EXCHANGE_COMMAND_SEND_INFO) {
            packFunc[0] = new_prot;
        }

        char payload[bufSize + 1];
        payload[0] = packFunc[0];

        for (i = 0; i < bufSize; i++) {
            payload[i + 1] = buf[i];
        }

        if (bufSize > 0) {
            std::memcpy(payload + 1, buf, strlen(buf));
        }

        len = bufSize + 1; //payload size
        unsigned char packId;
        packId = d_seq_nr;

        //        if(type == TYPE_BEACON_SYNC){
        //            printf("Pacote id, len: %c, %d\n", packId, len);
        //        }

        pack[0] = 0x41;
        pack[1] = 0x88;

        // seq nr
        pack[2] = packId;

        pack[3] = 0xcd;
        pack[4] = 0xab;
        pack[5] = 0xff;
        pack[6] = 0xff;
        pack[7] = mac_addr_1;
        pack[8] = mac_addr_2;

        std::memcpy(pack + 9, payload, len);

        uint16_t crc = crc16(pack, len + 9);

        pack[ 9 + len] = crc & 0xFF;
        pack[10 + len] = crc >> 8;
        pack[11 + len] = 0x7f;

        control_pack_len = 9 + len + 3;

    }

    char* generateFsmacPack(const char *buf) {//, char* dFsmac) {
        unsigned char packId;
        packId = (unsigned char) buf[1];

        // ack frame: type 010
        d_com[0] = 0x41;
        d_com[1] = 0x88;

        // seq nr
        d_com[2] = packId;

        d_com[3] = 0xcd;
        d_com[4] = 0xab;
        d_com[5] = 0xff;
        d_com[6] = 0xff;
        d_com[7] = mac_addr_1;
        d_com[8] = mac_addr_2;


        uint16_t crc = crc16(d_com, 9);

        d_com[9] = crc & 0xFF;
        d_com[10] = crc >> 8;
        d_com[11] = 0x7f;

        return d_com;
    }

    /**
     * Gera os pacotes de dados.
     * Essa função é da implementação original, melhorei ela um pouco, 
     * mas precisa ser muito melhorada ainda. Os autores dizem que o pacote 
     * está em acordo com o padrão 802.15.4. Confiei neles, conferi 
     * tudo exceto o subfild de controle.
     */
    void generate_mac(const char *buf, int len, char* addr_dest) {

        // FCF
        // data frame, no security
        d_msg[0] = 0x61;
        d_msg[1] = 0x88;

        // seq nr
        d_msg[2] = d_seq_nr++;

        // addr info
        d_msg[3] = 0xcd;
        d_msg[4] = 0xab;
        d_msg[5] = addr_dest[0];
        d_msg[6] = addr_dest[1];
        d_msg[7] = mac_addr_1;
        d_msg[8] = mac_addr_2;

        std::memcpy(d_msg + 9, buf, len);

        uint16_t crc = crc16(d_msg, len + 9);

        d_msg[ 9 + len] = crc & 0xFF;
        d_msg[10 + len] = crc >> 8;
        d_msg[11 + len] = 0x7f;

        d_msg_len = 9 + len + 3;
    }

    //    void print_message() {
    //        for (int i = 0; i < d_msg_len; i++) {
    //            dout << std::setfill('0') << std::setw(2) << std::hex << ((unsigned int) d_msg[i] & 0xFF) << std::dec << " ";
    //            if (i % 16 == 15) {
    //                dout << std::endl;
    //            }
    //        }
    //        dout << std::endl;
    //    }

    int get_num_packet_errors() {
        return d_num_packet_errors;
    }

    int get_num_packets_received() {
        return d_num_packets_received;
    }

    float get_packet_error_ratio() {
        return float(d_num_packet_errors) / d_num_packets_received;
    }

private:
    bool d_debug;
    int d_msg_offset;
    int d_msg_len;
    uint8_t d_seq_nr;
    char d_msg[256];
    char d_com[256];
    int indexDestNode;
    int indexLocalMac;
    //    uint8_t d_mac_addr[2];

    int d_num_packet_errors;
    int d_num_packets_received;
};

csma::sptr
csma::make(int mac_addr, int dest_node, bool debug) {
    return gnuradio::get_initial_sptr(new csma_impl(mac_addr, dest_node, debug));
}
