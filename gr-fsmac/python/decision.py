#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2016 <+YOU OR YOUR COMPANY+>.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy
from gnuradio import gr
import pmt
import numpy as np
import time
import thread

SECONDS = 5


class decision(gr.sync_block):
    """
    docstring for block decision
    """
    def __init__(self, is_coord):
        gr.sync_block.__init__(self,
                               name="decision",
                               in_sig=None, out_sig=None)

        self.is_coordinator = is_coord

        self.terminate = False

        # self.tdma_low_pert_decimals = [10,20,30,40,50,60]
        # self.tdma_hight_pert_decimals = [50,60,70,80,90,100]
        #
        # self.csma_low_pert_decimals = [10,20,30,40,50,60]
        # self.csma_hight_pert_decimals = [50,60,70,80,90,100]

        self.tdma_low_pert_decimals = [0,10,20,30,40,50,60]
        self.tdma_hight_pert_decimals = [40,50,60,70,80,90,100]

        self.csma_low_pert_decimals = [0,10,20,30,40,50,60]
        self.csma_hight_pert_decimals = [40,50,60,70,80,90,100]

        self.sens1_value = 1;
        self.sens2_value = 100;
        self.active_protocol = 1;

        self.message_port_register_out(pmt.intern('troca out'))
        self.message_port_register_in(pmt.intern('sens1 in'))
        self.message_port_register_in(pmt.intern('sens2 in'))
        self.message_port_register_in(pmt.intern('troca in'))

        self.set_msg_handler(pmt.intern("sens1 in"), self.handle_sens1)
        self.set_msg_handler(pmt.intern("sens2 in"), self.handle_sens2)
        self.set_msg_handler(pmt.intern("troca in"), self.handle_troca)
        self.main_loop()

    #    def work(self, input_items, output_items):
    #        in0 = input_items[0]
    #        out = output_items[0]
    #        # <+signal processing here+>
    #        out[:] = in0
    #        return len(output_items[0])

    def print_time (self, nomedothread, delay):
        while not self.terminate:
            time.sleep(delay)
            csma_adapt = float(self.calculate_csma_adaptability());
            tdma_adapt = float(self.calculate_tdma_adaptability());
            # csma_adapt = 10.0;
            # tdma_adapt = 90.0;
	    print ""
            print "NS: "+str(self.sens1_value)

	    if self.active_protocol == 1:
		print "ACTIVE PROTOCOL: CSMA"
   	    elif self.active_protocol == 2:
		print "ACTIVE PROTOCOL: TDMA"
					
            # print "CSMA adaptability " + str(csma_adapt) + "%"
            # print "TDMA adaptability " + str(tdma_adapt) + "%"            
            # print "LM: "+str(self.sens2_value)

            pmt_dict_prot_adapt = pmt.make_dict()

            if csma_adapt > tdma_adapt:
                pmt_dict_prot_adapt = pmt.dict_add(pmt_dict_prot_adapt, pmt.from_uint64(1), pmt.cons(pmt.from_uint64(1), pmt.from_double(csma_adapt)))
                pmt_dict_prot_adapt = pmt.dict_add(pmt_dict_prot_adapt, pmt.from_uint64(2), pmt.cons(pmt.from_uint64(2), pmt.from_double(tdma_adapt)))
            else:
                pmt_dict_prot_adapt = pmt.dict_add(pmt_dict_prot_adapt, pmt.from_uint64(1), pmt.cons(pmt.from_uint64(2), pmt.from_double(tdma_adapt)))
                pmt_dict_prot_adapt = pmt.dict_add(pmt_dict_prot_adapt, pmt.from_uint64(2), pmt.cons(pmt.from_uint64(1), pmt.from_double(csma_adapt)))

            self.message_port_pub(pmt.intern('troca out'), pmt_dict_prot_adapt)

    def main_loop(self):
        if self.is_coordinator:
            try:
                thread.start_new_thread(self.print_time, ("Thread-1", SECONDS,))
            except:
                print "Erro: nao foi possivel iniciar o thread."

    def handle_sens1(self,msg):
        self.sens1_value = pmt.to_uint64(msg);

    def handle_sens2(self,msg):
        if self.is_coordinator:
            self.sens2_value = pmt.to_float(msg);
            # print "Valor atual da latencia: " + str(self.sens2_value) + " ms"
        else:
            # print "Envindo latencia para exchanger"
            self.message_port_pub(pmt.intern('troca out'), msg)
            if self.active_protocol == 1:
		print ""
		print "ACTIVE PROTOCOL: CSMA"
	    elif self.active_protocol == 2:
		print ""
		print "ACTIVE PROTOCOL: TDMA"

    def handle_troca(self,msg):
        # print "DEC: Recebeu comunicado"
        self.active_protocol = pmt.to_uint64(msg);
        # print "NEW ACTIVE PROTOCOL: "+str(self.active_protocol)

    def calculate_fuzzy(self):
        csma_adapt = self.calculate_csma_adaptability()
        tdma_adapt = self.calculate_tdma_adaptability()

        return [csma_adapt, tdma_adapt]

    def calculate_tdma_adaptability(self):
        #if senders is higth AND data is higth then TDMA adaptability is hight
        #if senders is low AND data is higth then TDMA adaptability is low

        #======= FUZZYFICATION PHASE =======#
        senders_fuzy_pert = self.senders_function(self.sens1_value)
        data_fuzzy_pert = self.data_function(self.sens2_value)
        #===================================#

        numerator = 0
        denominator = 0

        value_hight_adapt = 0
        value_low_adapt = 0

        list_low_adapts = []
        list_hight_adapts = []

        #======= EVALUATION OF RULES =======#
        #if senders is higth AND data is higth then TDMA adaptability is hight
        #Intersection gets minimum between two values
        if senders_fuzy_pert[1] < data_fuzzy_pert[1]:
            list_hight_adapts.append(senders_fuzy_pert[1])
        else:
            list_hight_adapts.append(data_fuzzy_pert[1])

        #if senders is low AND data is higth then TDMA adaptability is low
        #Intersection gets minimum between two values
        if senders_fuzy_pert[0] < data_fuzzy_pert[1]:
            list_low_adapts.append(senders_fuzy_pert[0])
        else:
            list_low_adapts.append(data_fuzzy_pert[1])

        #if senders is higth AND data is low then TDMA adaptability is low
        #Intersection gets minimum between two values
        if senders_fuzy_pert[1] < data_fuzzy_pert[0]:
            list_low_adapts.append(senders_fuzy_pert[1])
        else:
            list_low_adapts.append(data_fuzzy_pert[0])

        #if senders is low AND data is low then TDMA adaptability is low
        #Intersection gets minimum between two values
        if senders_fuzy_pert[0] < data_fuzzy_pert[0]:
            list_low_adapts.append(senders_fuzy_pert[0])
        else:
            list_low_adapts.append(data_fuzzy_pert[0])

        list_low_adapts.sort(reverse=True)
        list_hight_adapts.sort(reverse=True)

        value_low_adapt = list_low_adapts[0]
        value_hight_adapt = list_hight_adapts[0]
        #===================================#


        #====== DEFUZZYFICATION PHASE ======#
        for i in self.tdma_hight_pert_decimals:
            numerator = numerator + i*value_hight_adapt

        for j in self.tdma_low_pert_decimals:
            numerator = numerator + j*value_low_adapt

        denominator = denominator + len(self.tdma_hight_pert_decimals)*value_hight_adapt
        denominator = denominator + len(self.tdma_low_pert_decimals)*value_low_adapt

        if denominator == 0:
            denominator = 1

        adaptability_degree = numerator/denominator
        #===================================#

        return adaptability_degree


    def calculate_csma_adaptability(self):
        #if senders is higth AND data is higth then CSMA adaptability is low
        #if senders is low AND data is higth then CSMA adaptability is higth

        #======= FUZZYFICATION PHASE =======#
        senders_fuzy_pert = self.senders_function(self.sens1_value)
        data_fuzzy_pert = self.data_function(self.sens2_value)
        #===================================#

        numerator = 0
        denominator = 0

        value_hight_adapt = 0
        value_low_adapt = 0

        list_low_adapts = []
        list_hight_adapts = []

        #======= EVALUATION OF RULES =======#
        #if senders is higth AND data is higth then CSMA adaptability is low
        #Intersection gets minimum between two values
        if senders_fuzy_pert[1] < data_fuzzy_pert[1]:
            list_low_adapts.append(senders_fuzy_pert[1])
        else:
            list_low_adapts.append(data_fuzzy_pert[1])

        #if senders is low AND data is higth then CSMA adaptability is higth
        #Intersection gets minimum between two values
        if senders_fuzy_pert[0] < data_fuzzy_pert[1]:
            list_hight_adapts.append(senders_fuzy_pert[0])
        else:
            list_hight_adapts.append(data_fuzzy_pert[1])
        #===================================#

        #if senders is hight AND data is low then CSMA adaptability is higth
        if senders_fuzy_pert[1] < data_fuzzy_pert[0]:
            list_hight_adapts.append(senders_fuzy_pert[1])
        else:
            list_hight_adapts.append(data_fuzzy_pert[0])

        #if senders is low AND data is low then CSMA adaptability is hight
        if senders_fuzy_pert[0] < data_fuzzy_pert[0]:
            list_hight_adapts.append(senders_fuzy_pert[0])
        else:
            list_hight_adapts.append(data_fuzzy_pert[0])


        list_hight_adapts.sort(reverse=True)
        list_low_adapts.sort(reverse=True)

        value_hight_adapt = list_hight_adapts[0]
        value_low_adapt = list_low_adapts[0]


        #====== DEFUZZYFICATION PHASE ======#
        for i in self.csma_hight_pert_decimals:
            numerator = numerator + i*value_hight_adapt

        for j in self.csma_low_pert_decimals:
            numerator = numerator + j*value_low_adapt

        denominator = denominator + len(self.csma_hight_pert_decimals)*value_hight_adapt
        denominator = denominator + len(self.csma_low_pert_decimals)*value_low_adapt

        if denominator == 0:
            denominator = 1

        adaptability_degree = numerator/denominator
        #===================================#

        return adaptability_degree

    def senders_function(self, x):
        low_pert = 0;
        hight_pert = 0;

	if x >= 0 and x <= 1:
	  low_pert = 100
	elif x > 1 and x < 2:
	  low_pert = -100*x  + 200
	else:
	  low_pert = 0
        
	if x >= 0 and x <= 1:
	  hight_pert = 0
	elif x > 1 and x < 2:
	  hight_pert = 100*x - 100
	else:
	  hight_pert = 100

        #if x >= 0 and x <= 2:
        #    low_pert = 100
        #elif x > 2 and x < 3:
        #    low_pert = -100*x + 300
        #else:
        #    low_pert = 0
		#
        #if x >= 0 and x <= 2:
        #    hight_pert = 0
        #elif x > 2 and x < 3:
        #    hight_pert = 100*x - 200
        #else:
        #    hight_pert = 100

        return [low_pert, hight_pert]

    def data_function(self, x):
        low_pert = 0;
        hight_pert = 0;

        if self.active_protocol == 1:
            if x >= 0 and x <= 20:
                low_pert = 100
            elif x > 20 and x < 40:
                low_pert = -5*x  + 200
            else:
                low_pert = 0
        
            if x >= 0 and x <= 20:
                hight_pert = 0
            elif x > 20 and x < 40:
                hight_pert = 5*x - 100
            else:
                hight_pert = 100
        
        elif self.active_protocol == 2:
            if x >= 0 and x <= 20:
                low_pert = 100
            elif x > 20 and x < 25:
                low_pert = -20*x  + 500
            else:
                low_pert = 0
        
            if x >= 0 and x <= 20:
                hight_pert = 0
            elif x > 20 and x < 25:
                hight_pert = 20*x - 400
            else:
                hight_pert = 100

        #if self.active_protocol == 1:
        #    if x >= 0 and x <= 35:
        #        low_pert = 100
        #    elif x > 35 and x < 55:
        #        low_pert = -5*x  + 275
        #    else:
        #        low_pert = 0
		#
        #    if x >= 0 and x <= 35:
        #        hight_pert = 0
        #    elif x > 35 and x < 55:
        #        hight_pert = 5*x - 175
        #    else:
        #        hight_pert = 100
		#
        #elif self.active_protocol == 2:
        #    if x >= 0 and x <= 23:
        #        low_pert = 100
        #    elif x > 23 and x < 26:
        #        low_pert = -33.33334*x  + 866.6667
        #    else:
        #        low_pert = 0
		#
        #    if x >= 0 and x <= 23:
        #        hight_pert = 0
        #    elif x > 23 and x < 26:
        #        hight_pert = 33.33334*x - 766.6667
        #    else:
        #        hight_pert = 100

        return [low_pert, hight_pert]
