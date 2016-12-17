/* -*- c++ -*- */

#define FSMAC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "fsmac_swig_doc.i"

%{
#include "fsmac/csma.h"
#include "fsmac/tdma.h"
#include "fsmac/sens_num_senders.h"
#include "fsmac/exchanger.h"
#include "fsmac/latency_sensor.h"
%}


%include "fsmac/csma.h"
GR_SWIG_BLOCK_MAGIC2(fsmac, csma);
%include "fsmac/tdma.h"
GR_SWIG_BLOCK_MAGIC2(fsmac, tdma);
%include "fsmac/sens_num_senders.h"
GR_SWIG_BLOCK_MAGIC2(fsmac, sens_num_senders);
%include "fsmac/exchanger.h"
GR_SWIG_BLOCK_MAGIC2(fsmac, exchanger);
%include "fsmac/latency_sensor.h"
GR_SWIG_BLOCK_MAGIC2(fsmac, latency_sensor);
