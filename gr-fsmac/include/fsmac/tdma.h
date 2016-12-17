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


#ifndef INCLUDED_FSMAC_TDMA_H
#define INCLUDED_FSMAC_TDMA_H

#include <fsmac/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace fsmac {

    /*!
     * \brief <+description of block+>
     * \ingroup fsmac
     *
     */
    class FSMAC_API tdma : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<tdma> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of fsmac::tdma.
       *
       * To avoid accidental use of raw pointers, fsmac::tdma's
       * constructor is in a private implementation
       * class. fsmac::tdma::make is the public interface for
       * creating new instances.
       */
      static sptr make(int mac_addr, int dest_node, bool debug= false, bool is_coord= false);
    };

  } // namespace fsmac
} // namespace gr

#endif /* INCLUDED_FSMAC_TDMA_H */

