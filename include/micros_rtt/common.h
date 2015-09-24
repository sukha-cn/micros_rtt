/* 
 *  common.h - micros common lib
 *  Copyright (C) 2015 Zaile Jiang
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#ifndef MICROSRTT_COMMON_H
#define MICROSRTT_COMMON_H

#include <list>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace micros_rtt
{

enum FlowStatus {NoData = 0, OldData = 1, NewData = 2};

}

#endif  /* COMMON_H */

