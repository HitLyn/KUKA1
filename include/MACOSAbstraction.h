//  ---------------------- Doxygen info ----------------------
//! \file MACOSAbstraction.h
//!
//! \brief
//! Header file for simple OS-specific functions for abstraction (MacOS)
//!
//! \details
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see 
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date October 2013
//!
//! \version 1.0.1
//!
//!	\author Torsten Kroeger, tkr@stanford.edu
//!
//!
//! \note Copyright (C) 2013 Stanford University.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __MACOSAbstraction__
#define __MACOSAbstraction__

#include <unistd.h>
#include <string.h>
#include <errno.h>

#ifndef  EOK
#define EOK				0
#endif

#ifndef  ETIME
#define ETIME			62
#endif

#ifndef  ENOTCONN
#define ENOTCONN		107
#endif

#ifndef  EALREADY
#define EALREADY		114
#endif

void delay(const int &TimeInMilliseconds);


int stricmp(const char *s1, const char *s2);





#endif
