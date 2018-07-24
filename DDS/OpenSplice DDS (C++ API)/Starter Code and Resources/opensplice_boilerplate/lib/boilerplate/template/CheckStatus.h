
/*
 *                         OpenSplice DDS
 *
 *   This software and documentation are Copyright 2006 to  PrismTech
 *   Limited, its affiliated companies and licensors. All rights reserved.
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

/************************************************************************
 * LOGICAL_NAME:    CheckStatus.h
 * FUNCTION:        OpenSplice Tutorial example code.
 * MODULE:          Tutorial for the C++ programming language.
 * DATE             june 2007.
 ************************************************************************
 * 
 * This file contains the headers for the error handling operations.
 * 
 ***/

#ifndef __CHECKSTATUS_H__
  #define __CHECKSTATUS_H__

  #include "ccpp_dds_dcps.h"
  #include <iostream>

  using namespace std;

  /**
   * Returns the name of an error code.
   **/
  string getErrorName(DDS::ReturnCode_t status);

  /**
   * Check the return status for errors. If there is an error, then terminate.
   **/
  void checkStatus(DDS::ReturnCode_t status, const char *info);

  /**
   * Check whether a valid handle has been returned. If not, then terminate.
   **/
  void checkHandle(void *handle, string info);

#endif
