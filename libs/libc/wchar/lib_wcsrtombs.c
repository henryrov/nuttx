/****************************************************************************
 * libs/libc/wchar/lib_wcsrtombs.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <wchar.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcsrtombs
 *
 * Description:
 *   Convert a wide-characterto a  multibyte string string
 ****************************************************************************/

size_t wcsrtombs(FAR char *dst, FAR const wchar_t **src,
                 size_t len, FAR mbstate_t *ps)
{
  size_t ret;

  ret = wcsnrtombs(dst, src, SIZE_MAX, len, ps);

  if (dst != NULL && ret != (size_t)-1 && ret != len)
    {
      dst[ret] = '\0';
    }

  return ret;
}
