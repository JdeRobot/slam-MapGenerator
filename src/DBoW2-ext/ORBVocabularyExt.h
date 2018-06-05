/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
 *
 *  The following code is a derivative work of the code from the ORB-SLAM2 project,
 *  which is licensed under the GNU Public License, version 3. This code therefore
 *  is also licensed under the terms of the GNU Public License, version 3.
 *  For more information see <https://github.com/raulmur/ORB_SLAM2>.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Changelog:   An extension of DoW2
 *  Jun 2, 2018: typedef ORBVocabularyExt
 */

#ifndef DBoW2_EXT_ORBVOCABULARYEXT_H
#define DBoW2_EXT_ORBVOCABULARYEXT_H

#include <DBoW2.h>
#include <DBoW2/FORB.h>
#include "TemplatedVocabularyExt.h"

// define ORBVocabulary
namespace MapGen {
    typedef DBoW2::TemplatedVocabularyExt<DBoW2::FORB::TDescriptor, DBoW2::FORB>
            ORBVocabularyExt;
}

#endif
