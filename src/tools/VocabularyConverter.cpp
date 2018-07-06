/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
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

 /*
  * Usage: ./VocabularyConverter ORBvoc.txt ORBvox.tar.gz
  */

#include <iostream>
#include "ORBVocabularyExt.h"
#include "logging_util.h"

int main(int argc, const char * argv[]){
    if (argc != 3){
        LOG_ERROR << "Invalid Usage" << std::endl;
        return 1;
    }

    MapGen::ORBVocabularyExt voc;
    // load vocabulary
    if (!voc.loadFromTextFile(argv[1])){
        LOG_ERROR << "Failed to load from " + std::string(argv[1]) << std::endl;
    }

    // save vocabulary
    voc.save(argv[2]);
    LOG_ERROR << "Vocabulary saved to file: " + std::string(argv[2]) << std::endl;

    return 0;

}