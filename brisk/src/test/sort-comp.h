/*
 * sort_comp.h
 *
 *  Created on: 19 Sep 2011
 *      Author: slynen
 */

#ifndef SORT_COMP_H_
#define SORT_COMP_H_
#include <sstream>
#include <algorithm>
#include <brisk/brisk.h>

bool numeric_string_compare(const std::string& s1, const std::string& s2)
{
  std::string::const_iterator it1 = s1.begin(), it2 = s2.begin();
  std::string::const_iterator rit1 = s1.end(), rit2 = s2.end();
  //find beginning of number
  --rit1; --rit2;
  while(!std::isdigit(*rit1)){--rit1;}
  it1 = rit1;
  while(std::isdigit(*it1)){--it1;}
  ++it1;
  while(!std::isdigit(*rit2)){--rit2;}
  it2 = rit2;
  while(std::isdigit(*it2)){--it2;}
  ++it2;

  if(rit1-it1==rit2-it2){
    while(it1!=rit1 && it2!=rit2){
      if(*it1>*it2){
        return true;
      }else if (*it1<*it2){
        return false;
      }
      ++it1;
      ++it2;
    }
    if(*it1>*it2){
      return true;
    }else{
      return false;
    }
  }else if(rit1-it1>rit2-it2){
    return true;
  }else{
    return false;
  }
}

#endif /* SORT_COMP_H_ */
