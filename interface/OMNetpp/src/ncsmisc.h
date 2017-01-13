/*
 * misc.h
 *
 *  Created on: Oct 19, 2016
 *      Author: mk
 */

#ifndef MISC_H_
#define MISC_H_

#include <iostream>

class ncsMisc{
public:

    template<typename T>
    static
    inline
    std::vector<T> UnrollVectors(const std::vector<std::vector<T>>& a){
        std::vector<T> v;
        for(size_t i=0; i<a.size(); i++)
            ncsMisc::AppendVector(v,a[i]);
        return v;
    }

    template<typename T>
    static
    inline
    void AppendVector(std::vector<T>& a, const std::vector<T>& b){
        a.reserve(a.size() + b.size());
        a.insert(a.end(), b.begin(), b.end());
    }

    static inline
    std::vector<double> CssToVectorDouble(std::string s){
        std::vector<double> ret;
        std::stringstream ss(s);

        while(ss.good()){
            std::string subs;
            std::getline(ss, subs, ',');

            double subElement;
            std::stringstream subss(subs);
            subss >> subElement;

            ret.push_back(subElement);
        }
        return ret;
    }

    static inline
    std::vector<std::string> CssToVectorStrings(std::string s){
        std::vector<std::string> ret;
        std::stringstream ss(s);

        while(ss.good()){
            std::string subs;
            std::getline(ss, subs, ',');
            ret.push_back(subs);
        }
        return ret;
    }

    static inline
    std::string VectorDoubleToCss(std::vector<double> v){
        std::stringstream ret("");

        for(size_t i=0; i<v.size(); i++){
            ret << v[i];
            if(i != (v.size()-1))
                ret << ",";
        }

        return ret.str();
    }
};




#endif /* MISC_H_ */
