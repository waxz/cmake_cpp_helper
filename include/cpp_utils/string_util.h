//
// Created by waxz on 18-11-22.
//

#ifndef EVENTLOOP_STRING_UTIL_H
#define EVENTLOOP_STRING_UTIL_H

#include <cstdio>
#include <valarray>
#include <iostream>
#include <vector>
#include <thread>
#include <memory>

#include <string>
#include <cstdlib>


namespace string_util {
    std::vector <std::string> string_split(const std::string &s, char delimiter) {
        std::vector <std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }


    class String {
    private:
        std::string m_string_;
    public:
        String(std::string str) : m_string_(str) {};

        std::vector <std::string> split(char delimiter) {

            return string_split(m_string_, delimiter);

        }

        template<typename T>
        std::vector <T> splitAsVector(char delimiter) {

            std::vector <std::string> vec = split(delimiter);
            std::vector <T> res;
            for (auto e : vec) {
                res.push_back(T(atof(e.c_str())));
            }

            return res;


        }

        template<typename T>
        std::valarray <T> splitAsValarray(char delimiter) {

            std::vector <std::string> vec = split(delimiter);
            size_t sz = vec.size();

            std::valarray <T> res(0.0, sz);
            for (size_t i = 0; i < sz; i++) {
                res[i] = T(atof(vec[i].c_str()));
            }

            return res;


        }


        String strip(const char delimiter) {

            std::string token;
            std::string new_string;
            std::istringstream tokenStream(m_string_);
            while (std::getline(tokenStream, token, delimiter)) {
                new_string += token;
            }
            this->m_string_ = new_string;
            return *this;

        }

    };
}

#endif //EVENTLOOP_STRING_UTIL_H
