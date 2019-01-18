//
// Created by waxz on 18-11-21.
//

#ifndef EVENTLOOP_SIGNAL_UTIL_H
#define EVENTLOOP_SIGNAL_UTIL_H

#include <thread>                   // thread
#include <memory>                   // shared_ptr
#include <functional>               // bind
#include <mutex>                    // mutex, lock
#include <condition_variable>       // condition_variable_any
#include <chrono>                   // time and sleep
#include <map>                      // map

#include <boost/signals2.hpp>       // signals2

namespace sig_util {

    // aggregate_values is a combiner which places all the values returned
    // from slots into a container
    // deal with bool
    template<typename Container>
    struct CollectResult {
        typedef Container result_type;

        template<typename InputIterator>
        Container operator()(InputIterator first, InputIterator last) const {
            Container values;

            while (first != last) {
//                std::cout<<"get res: " << *first<< std::endl;
                if (*first) {
                    return true;
                }

                ++first;
            }
            return false;
        }
    };


    template<typename T>
    class MsgQueue {
    public:
        typedef boost::signals2::signal<bool(double), CollectResult<bool>> signal_t;
    private:
        std::string m_topic;
        unsigned int m_buffer_size;
        double m_timeout;

        signal_t m_sig;

    public:

        // remember to initialize
        std::shared_ptr<T> m_msg_ptr;

        MsgQueue(std::string topic, unsigned int buffer_size = 1, double timeout = 0.0) :
                m_topic(topic),
                m_buffer_size(buffer_size),
                m_timeout(timeout),
                m_msg_ptr(std::make_shared<T>()) {}

        boost::signals2::connection connect(const signal_t::slot_type &sub) {
            return m_sig.connect(sub);
        }

        bool getOneMsg(double timeout) {
            auto res = m_sig(timeout);

            return res;
        };

        std::shared_ptr<T> getMsg() {
            return m_msg_ptr;
        }

        std::string Topic() {
            return m_topic;
        }

        double Timeout() {
            return m_timeout;
        }

        unsigned int BuffSize() {
            return m_buffer_size;
        }


    };
}


#endif //EVENTLOOP_SIGNAL_UTIL_H
