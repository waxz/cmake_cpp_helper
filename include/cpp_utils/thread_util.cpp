//
// Created by waxz on 19-1-16.
//
#include "thread_util.h"

namespace thread_util {

    ThreadGuard::~ThreadGuard() {
        if (m_thread_.joinable()) {
            // todo fix
            m_thread_.join();
        }
    }


}