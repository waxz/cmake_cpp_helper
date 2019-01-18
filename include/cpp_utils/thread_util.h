//
// Created by waxz on 18-11-21.
//

#ifndef EVENTLOOP_THREAD_UTIL_H
#define EVENTLOOP_THREAD_UTIL_H

#include <thread>                   // thread
#include <memory>                   // shared_ptr
#include <functional>               // bind
#include <mutex>                    // mutex, lock
#include <condition_variable>       // condition_variable_any
#include <chrono>                   // time and sleep
#include <map>                      // map

namespace thread_util {


    class ThreadGuard {
    private:
        std::thread &m_thread_;

    public:
        explicit ThreadGuard(std::thread &thread_) : m_thread_(thread_) {};

        ~ThreadGuard();

        // delete
        ThreadGuard(ThreadGuard const &) = delete;

        ThreadGuard &operator=(ThreadGuard const &) = delete;

    };


    class ThreadGuardscope {
    private:
        std::thread m_thread_;

    public:
        explicit ThreadGuardscope(std::thread thread_) : m_thread_(std::move(thread_)) {
            if (!m_thread_.joinable()) {
                throw std::logic_error("No thread !");
            }
        };

        ~ThreadGuardscope() {
            if (m_thread_.joinable()) {
                m_thread_.join();
            }
        }

        // delete
        ThreadGuardscope(ThreadGuardscope const &) = delete;

        ThreadGuardscope &operator=(ThreadGuardscope const &) = delete;

    };

    class BaseEntry {
    protected:
        bool m_started;
        bool m_running;
        std::condition_variable_any m_cv;
        std::recursive_mutex m_mutex;


    public:
        BaseEntry() : m_started(false), m_running(true) {}

        void Start() {
            m_started = true;
            m_cv.notify_all();

        }

        void Pause() {
            m_started = false;
        }

        void Stop() {
            m_running = false;
        }

        bool isStarted() {
            return m_started;
        }

        bool isRun() {
            return m_running;
        }

        virtual void run()=0;
    };

    class LoopEntry : public BaseEntry {
    public:
        LoopEntry() : BaseEntry() {

        }

        void update() {
            // get lock
            this->Pause();

            std::unique_lock<std::recursive_mutex> lock(m_mutex);


            // update member data
        }

        void run() {
            std::unique_lock<std::recursive_mutex> lock(m_mutex);
            std::cout << "======\n cv wait lock  \n========" << std::endl;
            int i = 0;
            while (m_running) {

                i++;
                m_cv.wait(lock, std::bind(&LoopEntry::isStarted, this));
                std::cout << "======\n cv \n ========\n i = " << i << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

            }
        }

    };

    template<typename T>
    void EntryRunnerRef(T &entry) {
        entry.run();
    }

    // thread model with guard , thread constructor
    template<typename T>
    class ThredModel {
    private:
        T &m_entry_;
        std::thread m_thread_;
        ThreadGuard tg;
    public:
        ThredModel(T &entry) : m_entry_(entry),
                               m_thread_(EntryRunnerRef<T>, std::ref(entry)),
                               tg(m_thread_) {

        }

        ~ThredModel() {
            m_entry_.Stop();
            if (!m_entry_.isStarted()) {
                m_entry_.Start();

            }
        }
    };

    // entry with sig
    // why sig
    // i want to control and update entry without knowning it
    template<typename T>
    class EntrySig : public BaseEntry {
    protected:
        T &m_sig;


    public:
        void sigControlFunc(bool cmd) {

            if (cmd) {
                // start loop
                if (!isStarted()) {
                    Start();
                }
            } else {
                //block loop
                if (isStarted()) {
                    Pause();
                }
            }
        }

        EntrySig(T &sig) : BaseEntry(), m_sig(sig) {

            // connect function

        }
    };

    // use case
#if 0
    // use thread model
    thread_util::LoopEntry entry;
    thread_util::ThredModel<thread_util::LoopEntry> tm(entry);
    entry.Start();
    entry.Pause();


    // thread management
    std::vector<std::thread> threadvec(n);
    for (int i = 0; i < n; i++){
        threadvec[i] = std::thread(cv_util::filterMatch, std::ref(grid),std::ref(kern),std::ref(matchLoc),std::ref(matchVal), true);

    }


    std::for_each(threadvec.begin(), threadvec.end(), std::mem_fn(&std::thread::join));

#endif

}
#endif //EVENTLOOP_THREAD_UTIL_H
