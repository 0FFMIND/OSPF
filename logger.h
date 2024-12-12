#ifndef LOGGER_H
#define LOGGER_H

#include <mutex>
#include <iostream>
#include <string>
#include <unordered_map>

class Logger{
private:
    std::mutex mtx;
    std::unordered_map<std::string, bool> logLevels;
    Logger() = default;
public:
    static Logger& getInstance(){
        static Logger instance;
        return instance;
    }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    void enableLog(const std::string& module){
        std::unique_lock<std::mutex> lock(mtx);
        logLevels[module] = true;
    }
    void disableLog(const std::string& module){
        std::unique_lock<std::mutex> lock(mtx);
        logLevels[module] = false;
    }
    void Log()
};

#endif