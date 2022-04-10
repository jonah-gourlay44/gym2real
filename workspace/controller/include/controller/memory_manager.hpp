#pragma once
#define STACK_SIZE 16
#include <sys/mman.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <experimental/optional>
#include <controller/controller.hpp>

#define errExit(msg)        \
    do                      \
    {                       \
        perror(msg);        \
        exit(EXIT_FAILURE); \
    } while (0)

struct MemoryConfig
{
    MemoryConfig(const std::string &config_path)
    {
        YAML::Node config = YAML::LoadFile(config_path);
        auto buffers = config["buffers"];
        for (YAML::const_iterator it = buffers.begin(); it != buffers.end(); ++it)
        {
            buffer_config.emplace(it->first.as<std::string>(), it->second.as<int>());
        }

        auto controllers = config["controllers"];
        for (YAML::const_iterator it = controllers.begin(); it != controllers.end(); ++it)
        {
            controller_config.emplace(it->first.as<std::string>(), it->second);
        }
    }
    std::map<std::string, int> buffer_config;
    std::map<std::string, YAML::Node> controller_config;
};

class MemoryManager
{
public:
    MemoryManager(const std::string &config_path) : cfg_(config_path)
    {

        if (mlockall(MCL_FUTURE) == -1) // needs root
            errExit("mlockall");

        for (std::map<std::string, int>::iterator it = cfg_.buffer_config.begin(); it != cfg_.buffer_config.end(); ++it)
        {
            /* name of shared memory segment */
            const char *name = it->first.c_str();
            /* size (in bytes) of shared memory segment */
            const int SIZE = it->second;

            /* shared memory file descriptor */
            int shm_fd;

            /* pointer to shared memory object */
            void *ptr;

            /* create the shared memory object */
            shm_fd = shm_open(name, O_CREAT | O_RDWR, S_IRWXU | S_IRWXO);
            if (shm_fd == -1)
                errExit("shm_open");

            /* configure the size of the shared memory object */
            ftruncate(shm_fd, SIZE);

            /* memory map the shared memory object */
            ptr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
            if (ptr == MAP_FAILED)
                errExit("mmap");

            if (close(shm_fd) == -1)
                errExit("close");
        }

        for (std::map<std::string, YAML::Node>::iterator it = cfg_.controller_config.begin(); it != cfg_.controller_config.end(); ++it)
        {
            if (it->second["type"].as<std::string>() == "onnx")
            {
                auto &dict = it->second;
                controllers_.emplace(
                    it->first,
                    std::make_shared<OnnxController>(dict["observations"].as<int>(), dict["actions"].as<int>(), dict["model_path"].as<std::string>()));
            }
            else if (it->second["type"].as<std::string>() == "pid")
            {
                auto &dict = it->second;

                controllers_.emplace(
                    it->first,
                    std::make_shared<PidController>(
                        dict["observations"].as<int>(),
                        dict["actions"].as<int>(),
                        dict["kp"].as<std::vector<float>>(),
                        dict["ki"].as<std::vector<float>>(),
                        dict["kd"].as<std::vector<float>>()));
            }
        }
    }

    ~MemoryManager()
    {
        for (std::map<std::string, int>::iterator it = cfg_.buffer_config.begin(); it != cfg_.buffer_config.end(); ++it)
        {
            /* name of shared memory segment */
            const char *name = it->first.c_str();

            if (shm_unlink(name) == -1)
                errExit("close");
        }
        if (munlockall() == -1)
            errExit("munlockall");
    }

    std::experimental::optional<std::shared_ptr<BaseController>> getController(const std::string &name)
    {
        auto it = controllers_.find(name);
        if (it == controllers_.end())
            return std::experimental::nullopt;
        return it->second;
    }

    // rw=0 for read, rw=1 for write
    template <class T>
    int create_buffer(const char *name, const int &rw, T *&ptr)
    {
        /* size (in bytes) of shared memory segment */
        size_t size = cfg_.buffer_config.at(name);
        int shm_fd = shm_open(name, O_RDWR, S_IRWXU | S_IRWXO);
        if (shm_fd == -1)
            return -1;
        void *p;
        if (rw == 0)
            p = mmap(0, size, PROT_READ, MAP_SHARED, shm_fd, 0);
        else if (rw == 1)
            p = mmap(0, size, PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (p == MAP_FAILED)
            return -1;
        size = size / sizeof(T);
        ptr = (T *)p;
        return size;
    }

private:
    MemoryConfig cfg_;
    std::map<std::string, std::shared_ptr<BaseController>> controllers_;
};
