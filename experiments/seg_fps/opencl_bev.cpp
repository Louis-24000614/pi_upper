#define CL_TARGET_OPENCL_VERSION 120
#include <CL/cl.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
thread_local std::string error;
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
void check(cl_int code) {
    if (code != CL_SUCCESS) throw std::runtime_error("OpenCL code=" + std::to_string(code));
}
struct Session {
    cl_context context = nullptr;
    cl_command_queue queue = nullptr;
    cl_program program = nullptr;
    cl_kernel kernel = nullptr;
    cl_mem input = nullptr, output = nullptr, map = nullptr;
    size_t input_bytes = 0, pixels = 0;
    double times[4]{};
    ~Session() {
        if (queue) clFinish(queue);
        if (map) clReleaseMemObject(map);
        if (output) clReleaseMemObject(output);
        if (input) clReleaseMemObject(input);
        if (kernel) clReleaseKernel(kernel);
        if (program) clReleaseProgram(program);
        if (queue) clReleaseCommandQueue(queue);
        if (context) clReleaseContext(context);
    }
};
const char* source = R"CL(
__kernel void gather(__global const uchar* src, __global const int* indices,
                     __global uchar* dst, const int count) {
    const int i = get_global_id(0);
    if (i < count) {
        const int p = indices[i];
        dst[i] = p < 0 ? (uchar)0 : src[p];
    }
}
)CL";
}
extern "C" {
const char* road_cl_error() { return error.c_str(); }
void* road_cl_create(size_t input_bytes, const int32_t* indices, size_t pixels) {
    auto* s = new Session;
    try {
        if (!input_bytes || !pixels || !indices) throw std::runtime_error("invalid BEV map");
        for (size_t i = 0; i < pixels; ++i)
            if (indices[i] < -1 || indices[i] >= static_cast<int64_t>(input_bytes))
                throw std::runtime_error("BEV map index out of bounds");
        cl_uint count = 0;
        check(clGetPlatformIDs(0, nullptr, &count));
        std::vector<cl_platform_id> platforms(count);
        check(clGetPlatformIDs(count, platforms.data(), nullptr));
        cl_device_id device = nullptr;
        for (auto platform : platforms) {
            if (clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr) == CL_SUCCESS) break;
        }
        if (!device) throw std::runtime_error("no GPU OpenCL device; CPU fallback forbidden in experiment");
        cl_int code = 0;
        s->context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &code); check(code);
        s->queue = clCreateCommandQueue(s->context, device, CL_QUEUE_PROFILING_ENABLE, &code); check(code);
        s->program = clCreateProgramWithSource(s->context, 1, &source, nullptr, &code); check(code);
        code = clBuildProgram(s->program, 1, &device, "", nullptr, nullptr);
        if (code != CL_SUCCESS) {
            size_t n = 0;
            clGetProgramBuildInfo(s->program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &n);
            std::string log(n, '\0');
            clGetProgramBuildInfo(s->program, device, CL_PROGRAM_BUILD_LOG, n, log.data(), nullptr);
            throw std::runtime_error(log);
        }
        s->kernel = clCreateKernel(s->program, "gather", &code); check(code);
        s->input = clCreateBuffer(s->context, CL_MEM_READ_ONLY, input_bytes, nullptr, &code); check(code);
        s->output = clCreateBuffer(s->context, CL_MEM_WRITE_ONLY, pixels, nullptr, &code); check(code);
        s->map = clCreateBuffer(s->context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                               pixels * sizeof(int32_t), const_cast<int32_t*>(indices), &code); check(code);
        int n = static_cast<int>(pixels);
        check(clSetKernelArg(s->kernel, 0, sizeof(cl_mem), &s->input));
        check(clSetKernelArg(s->kernel, 1, sizeof(cl_mem), &s->map));
        check(clSetKernelArg(s->kernel, 2, sizeof(cl_mem), &s->output));
        check(clSetKernelArg(s->kernel, 3, sizeof(int), &n));
        s->input_bytes = input_bytes; s->pixels = pixels;
        return s;
    } catch (const std::exception& e) { error = e.what(); delete s; return nullptr; }
}
int road_cl_run(void* handle, const uint8_t* input, uint8_t* output, double* times) {
    auto* s = static_cast<Session*>(handle);
    cl_event event = nullptr;
    try {
        auto t = Clock::now();
        check(clEnqueueWriteBuffer(s->queue, s->input, CL_TRUE, 0, s->input_bytes,
                                  input, 0, nullptr, nullptr));
        s->times[0] = ms(t); t = Clock::now();
        size_t global = ((s->pixels + 63) / 64) * 64, local = 64;
        check(clEnqueueNDRangeKernel(s->queue, s->kernel, 1, nullptr, &global, &local, 0, nullptr, &event));
        check(clWaitForEvents(1, &event));
        s->times[1] = ms(t); t = Clock::now();
        check(clEnqueueReadBuffer(s->queue, s->output, CL_TRUE, 0, s->pixels,
                                 output, 0, nullptr, nullptr));
        s->times[2] = ms(t);
        cl_ulong begin = 0, end = 0;
        check(clGetEventProfilingInfo(event, CL_PROFILING_COMMAND_START, sizeof(begin), &begin, nullptr));
        check(clGetEventProfilingInfo(event, CL_PROFILING_COMMAND_END, sizeof(end), &end, nullptr));
        s->times[3] = (end - begin) / 1e6;
        std::memcpy(times, s->times, sizeof(s->times));
        clReleaseEvent(event);
        return 0;
    } catch (const std::exception& e) {
        if (event) clReleaseEvent(event);
        error = e.what(); return -1;
    }
}
void road_cl_destroy(void* handle) { delete static_cast<Session*>(handle); }
}
