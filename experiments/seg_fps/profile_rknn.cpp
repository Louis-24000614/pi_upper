// 标准 C API 分段计时：先确认 IO 是否为瓶颈，再决定是否值得引入 zero-copy。
// 输入是冻结的 RGB/NHWC/uint8/640×640 张量；不修改模型、量化或频率策略。
#include <rknn_api.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <cstdlib>

using Clock = std::chrono::steady_clock;
static double ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now()-start).count();
}
static std::vector<char> read_file(const char* path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("无法读取输入文件");
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
static void check(int code) {
    if (code != RKNN_SUCC) throw std::runtime_error("RKNN API 错误: " + std::to_string(code));
}
int main(int argc, char** argv) {
    if (argc != 5) return 2;
    rknn_context ctx = 0;
    try {
        auto model = read_file(argv[1]);
        auto input = read_file(argv[2]);
        check(rknn_init(&ctx, model.data(), model.size(), 0, nullptr));
        check(rknn_set_core_mask(ctx, static_cast<rknn_core_mask>(std::atoi(argv[3]))));
        rknn_input_output_num counts{};
        check(rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &counts, sizeof(counts)));
        if (counts.n_input != 1 || input.size() != 640*640*3) throw std::runtime_error("输入不匹配");
        rknn_input in{};
        in.index = 0; in.buf = input.data(); in.size = input.size();
        in.type = RKNN_TENSOR_UINT8; in.fmt = RKNN_TENSOR_NHWC; in.pass_through = 0;
        std::vector<rknn_output> outputs(counts.n_output);
        std::vector<std::vector<float>> storage(counts.n_output);
        for (unsigned index=0; index<counts.n_output; ++index) {
            rknn_tensor_attr attr{}; attr.index = index;
            check(rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)));
            storage[index].resize(attr.n_elems);
            outputs[index].index = index; outputs[index].want_float = 1;
            outputs[index].is_prealloc = 1; outputs[index].buf = storage[index].data();
            outputs[index].size = attr.n_elems*sizeof(float);
            std::cerr << "output " << index << " elements=" << attr.n_elems
                      << " stride_size=" << attr.size_with_stride << " type=" << attr.type
                      << " fmt=" << attr.fmt << " quant=" << attr.qnt_type << '\n';
        }
        double io_in=0, run=0, io_out=0;
        int iterations = std::atoi(argv[4]);
        for (int i=-5; i<iterations; ++i) {
            auto t=Clock::now(); check(rknn_inputs_set(ctx, 1, &in)); double a=ms(t);
            t=Clock::now(); check(rknn_run(ctx, nullptr)); double b=ms(t);
            t=Clock::now(); check(rknn_outputs_get(ctx, counts.n_output, outputs.data(), nullptr));
            double c=ms(t); check(rknn_outputs_release(ctx, counts.n_output, outputs.data()));
            if (i>=0) { io_in+=a; run+=b; io_out+=c; }
        }
        std::cout << "PROFILE {\"input_ms\":" << io_in/iterations
                  << ",\"run_ms\":" << run/iterations
                  << ",\"output_ms\":" << io_out/iterations << "}" << std::endl;
        rknn_destroy(ctx);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        if (ctx) rknn_destroy(ctx);
        return 1;
    }
}
