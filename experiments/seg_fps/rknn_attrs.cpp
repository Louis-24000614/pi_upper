#include <rknn_api.h>
#include <fstream>
#include <iterator>
#include <cstdio>
#include <vector>
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    std::ifstream f(argv[1], std::ios::binary);
    std::vector<char> model{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    rknn_context ctx = 0;
    if (rknn_init(&ctx, model.data(), model.size(), 0, nullptr)) return 3;
    for (auto query : {RKNN_QUERY_OUTPUT_ATTR, RKNN_QUERY_NATIVE_OUTPUT_ATTR, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR}) {
        for (unsigned i = 0; i < 2; ++i) {
            rknn_tensor_attr a{}; a.index = i;
            int code = rknn_query(ctx, query, &a, sizeof(a));
            printf("query=%d index=%u code=%d type=%d qnt=%d zp=%d scale=%g fmt=%d elems=%u size=%u stride_size=%u w_stride=%u h_stride=%u dims=",
                   query, i, code, a.type, a.qnt_type, a.zp, a.scale, a.fmt, a.n_elems, a.size,
                   a.size_with_stride, a.w_stride, a.h_stride);
            for (unsigned d = 0; d < a.n_dims; ++d) printf("%u,", a.dims[d]);
            printf("\n");
        }
    }
    rknn_destroy(ctx);
}
