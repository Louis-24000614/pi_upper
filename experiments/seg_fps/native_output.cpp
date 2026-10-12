// Reuse the existing C backend without replacing its library or production API.
#define road_rknn_error base_error
#define road_rknn_create base_create
#define road_rknn_shape base_shape
#define road_rknn_output base_output
#define road_rknn_infer base_infer
#define road_rknn_times base_times
#define road_rknn_destroy base_destroy
#include "../../navigation/road_follow/native/backend.cpp"
#undef road_rknn_error
#undef road_rknn_create
#undef road_rknn_shape
#undef road_rknn_output
#undef road_rknn_infer
#undef road_rknn_times
#undef road_rknn_destroy

namespace {
struct NativeOutput {
    Session* base = nullptr;
    bool native = false;
    bool external_ready = false;
    std::vector<rknn_tensor_attr> attrs;
    std::vector<rknn_tensor_mem*> memories;
    ~NativeOutput() {
        if (base) {
            for (auto* mem : memories) if (mem) rknn_destroy_mem(base->ctx, mem);
            delete base;
        }
    }
    void convert(unsigned i) {
        const auto& a = attrs[i];
        const auto& logical = base->attrs[i];
        const auto* source = static_cast<const __fp16*>(memories[i]->virt_addr);
        auto& output = base->data[i];
        if (a.fmt == RKNN_TENSOR_NC1HWC2) {
            // Follow the RKNN model-zoo channel-block mapping; retain padding in allocation only.
            unsigned channels = logical.dims[1], h = logical.dims[2], w = logical.dims[3];
            unsigned c2 = a.dims[4], hw = a.dims[2] * a.dims[3];
            for (unsigned c = 0; c < channels; ++c) {
                const auto* plane = source + (c / c2) * hw * c2 + c % c2;
                for (unsigned y = 0; y < h; ++y)
                    for (unsigned x = 0; x < w; ++x)
                        output[(c * h + y) * w + x] = plane[(y * a.dims[3] + x) * c2];
            }
        } else {
            for (unsigned j = 0; j < logical.n_elems; ++j) output[j] = source[j];
        }
    }
};
}
extern "C" {
const char* road_rknn_error() { return error.c_str(); }
void* road_rknn_create(const char* path, int core, int flags) {
    auto* s = new NativeOutput;
    try {
        s->base = static_cast<Session*>(base_create(path, core, flags & 1));
        if (!s->base) throw std::runtime_error(error);
        s->native = flags & 2;
        if (s->native) {
            s->attrs.resize(2); s->memories.resize(2, nullptr);
            for (unsigned i = 0; i < 2; ++i) {
                auto& a = s->attrs[i]; a.index = i;
                check(rknn_query(s->base->ctx, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &a, sizeof(a)));
                const bool identity_quantization = a.qnt_type == RKNN_TENSOR_QNT_NONE ||
                    (a.qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && a.scale == 1.0f && a.zp == 0);
                if (a.type != RKNN_TENSOR_FLOAT16 || !identity_quantization)
                    throw std::runtime_error("unsupported native type=" + std::to_string(a.type) +
                                             " quantization=" + std::to_string(a.qnt_type) +
                                             " scale=" + std::to_string(a.scale) + " zp=" + std::to_string(a.zp));
                const auto& logical = s->base->attrs[i];
                if (a.fmt == RKNN_TENSOR_NC1HWC2) {
                    if (a.n_dims != 5 || logical.n_dims != 4 || logical.fmt != RKNN_TENSOR_NCHW ||
                        a.dims[0] != 1 || logical.dims[0] != 1 ||
                        a.dims[1] * a.dims[4] < logical.dims[1] ||
                        a.dims[2] != logical.dims[2] || a.dims[3] != logical.dims[3])
                        throw std::runtime_error("unsupported NC1HWC2 shape/stride");
                } else {
                    if (a.n_dims != logical.n_dims || a.w_stride != 0)
                        throw std::runtime_error("unsupported native non-blocked stride");
                    for (unsigned d = 0; d < a.n_dims; ++d)
                        if (a.dims[d] != logical.dims[d]) throw std::runtime_error("native dimension mismatch");
                }
                s->memories[i] = rknn_create_mem(s->base->ctx, a.size_with_stride);
                if (!s->memories[i]) throw std::runtime_error("native output allocation failed");
                check(rknn_set_io_mem(s->base->ctx, s->memories[i], &a));
            }
        }
        return s;
    } catch (const std::exception& e) { error = e.what(); delete s; return nullptr; }
}
int road_rknn_shape(void* handle, int index, unsigned* dims) {
    return base_shape(static_cast<NativeOutput*>(handle)->base, index, dims);
}
float* road_rknn_output(void* handle, int index) {
    return base_output(static_cast<NativeOutput*>(handle)->base, index);
}
int road_rknn_infer(void* handle, const unsigned char* input, unsigned bytes) {
    auto* s = static_cast<NativeOutput*>(handle);
    auto* b = s->base;
    if (!s->native && !s->external_ready) return base_infer(b, input, bytes);
    try {
        if (bytes != 640 * 640 * 3) throw std::runtime_error("invalid input length");
        auto t = Clock::now();
        if (s->external_ready) {
            s->external_ready = false;
            if (!b->zero) throw std::runtime_error("external input requires bound IO");
            check(rknn_mem_sync(b->ctx, b->input_mem, RKNN_MEMORY_SYNC_FROM_DEVICE));
        } else if (b->zero) {
            std::memset(b->input_mem->virt_addr, 0, b->input_mem->size);
            auto stride = std::max<unsigned>(640, b->native_input.w_stride) * 3;
            for (unsigned y = 0; y < 640; ++y)
                std::memcpy(static_cast<unsigned char*>(b->input_mem->virt_addr) + y * stride,
                            input + y * 640 * 3, 640 * 3);
            check(rknn_mem_sync(b->ctx, b->input_mem, RKNN_MEMORY_SYNC_TO_DEVICE));
        } else {
            rknn_input in{}; in.buf = const_cast<unsigned char*>(input); in.size = bytes;
            in.type = RKNN_TENSOR_UINT8; in.fmt = RKNN_TENSOR_NHWC;
            check(rknn_inputs_set(b->ctx, 1, &in));
        }
        b->times[0] = elapsed(t); t = Clock::now();
        check(rknn_run(b->ctx, nullptr)); b->times[1] = elapsed(t); t = Clock::now();
        if (s->native) {
            for (unsigned i = 0; i < 2; ++i) {
                check(rknn_mem_sync(b->ctx, s->memories[i], RKNN_MEMORY_SYNC_FROM_DEVICE));
                s->convert(i);
            }
        } else {
            check(rknn_outputs_get(b->ctx, b->outputs.size(), b->outputs.data(), nullptr));
            check(rknn_outputs_release(b->ctx, b->outputs.size(), b->outputs.data()));
        }
        b->times[2] = elapsed(t);
        return 0;
    } catch (const std::exception& e) { error = e.what(); return -1; }
}
void road_rknn_times(void* handle, double* dst) { base_times(static_cast<NativeOutput*>(handle)->base, dst); }
void road_rknn_destroy(void* handle) { delete static_cast<NativeOutput*>(handle); }
int road_rknn_input_fd(void* handle) {
    auto* b = static_cast<NativeOutput*>(handle)->base;
    return b->input_mem ? b->input_mem->fd : -1;
}
void* road_rknn_input_ptr(void* handle) {
    auto* b = static_cast<NativeOutput*>(handle)->base;
    return b->input_mem ? b->input_mem->virt_addr : nullptr;
}
unsigned road_rknn_input_size(void* handle) {
    auto* b = static_cast<NativeOutput*>(handle)->base;
    return b->input_mem ? b->input_mem->size : 0;
}
unsigned road_rknn_input_stride(void* handle) {
    auto* b = static_cast<NativeOutput*>(handle)->base;
    return std::max<unsigned>(640, b->native_input.w_stride);
}
void road_rknn_mark_ready(void* handle) { static_cast<NativeOutput*>(handle)->external_ready = true; }
}
