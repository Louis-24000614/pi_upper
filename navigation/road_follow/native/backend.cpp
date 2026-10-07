// 标准 IO 与“仅输入绑定内存”使用相同 C API/输出路径，以隔离 IO 方式的收益。
// 不猜测 dtype/stride；不支持的原生布局直接报错，不能静默生成错误道路 mask。
#include <rknn_api.h>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
thread_local std::string error;
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point t) {return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
void check(int r) {if(r!=RKNN_SUCC) throw std::runtime_error("RKNN code="+std::to_string(r));}
struct Session {
    rknn_context ctx=0;
    rknn_tensor_mem* input_mem=nullptr;
    rknn_tensor_attr native_input{};
    std::vector<rknn_tensor_attr> attrs;
    std::vector<std::vector<float>> data;
    std::vector<rknn_output> outputs;
    bool zero=false;
    double times[3]{};
    ~Session() {
        if(input_mem) rknn_destroy_mem(ctx,input_mem);
        if(ctx) rknn_destroy(ctx);
    }
};
}
extern "C" {
const char* road_rknn_error() {return error.c_str();}
void* road_rknn_create(const char* path, int core, int zero) {
    auto* s=new Session;
    try {
        std::ifstream f(path,std::ios::binary);
        if(!f) throw std::runtime_error("无法读取模型");
        std::vector<char> model{std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};
        check(rknn_init(&s->ctx,model.data(),model.size(),0,nullptr));
        check(rknn_set_core_mask(s->ctx,static_cast<rknn_core_mask>(core)));
        rknn_input_output_num n{};
        check(rknn_query(s->ctx,RKNN_QUERY_IN_OUT_NUM,&n,sizeof(n)));
        if(n.n_input!=1 || n.n_output!=2) throw std::runtime_error("仅支持道路 seg 的一个输入/两个输出");
        s->zero=zero;
        if(zero) {
            check(rknn_query(s->ctx,RKNN_QUERY_NATIVE_INPUT_ATTR,&s->native_input,sizeof(s->native_input)));
            auto& a=s->native_input;
            if(a.fmt!=RKNN_TENSOR_NHWC || a.n_dims!=4 || a.dims[0]!=1 ||
               a.dims[1]!=640 || a.dims[2]!=640 || a.dims[3]!=3)
                throw std::runtime_error("不支持的原生输入布局；拒绝猜测 stride");
            // 官方示例允许 UINT8 输入由 NPU 执行归一化/量化，不能手工猜 mean/std。
            a.type=RKNN_TENSOR_UINT8; a.pass_through=0;
            const auto bytes=640*std::max<unsigned>(640,a.w_stride)*3;
            s->input_mem=rknn_create_mem(s->ctx,std::max<unsigned>(bytes,a.size_with_stride));
            if(!s->input_mem) throw std::runtime_error("rknn_create_mem 失败");
            check(rknn_set_io_mem(s->ctx,s->input_mem,&a));
        }
        s->attrs.resize(n.n_output); s->data.resize(n.n_output); s->outputs.resize(n.n_output);
        for(unsigned i=0;i<n.n_output;++i) {
            auto& a=s->attrs[i]; a.index=i;
            check(rknn_query(s->ctx,RKNN_QUERY_OUTPUT_ATTR,&a,sizeof(a)));
            if(a.n_dims>8) throw std::runtime_error("输出维度超限");
            s->data[i].resize(a.n_elems);
            auto& o=s->outputs[i]; o.index=i; o.want_float=1; o.is_prealloc=1;
            o.buf=s->data[i].data(); o.size=a.n_elems*sizeof(float);
        }
        return s;
    } catch(const std::exception& e) {error=e.what(); delete s; return nullptr;}
}
int road_rknn_shape(void* handle,int index,unsigned* dims) {
    auto* s=static_cast<Session*>(handle);
    if(index<0 || index>=static_cast<int>(s->attrs.size())) return -1;
    auto& a=s->attrs[index];
    std::memcpy(dims,a.dims,a.n_dims*sizeof(unsigned));
    return a.n_dims;
}
float* road_rknn_output(void* handle,int index) {return static_cast<Session*>(handle)->data.at(index).data();}
int road_rknn_infer(void* handle,const unsigned char* input,unsigned bytes) {
    auto* s=static_cast<Session*>(handle);
    try {
        if(bytes!=640*640*3) throw std::runtime_error("输入字节数错误");
        auto t=Clock::now();
        if(s->zero) {
            // 按查询到的行跨度复制，先清空 padding，防止输入间残留。
            std::memset(s->input_mem->virt_addr,0,s->input_mem->size);
            auto stride=std::max<unsigned>(640,s->native_input.w_stride)*3;
            for(unsigned y=0;y<640;++y)
                std::memcpy(static_cast<unsigned char*>(s->input_mem->virt_addr)+y*stride,input+y*640*3,640*3);
            check(rknn_mem_sync(s->ctx,s->input_mem,RKNN_MEMORY_SYNC_TO_DEVICE));
        } else {
            rknn_input in{}; in.index=0; in.buf=const_cast<unsigned char*>(input); in.size=bytes;
            in.type=RKNN_TENSOR_UINT8; in.fmt=RKNN_TENSOR_NHWC; in.pass_through=0;
            check(rknn_inputs_set(s->ctx,1,&in));
        }
        s->times[0]=elapsed(t); t=Clock::now();
        check(rknn_run(s->ctx,nullptr)); s->times[1]=elapsed(t); t=Clock::now();
        check(rknn_outputs_get(s->ctx,s->outputs.size(),s->outputs.data(),nullptr));
        s->times[2]=elapsed(t);
        check(rknn_outputs_release(s->ctx,s->outputs.size(),s->outputs.data()));
        return 0;
    } catch(const std::exception& e) {error=e.what();return -1;}
}
void road_rknn_times(void* handle,double* dst) {std::memcpy(dst,static_cast<Session*>(handle)->times,3*sizeof(double));}
void road_rknn_destroy(void* handle) {delete static_cast<Session*>(handle);}
}
