import onnxruntime as ort

so = ort.SessionOptions()
so.enable_profiling = True
so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED

# TensorRT EP 参数
trt_provider_options = {
    "trt_engine_cache_enable": "True",
    "trt_engine_cache_path": "./trt_cache",
}

sess = ort.InferenceSession(
    "model/best_fp16.onnx",
    so,
    providers=[("TensorrtExecutionProvider", trt_provider_options)],
)
