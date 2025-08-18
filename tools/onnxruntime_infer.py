import onnxruntime as ort
import numpy as np

# 量化后模型路径
model_quant_path = "model/best_int8.onnx"

# 创建 ONNX Runtime Session
# 可以指定执行提供者，例如 'CPUExecutionProvider' 或 'CUDAExecutionProvider'
sess = ort.InferenceSession(model_quant_path, providers=["CPUExecutionProvider"])

# 获取模型的输入和输出信息
input_name = sess.get_inputs()[0].name
output_name = sess.get_outputs()[0].name

# 准备输入数据 (确保数据类型和形状与模型期望的匹配)
# 即使是量化模型，输入通常仍然是浮点类型，ONNX Runtime 会在内部处理量化
dummy_input = np.random.rand(1, 3, 384, 640).astype(np.float32)

# 执行推理
outputs = sess.run([output_name], {input_name: dummy_input})

print("推理结果:", outputs[0])
