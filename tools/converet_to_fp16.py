import onnx
from onnxconverter_common import float16

# 1. 读取你的原始模型（要是非量化模型才行，最好是 float32）
model = onnx.load("model/best.onnx")

# 2. 转换成 float16
model_fp16 = float16.convert_float_to_float16(model, keep_io_types=True)

# 3. 保存成新模型
onnx.save(model_fp16, "model/best_fp16.onnx")
