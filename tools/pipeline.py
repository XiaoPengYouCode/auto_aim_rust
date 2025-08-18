from onnxruntime.quantization import quantize_dynamic, QuantType
import onnxruntime as ort
import numpy as np


def quantize_model():
    # 输入和输出模型路径
    input_model_path = "model/best_preprocess.onnx"
    output_model_path = "model/best_int8.onnx"

    # 执行动态量化（只量化权重）
    quantize_dynamic(
        model_input=input_model_path,
        model_output=output_model_path,
        weight_type=QuantType.QInt8,
    )
    print("✅ 模型量化完成:", output_model_path)


def run_inference(model_path):
    # 创建 ONNX Runtime Session，使用 CPU 执行提供者（支持量化算子）
    sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])

    # 获取输入输出名称
    input_name = sess.get_inputs()[0].name
    output_name = sess.get_outputs()[0].name

    # 模拟输入数据（形状要与原模型一致）
    dummy_input = np.random.rand(1, 3, 640, 384).astype(np.float32)

    # 推理
    outputs = sess.run([output_name], {input_name: dummy_input})

    print("✅ 推理完成，输出 shape:", outputs[0].shape)


if __name__ == "__main__":
    quantize_model()
    run_inference("model/best_int8.onnx")
