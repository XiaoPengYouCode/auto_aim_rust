from onnxruntime.quantization import quantize_dynamic, QuantType


def main():
    # 输入和输出模型路径
    input_model_path = "model/best_preprocess.onnx"
    output_model_path = "model/best_int8.onnx"

    # 动态量化（仅量化权重）
    quantize_dynamic(
        input_model_path,
        output_model_path,
        weight_type=QuantType.QUInt8,  # 量化权重为 INT8
    )


if __name__ == "__main__":
    main()
