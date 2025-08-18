import onnxruntime
import numpy as np
import numpy as np

import openvino as ov


def run_onnx_model_with_openvino(model_path):
    """
    Runs an ONNX model using OpenVINO.

    Args:
        model_path (str): Path to the ONNX model.
    """
    core = ov.Core()
    print("Available devices:", core.available_devices)
    # print(core.get_property("GPU", "OPTIMIZATION_CAPABILITIES"))

    model_onnx = core.read_model(model_path)
    compiled_model = core.compile_model(
        model_onnx, "GPU"
    )  # "AUTO" for automatic device selection

    input_layer = compiled_model.input(0)
    input_shape = input_layer.shape

    # Create dummy input data (replace with your actual input data)
    input_data = np.random.randn(*input_shape).astype(np.float32)

    # Run inference
    results = compiled_model([input_data])

    print("OpenVINO inference results:", results)


def run_onnx_model_with_onnxruntime(model_path):
    """
    Runs an ONNX model using ONNX Runtime.

    Args:
        model_path (str): Path to the ONNX model.
    """
    sess = onnxruntime.InferenceSession(model_path)
    input_name = sess.get_inputs()[0].name
    input_shape = sess.get_inputs()[0].shape

    # Create dummy input data (replace with your actual input data)
    input_data = np.random.randn(*input_shape).astype(np.float32)

    # Run inference
    results = sess.run(None, {input_name: input_data})

    print("ONNX Runtime inference results:", results)


if __name__ == "__main__":
    model_path = "model/best.onnx"  # Path to your ONNX model

    print("Running with OpenVINO:")
    run_onnx_model_with_openvino(model_path)

    # print("\nRunning with ONNX Runtime:")
    # run_onnx_model_with_onnxruntime(model_path)
