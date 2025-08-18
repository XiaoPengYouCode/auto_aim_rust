# 添加归一化节点

import onnx
from onnx import helper, numpy_helper, TensorProto
import numpy as np

model = onnx.load("model/best_fp16.onnx")
graph = model.graph

input_name = graph.input[0].name
output_name = input_name + "_norm"

# 常数节点
scale = numpy_helper.from_array(
    np.array([1 / 255.0], dtype=np.float32), name="scale255"
)
mean = numpy_helper.from_array(
    np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape((3, 1, 1)), name="mean"
)
std = numpy_helper.from_array(
    np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape((3, 1, 1)), name="std"
)

graph.initializer.extend([scale, mean, std])

# Mul: input * 1/255
mul_node = helper.make_node("Mul", [input_name, "scale255"], [input_name + "_scaled"])
# Sub: - mean
sub_node = helper.make_node(
    "Sub", [input_name + "_scaled", "mean"], [input_name + "_centered"]
)
# Div: / std
div_node = helper.make_node("Div", [input_name + "_centered", "std"], [output_name])

# 重定向 graph 第一层节点输入
# 找到 Cast 节点并更新其输入
for i, node in enumerate(graph.node):
    if node.name == "graph_input_cast0":  # 根据你提供的信息
        for j, input_name_in_node in enumerate(node.input):
            if input_name_in_node == "images":
                node.input[j] = output_name
        break

# 插入节点
graph.node.insert(0, div_node)
graph.node.insert(0, sub_node)
graph.node.insert(0, mul_node)

onnx.save(model, "model/best_fp16_norm.onnx")
